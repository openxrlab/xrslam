//#import <thread>
#import <opencv2/opencv.hpp>
#import "XRSLAM.h"
#import "XRSLAM2.h"
#import "XRGlobalLocalizer.h"
#import <UIKit/UIKit.h>
#import <SceneKit/SceneKit.h>
#import <Foundation/Foundation.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#import <opencv2/imgcodecs/ios.h>
#import <Eigen/Eigen>
#pragma clang diagnostic pop

template <int Rows = Eigen::Dynamic, int Cols = Rows, bool UseRowMajor = false,
          typename T = double>
using matrix = typename std::conditional<
    Rows != 1 && Cols != 1,
    Eigen::Matrix<T, Rows, Cols,
                  UseRowMajor ? Eigen::RowMajor : Eigen::ColMajor>,
    Eigen::Matrix<T, Rows, Cols>>::type;

template <int Dimension = Eigen::Dynamic, bool RowVector = false,
          typename T = double>
using vector =
    typename std::conditional<RowVector, matrix<1, Dimension, false, T>,
                              matrix<Dimension, 1, false, T>>::type;

using quaternion = Eigen::Quaternion<double>;

namespace xrslam {
struct Pose {
    Pose() {
        q.setIdentity();
        p.setZero();
    }
    quaternion q;
    vector<3> p;
};

using OutputPose = Pose;
using uchar = unsigned char;

struct OutputState {
    double t;
    quaternion q;
    vector<3> p;
    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};

struct OutputObject {
    quaternion q;
    vector<3> p;
    int isolated;
};

enum SysState { SYS_INITIALIZING = 0, SYS_TRACKING, SYS_CRASH, SYS_UNKNOWN };
}

id xrslam_instance;

struct OutputState {
    double t;
    UIImage *image;
    ::xrslam::OutputPose pose;
};

@implementation XRSLAM {
    double fov;
    ::xrslam::SysState system_state;
    OutputState latest_state;
    std::list<OutputState> state_list;
    ::xrslam::OutputObject some_virtual_object_pose;
    std::vector<double> track_time;
    UIImage *uiimage;
    cv::Mat cvimage;
}

- (id)init:(NSString *)model {
    if (self = [super init]) {
        std::cout << "Device: " << [model UTF8String] << std::endl;
        {
            NSString *slam_params =
                [[NSBundle mainBundle] pathForResource:@"configs/slam_params"
                                                ofType:@"yaml"];
            NSString* slam_config_content = [NSString stringWithContentsOfFile:slam_params encoding:NSUTF8StringEncoding error:NULL];
            NSString *device_params =
                [[NSBundle mainBundle] pathForResource:[@"configs/" stringByAppendingString:model]
                     ofType:@"yaml"];
            if (![[NSFileManager defaultManager] fileExistsAtPath:device_params]) {
                std::cout << "XRSLAM Do NOT Support This Device!" << std::endl;
                return;
            }
            NSString* device_config_content = [NSString stringWithContentsOfFile:device_params encoding:NSUTF8StringEncoding error:NULL];
            NSString *license_file =
                [[NSBundle mainBundle] pathForResource:@"configs/SENSESLAMSDK_165BA1A4-F959-4790-A891-C85DBF5D26EA"
                                                ofType:@"lic"];

            int num = XRSLAMCreate([slam_config_content UTF8String], [device_config_content UTF8String], [license_file UTF8String], "SenseSLAMSDK");
            XRGlobalLocalizerCreate([slam_config_content UTF8String], [device_config_content UTF8String]);
            std::cout<<"init xr slam: "<<num << std::endl;
        }
        fov = 360.0 / 3.1415926535897 * atan2(314.618580309, 483.302341374);
    }
    xrslam_instance = self;
    return self;
}

- (void)dealloc {
}

- (void)processBuffer:(CMSampleBufferRef)buffer {
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(buffer);
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);

    int w = (int)CVPixelBufferGetWidth(pixelBuffer);
    int h = (int)CVPixelBufferGetHeight(pixelBuffer);
    int pixelPerRow = (int)CVPixelBufferGetBytesPerRow(pixelBuffer);
    unsigned char *baseAddress =
        (unsigned char *)CVPixelBufferGetBaseAddress(pixelBuffer);

    cv::Mat raw_image = cv::Mat(h, w, CV_8UC4, baseAddress, pixelPerRow);

    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);

    cv::Mat rgb_image;
    cv::cvtColor(raw_image, cvimage, cv::COLOR_BGRA2GRAY);
    cv::cvtColor(raw_image, rgb_image, cv::COLOR_BGRA2RGB);

    uiimage = MatToUIImage(rgb_image);
}

- (void)trackCamera:(double)t buffer:(CMSampleBufferRef)buffer {

    [self processBuffer:buffer];

    {
        XRSLAMImage image;
        image.camera_id = 0;
        image.ext = nullptr;
        image.timeStamp = t;
        image.data = cvimage.data;
        image.stride = cvimage.step[0];
        XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);
        XRSLAMRunOneFrame();
        
        XRSLAMState result;
        XRSLAMGetResult(XRSLAM_RESULT_STATE, &result);
        if(result == XRSLAM_STATE_TRACKING_SUCCESS){
            XRSLAMPose pose_c;
            XRSLAMGetResult(XRSLAM_RESULT_CAMERA_POSE, &pose_c);
            XRGlobalLocalizerQueryLocalization(&image, &pose_c);
            
            latest_state.t = pose_c.timestamp;
            XRSLAMPose pose_sfm = XRGlobalLocalizerTransformPose(pose_c);
            latest_state.pose.p[0] = pose_sfm.translation[0];
            latest_state.pose.p[1] = pose_sfm.translation[1];
            latest_state.pose.p[2] = pose_sfm.translation[2];
            latest_state.pose.q.x() = pose_sfm.quaternion[0];
            latest_state.pose.q.y() = pose_sfm.quaternion[1];
            latest_state.pose.q.z() = pose_sfm.quaternion[2];
            latest_state.pose.q.w() = pose_sfm.quaternion[3]; 
        }
    }

    [self updateImage:t image:uiimage];
}

- (void)trackGyroscope:(double)t x:(double)x y:(double)y z:(double)z {
    {
        XRSLAMGyroscope gyro;
        gyro.timestamp = t;
        gyro.data[0] = x; gyro.data[1] = y; gyro.data[2] = z;
        XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyro);
    }
}

- (void)trackAccelerometer:(double)t x:(double)x y:(double)y z:(double)z {
    {
        XRSLAMAcceleration acc;
        acc.timestamp = t;
        acc.data[0] = x; acc.data[1] = y; acc.data[2] = z;
        XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc);
    }
}

- (double)getFOV {
    return fov;
}

- (SCNVector3)getCameraPosition {
    return SCNVector3Make(-latest_state.pose.p.y(), -latest_state.pose.p.x(),
                          -latest_state.pose.p.z());
}

- (SCNQuaternion)getCameraRotation {
    return SCNVector4Make(-latest_state.pose.q.y(), -latest_state.pose.q.x(),
                          -latest_state.pose.q.z(), latest_state.pose.q.w());
}

- (UIImage *)getLatestImage {
    return latest_state.image;
}

- (UIImage *)getCurrentImage {
    return uiimage;
}

- (size_t)createVirtualObject {
    static int _id = 0;
    if([self get_system_state] == SysState::SYS_TRACKING)
    {
        Eigen::Vector3f twc(latest_state.pose.p(0),latest_state.pose.p(1),latest_state.pose.p(2));
        Eigen::Quaternionf pose_q(latest_state.pose.q.w(), latest_state.pose.q.x(),latest_state.pose.q.y(),latest_state.pose.q.z()); //wxyz
        Eigen::Matrix3f Rcw = pose_q.toRotationMatrix().transpose();
        Eigen::Vector3f tcw = -Rcw * twc;
        float keypoint_radius = 0.2;
        Eigen::Vector3f group_origin = {0.0, 0.0, 0.0};
        XRSLAMLandmarks landmarks;
        XRSLAMGetResult(XRSLAM_RESULT_LANDMARKS, &landmarks);
        int near_landmarks = 0;
        for (int i = 0; i < landmarks.num_landmarks; ++i) {
            Eigen::Vector3f cur_lk = Eigen::Vector3f(landmarks.landmarks[i].x, landmarks.landmarks[i].y, landmarks.landmarks[i].z);
            Eigen::Vector3f pc = Rcw * cur_lk + tcw;
            pc(0)= pc(0)/pc(2);
            pc(1)= pc(1)/pc(2);
            if(pc(0)<-keypoint_radius || pc(0) > keypoint_radius || pc(1)<-keypoint_radius || pc(1) > keypoint_radius)
            {
                continue;
            }
            group_origin += cur_lk;
            near_landmarks++;
        }
        if(near_landmarks <5) return -1;
        group_origin /= near_landmarks;
        Eigen::Vector3f central_ray = ( twc - group_origin).normalized();
        Eigen::Vector3f up = {0.0, 0.0, 1.0};
        Eigen::Vector3f right = up.cross(central_ray).normalized();
        Eigen::Vector3f backward = right.cross(up).normalized();
        Eigen::Matrix3f R;
        R.col(0) = -up;
        R.col(1) = right;
        R.col(2) = backward;
        Eigen::Quaternionf obj_q;
        obj_q = R;
        XRSLAMPose slam_obj_pose;
        slam_obj_pose.translation[0] = group_origin(0);
        slam_obj_pose.translation[1] = group_origin(1);
        slam_obj_pose.translation[2] = group_origin(2);
        slam_obj_pose.quaternion[0] = obj_q.x();
        slam_obj_pose.quaternion[1] = obj_q.y();
        slam_obj_pose.quaternion[2] = obj_q.z();
        slam_obj_pose.quaternion[3] = obj_q.w();
        XRSLAMPose sfm_obj_pose = XRGlobalLocalizerTransformPose(slam_obj_pose);
        some_virtual_object_pose.p.x() = sfm_obj_pose.translation[0];
        some_virtual_object_pose.p.y() = sfm_obj_pose.translation[1];
        some_virtual_object_pose.p.z() = sfm_obj_pose.translation[2];
        some_virtual_object_pose.q.x() = sfm_obj_pose.quaternion[0];
        some_virtual_object_pose.q.y() = sfm_obj_pose.quaternion[1];
        some_virtual_object_pose.q.z() = sfm_obj_pose.quaternion[2];
        some_virtual_object_pose.q.w() = sfm_obj_pose.quaternion[3];
        some_virtual_object_pose.isolated = 0;
        return _id++;
    }
    return -1;
}

- (void)updateVirtualObject:(size_t)object_id {
}

- (SCNVector3)getVirtualObjectPosition {
    return SCNVector3Make(-some_virtual_object_pose.p.y(),
                          -some_virtual_object_pose.p.x(),
                          -some_virtual_object_pose.p.z());
}

- (SCNQuaternion)getVirtualObjectRotation {
    return SCNVector4Make(
        -some_virtual_object_pose.q.y(), -some_virtual_object_pose.q.x(),
        -some_virtual_object_pose.q.z(), some_virtual_object_pose.q.w());
}

- (int)getVirtualObjectIsolated {
    return some_virtual_object_pose.isolated;
}

- (SysState)get_system_state {
    {
        XRSLAMState result;
        XRSLAMGetResult(XRSLAM_RESULT_STATE, &result);
        if(result == XRSLAM_STATE_INITIALIZING){
            return SysState::SYS_INITIALIZING;
        } else if (result == XRSLAM_STATE_TRACKING_SUCCESS) {
            return SysState::SYS_TRACKING;
        } else {
            return SysState::SYS_UNKNOWN;
        }
    }
}

- (void)enable_global_localization {
    XRGlobalLocalizerEnable(1);
}

- (void)disable_global_localization {
    XRGlobalLocalizerEnable(0);
}

- (int)global_localization_initialized {
    return XRGlobalLocalizerIsInitialized();
}

- (void)query_frame {
    XRGlobalLocalizerQueryFrame();
}

- (NSMutableArray *)get_logger_message {
    id nsstrings = [NSMutableArray new];
    return nsstrings;
}

- (void)updateImage:(double)t image:(UIImage *)image {

    OutputState state;
    state.t = t;
    state.image = image;

    if (latest_state.t > 0) {
        state_list.push_back(state);

        while (!state_list.empty() &&
               latest_state.t - state_list.front().t > 1.0e-5)
            state_list.pop_front();
        latest_state.image = state_list.front().image;

    } else {
        latest_state.image = image;
    }
}

@end
