//#import <thread>
#import <opencv2/opencv.hpp>
#import "XRSLAM.h"
#import <UIKit/UIKit.h>
#import <SceneKit/SceneKit.h>
#import <Foundation/Foundation.h>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdocumentation"
#import <xrslam/xrslam.h>
#import <xrslam/extra/opencv_image.h>
#import <opencv2/imgcodecs/ios.h>
#import "XRSLAMIosConfig.h"
#pragma clang diagnostic pop

id xrslam_instance;

struct OutputState {
    double t;
    UIImage *image;
    ::xrslam::OutputPose pose;
};

@implementation XRSLAM {
    std::unique_ptr<::xrslam::XRSLAM> xrslam;
    std::shared_ptr<::xrslam::Config> xrslam_config;
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

        NSString *config_path = [[NSBundle mainBundle]
            pathForResource:[@"configs/" stringByAppendingString:model]
                     ofType:@"yaml"];

        if (![[NSFileManager defaultManager] fileExistsAtPath:config_path]) {
            std::cout << "XRSLAM Do NOT Support This Device!" << std::endl;
            return;
        }

        NSString *common_config_path =
            [[NSBundle mainBundle] pathForResource:@"configs/params"
                                            ofType:@"yaml"];
        NSString *common_config_content =
            [NSString stringWithContentsOfFile:common_config_path
                                      encoding:NSUTF8StringEncoding
                                         error:NULL];
        NSString *iphone_config_content =
            [NSString stringWithContentsOfFile:config_path
                                      encoding:NSUTF8StringEncoding
                                         error:NULL];

        std::shared_ptr<xrslam::CommonIosConfig> common_config =
            std::make_shared<xrslam::CommonIosConfig>(
                [common_config_content UTF8String]);
        std::shared_ptr<xrslam::XRSLAMIosConfig> xrslam_config =
            std::make_shared<xrslam::XRSLAMIosConfig>(
                [iphone_config_content UTF8String], common_config);
        xrslam = std::make_unique<::xrslam::XRSLAM>(xrslam_config);
        ::xrslam::matrix<3> K = xrslam_config->camera_intrinsic();
        fov = 360.0 / 3.1415926535897 * atan2(K(0, 2), K(0, 0));
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

    std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image =
        std::make_shared<xrslam::extra::OpenCvImage>();
    opencv_image->t = t;
    opencv_image->image = cvimage.clone();
    opencv_image->raw = cvimage.clone();

    xrslam->track_camera(opencv_image);
    auto [timestamp, pose_state] = xrslam->get_latest_camera_state();
    latest_state.t = timestamp;
    latest_state.pose = pose_state;

    [self updateImage:t image:uiimage];
}

- (void)trackGyroscope:(double)t x:(double)x y:(double)y z:(double)z {
    xrslam->track_gyroscope(t, x, y, z);
}

- (void)trackAccelerometer:(double)t x:(double)x y:(double)y z:(double)z {
    xrslam->track_accelerometer(t, x, y, z);
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
    return xrslam->create_virtual_object();
}

- (void)updateVirtualObject:(size_t)object_id {
    some_virtual_object_pose = xrslam->get_virtual_object_pose_by_id(object_id);
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
    system_state = xrslam->get_system_state();

    if (system_state == ::xrslam::SysState::SYS_INITIALIZING) {
        return SysState::SYS_INITIALIZING;
    } else if (system_state == ::xrslam::SysState::SYS_TRACKING) {
        return SysState::SYS_TRACKING;
    } else if (system_state == ::xrslam::SysState::SYS_CRASH) {
        return SysState::SYS_CRASH;
    } else {
        return SysState::SYS_UNKNOWN;
    }
}

- (void)enable_global_localization {
    xrslam->enable_global_localization();
}

- (void)disable_global_localization {
    xrslam->disable_global_localization();
}

- (bool)global_localization_initialized {
    return xrslam->global_localization_initialized();
}

- (void)query_frame {
    xrslam->query_frame();
}

- (NSMutableArray *)get_logger_message {
    id nsstrings = [NSMutableArray new];

    std::vector<std::string> strs = xrslam->get_logger_message();
    for (auto s : strs) {
        id nsstr = [NSString stringWithUTF8String:s.c_str()];
        [nsstrings addObject:nsstr];
    }
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
