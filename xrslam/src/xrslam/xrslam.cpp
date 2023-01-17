#include <xrslam/bridge.h>
#include <xrslam/xrslam.h>
#include <xrslam/extra/yaml_config.h>
#include <xrslam/extra/opencv_image.h>

namespace xrslam {
    // public ptr
    static std::unique_ptr<xrslam::XRSLAM> xrslam = nullptr;
    static std::shared_ptr<xrslam::extra::YamlConfig> yaml_config = nullptr;
    static std::shared_ptr<xrslam::Pose> pose = nullptr;
    static std::shared_ptr<xrslam::Image> image = nullptr;

int XRSLAMCreate(
    const char* slam_config_path,       // slam configuration file path
    const char* device_config_path      // device configuration file path
    )
{
    yaml_config = std::make_shared<xrslam::extra::YamlConfig>(slam_config_path, device_config_path);
    xrslam = std::make_unique<xrslam::XRSLAM>(yaml_config);
    return 1;
}

/* @brief: push sensor data
*  @input param: sensor type, sensor data
* */
void XRSLAMPushSensorData(
    XRSLAMSensorType sensor_type,       // sensor type
    void *sensor_data               // sensor data
    )
{
    switch (sensor_type) {
        case XRSLAM_SENSOR_CAMERA: {
            XRSLAMImage *pointer=(XRSLAMImage*)sensor_data;
            xrslam::extra::OpenCvImage cvImage = xrslam::extra::OpenCvImage();
            int rows = yaml_config->camera_resolution()[0];
            int cols = yaml_config->camera_resolution()[1];
            cvImage.t = pointer->timeStamp;
            cvImage.image = cv::Mat(rows, cols, CV_32FC1, pointer->data);
            cvImage.raw = cvImage.image.clone();
            std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image = 
                std::make_shared<xrslam::extra::OpenCvImage>();
            opencv_image->image = cvImage.image;
            static std::shared_ptr<xrslam::Image> image = std::shared_ptr<xrslam::Image>(opencv_image);

        } break;
        case XRSLAM_SENSOR_ACCELERATION: {
            XRSLAMAcceleration *pointer=(XRSLAMAcceleration*)sensor_data;
            xrslam->track_accelerometer(pointer->timestamp, 
                                        pointer->data[0], 
                                        pointer->data[1], 
                                        pointer->data[2]);
        } break;
        case XRSLAM_SENSOR_GYROSCOPE: {
            XRSLAMGyroscope *pointer=(XRSLAMGyroscope*)sensor_data;
            xrslam->track_gyroscope(pointer->timestamp, 
                                    pointer->data[0], 
                                    pointer->data[1], 
                                    pointer->data[2]);
        } break;
        default: {} break;
    }
}

/* @brief: end one frame input and run slam
* */
void XRSLAMRunOneFrame()
{
    auto result = xrslam->run_one_frame(image);
    pose = std::make_shared<xrslam::Pose>(result); //error
}

/* @brief: get SLAM tracking result at time t
* @input param: time t
* @return value: slam tracking result
* */
void XRSLAMGetResult(
    XRSLAMResultType result_type,   // result type
    void *result_data               // result data
    )
{
    switch (result_type) {
        case XRSLAM_RESULT_STATE: {
            if (!pose->q.coeffs().isZero()) {
                int result = XRSLAM_STATE_TRACKING_SUCCESS;
                result_data = &result;
            }else{
                int result = XRSLAM_STATE_TRACKING_FAIL;
                result_data = &result;
            }
        }break;
        case XRSLAM_RESULT_BODY_POSE: {
            XRSLAMPose pose_o;
            pose_o.quaternion[0] = pose->q.x();
            pose_o.quaternion[1] =pose->q.y();
            pose_o.quaternion[2] =pose->q.z();
            pose_o.quaternion[3] =pose->q.w();

            pose_o.translation[0] = pose->p.x();
            pose_o.translation[1] = pose->p.y();
            pose_o.translation[2] = pose->p.z();

            pose_o.timestamp = image->t;
            result_data = &pose_o;
        } break;
        default: {} break;
    }
}

/* @brief: destroy SLAM system
* */
void XRSLAMDestroy()
{
    xrslam = nullptr;
    pose = nullptr;
    image = nullptr;
}

} // namespace xrslam