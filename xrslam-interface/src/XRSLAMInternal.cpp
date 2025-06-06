#include "XRSLAMManager.h"
#include "xrslam/xrslam.h"

int XRSLAMCreate(
    const char *slam_config_path,   // slam configuration file path
    const char *device_config_path, // device configuration file path
    const char *license_path, const char *product_name, void **config) {
    if (xrslam::XRSLAMManager::Instance().CheckLicense(license_path,
                                                       product_name) == 0)
        return 0;

    std::shared_ptr<xrslam::extra::YamlConfig> yaml_config =
        std::make_shared<xrslam::extra::YamlConfig>(slam_config_path,
                                                    device_config_path);
    xrslam::XRSLAMManager::Instance().Init(yaml_config);
    *config = static_cast<void *>(yaml_config.get());
    return 1;
}

void XRSLAMPushSensorData(XRSLAMSensorType sensor_type, // sensor type
                          void *sensor_data             // sensor data
) {
    switch (sensor_type) {
    case XRSLAM_SENSOR_CAMERA:
        xrslam::XRSLAMManager::Instance().PushImage(
            static_cast<XRSLAMImage *>(sensor_data));
        break;
    case XRSLAM_SENSOR_ACCELERATION:
        xrslam::XRSLAMManager::Instance().PushAcceleration(
            static_cast<XRSLAMAcceleration *>(sensor_data));
        break;
    case XRSLAM_SENSOR_GYROSCOPE:
        xrslam::XRSLAMManager::Instance().PushGyroscope(
            static_cast<XRSLAMGyroscope *>(sensor_data));
        break;
    case XRSLAM_SENSOR_DEPTH_CAMERA:
    case XRSLAM_SENSOR_GRAVITY:
    case XRSLAM_SENSOR_ROTATION_VECTOR:
    case XRSLAM_SENSOR_UNKNOWN:
    default:
        break;
    }
}

void XRSLAMSetViewer(void* viewer){
    SLAMWindow* viewer_ptr = static_cast<SLAMWindow*>(viewer);
    xrslam::XRSLAMManager::Instance().SetViewer(viewer_ptr);
}

void XRSLAMRunOneFrame() { xrslam::XRSLAMManager::Instance().RunOneFrame(); }

void XRSLAMGetResult(XRSLAMResultType result_type, // result type
                     void *result_data             // result data
) {
    switch (result_type) {
    case XRSLAM_RESULT_BODY_POSE:
        xrslam::XRSLAMManager::Instance().GetResultBodyPose(
            static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_CAMERA_POSE:
        xrslam::XRSLAMManager::Instance().GetResultCameraPose(
            static_cast<XRSLAMPose *>(result_data));
        break;
    case XRSLAM_RESULT_STATE:
        xrslam::XRSLAMManager::Instance().GetResultState(
            static_cast<XRSLAMState *>(result_data));
        break;
    case XRSLAM_RESULT_LANDMARKS:
        xrslam::XRSLAMManager::Instance().GetResultLandmarks(
            static_cast<XRSLAMLandmarks *>(result_data));
        break;
    case XRSLAM_RESULT_FEATURES:
        xrslam::XRSLAMManager::Instance().GetResultFeatures(
            static_cast<XRSLAMFeatures *>(result_data));
        break;
    case XRSLAM_RESULT_BIAS:
        xrslam::XRSLAMManager::Instance().GetResultBias(
            static_cast<XRSLAMIMUBias *>(result_data));
        break;
    case XRSLAM_RESULT_VERSION:
        xrslam::XRSLAMManager::Instance().GetResultVersion(
            static_cast<XRSLAMStringOutput *>(result_data));
        break;
    case XRSLAM_INFO_INTRINSICS:
        xrslam::XRSLAMManager::Instance().GetInfoIntrinsics(
            static_cast<XRSLAMIntrinsics *>(result_data));
        break;
    case XRSLAM_RESULT_DEBUG_LOGS:
    case XRSLAM_RESULT_UNKNOWN:
    default:
        break;
    }
}

void XRSLAMDestroy() { xrslam::XRSLAMManager::Instance().Destroy(); }
