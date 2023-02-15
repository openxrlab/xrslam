#include "XRGlobalLocalizerManager.h"

void XRGlobalLocalizerCreate(const char *slam_config_path,
                             const char *device_config_path) {
    std::shared_ptr<xrslam::extra::YamlConfig> yaml_config =
        std::make_shared<xrslam::extra::YamlConfig>(slam_config_path,
                                                    device_config_path);

    xrslam::XRGlobalLocalizerManager::Instance().Init(yaml_config);
}

void XRGlobalLocalizerQueryFrame() {
    xrslam::XRGlobalLocalizerManager::Instance().QueryFrame();
}

int XRGlobalLocalizerIsInitialized() {
    return xrslam::XRGlobalLocalizerManager::Instance().IsInitialized();
}

void XRGlobalLocalizerEnable(int state) {
    xrslam::XRGlobalLocalizerManager::Instance().SetGlobalLocalizationState(
        state);
}

void XRGlobalLocalizerQueryLocalization(XRSLAMImage *image, XRSLAMPose *pose) {
    xrslam::XRGlobalLocalizerManager::Instance().QueryLocalization(image, pose);
}

XRSLAMPose XRGlobalLocalizerTransformPose(const XRSLAMPose &pose) {
    return xrslam::XRGlobalLocalizerManager::Instance().TransformPose(pose);
}

void XRSLAMGlobalLocalizerDestory() {
    xrslam::XRGlobalLocalizerManager::Instance().Destroy();
}
