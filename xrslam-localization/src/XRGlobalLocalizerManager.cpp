#include "XRGlobalLocalizerManager.h"

namespace xrslam {
XRGlobalLocalizerManager &XRGlobalLocalizerManager::Instance() {
    static XRGlobalLocalizerManager GlobalLocalizerManagerInstance;
    return GlobalLocalizerManagerInstance;
}

XRGlobalLocalizerManager::XRGlobalLocalizerManager() {}
XRGlobalLocalizerManager::~XRGlobalLocalizerManager() {}

void XRGlobalLocalizerManager::Init(std::shared_ptr<Config> config) {
    config_ = config;
    localizer_ = std::make_unique<Localizer>(config);
}

int XRGlobalLocalizerManager::IsInitialized() {
    if (localizer_)
        return localizer_->is_initialized();
    return 0;
}

void XRGlobalLocalizerManager::QueryFrame() { localizer_->query_frame(); }

void XRGlobalLocalizerManager::QueryLocalization(XRSLAMImage *image,
                                                 XRSLAMPose *pose) {
    if (global_localization_state_ == 0)
        return;
    std::shared_ptr<xrslam::extra::OpenCvImage> opencv_image =
        std::make_shared<xrslam::extra::OpenCvImage>();
    int cols = config_->camera_resolution()[0];
    int rows = config_->camera_resolution()[1];
    opencv_image->t = image->timeStamp;
    cv::Mat img = cv::Mat(rows, cols, CV_8UC1, image->data,
                          image->stride); // gray img
    opencv_image->image = img.clone();
    opencv_image->raw = img.clone();
    Pose image_pose;
    for (int i = 0; i < 3; i++)
        image_pose.p(i) = pose->translation[i];
    image_pose.q.x() = pose->quaternion[0];
    image_pose.q.y() = pose->quaternion[1];
    image_pose.q.z() = pose->quaternion[2];
    image_pose.q.w() = pose->quaternion[3];

    localizer_->query_localization(std::shared_ptr<xrslam::Image>(opencv_image),
                                   image_pose);
}
void XRGlobalLocalizerManager::SetGlobalLocalizationState(int state) {
    global_localization_state_ = state;
}

XRSLAMPose XRGlobalLocalizerManager::TransformPose(const XRSLAMPose &pose) {
    if (global_localization_state_ == 0)
        return pose;
    Pose slam_pose;
    for (int i = 0; i < 3; i++)
        slam_pose.p(i) = pose.translation[i];
    slam_pose.q.x() = pose.quaternion[0];
    slam_pose.q.y() = pose.quaternion[1];
    slam_pose.q.z() = pose.quaternion[2];
    slam_pose.q.w() = pose.quaternion[3];

    Pose sfm_pose = localizer_->transform(slam_pose);

    XRSLAMPose out_pose;
    for (int i = 0; i < 3; i++)
        out_pose.translation[i] = sfm_pose.p(i);
    out_pose.quaternion[0] = sfm_pose.q.x();
    out_pose.quaternion[1] = sfm_pose.q.y();
    out_pose.quaternion[2] = sfm_pose.q.z();
    out_pose.quaternion[3] = sfm_pose.q.w();
    return out_pose;
}

void XRGlobalLocalizerManager::Destroy() {}

} // namespace xrslam
