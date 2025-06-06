#ifndef XRSLAM_SLAMMANAGER_H
#define XRSLAM_SLAMMANAGER_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "xrslam/inspection.h"
#include "xrslam/core/detail.h"
#include <xrslam/utility/logger.h>
#include "xrslam/extra/opencv_image.h"
#include "xrslam/extra/yaml_config.h"
#include "XRSLAM.h"

namespace xrslam {

class Config;
class Image;

class XRSLAMManager {
  public:
    static XRSLAMManager &Instance();
    ~XRSLAMManager();

    void Init(std::shared_ptr<Config> config);
    int CheckLicense(const char *license_path, const char *product_name);

    void PushImage(XRSLAMImage *image);
    void PushAcceleration(XRSLAMAcceleration *acc);
    void PushGyroscope(XRSLAMGyroscope *gyro);

    void RunOneFrame();

    void Destroy();

    void GetResultCameraPose(XRSLAMPose *pose) const;
    void GetResultBodyPose(XRSLAMPose *pose) const;
    void GetResultState(XRSLAMState *state) const;
    void GetResultLandmarks(XRSLAMLandmarks *landmarks) const;
    void GetResultFeatures(XRSLAMFeatures *features) const;
    void GetResultBias(XRSLAMIMUBias *bias) const;
    void GetResultVersion(XRSLAMStringOutput *output) const;
    void GetInfoIntrinsics(XRSLAMIntrinsics *intrinsics) const;
    void SetViewer(SLAMWindow* viewer);
  private:
    XRSLAMManager();

  private:
    std::shared_ptr<Config> config_;
    std::unique_ptr<XRSLAM::Detail> detail_;
    std::mutex input_mutex_;
    std::shared_ptr<xrslam::Image> cur_image_;
};
} // namespace xrslam
#endif
