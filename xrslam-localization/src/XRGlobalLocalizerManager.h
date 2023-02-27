#ifndef XR_GLOBAL_LOCALIZER_MANAGER_H
#define XR_GLOBAL_LOCALIZER_MANAGER_H

#include <iostream>
#include <opencv2/opencv.hpp>

#include "xrslam/extra/opencv_image.h"
#include "xrslam/extra/yaml_config.h"
#include "xrslam/localizer/localizer.h"

#include "XRGlobalLocalizer.h"
#include "XRSLAM.h"

namespace xrslam {

class Config;
class Image;

class XRGlobalLocalizerManager {
  public:
    static XRGlobalLocalizerManager &Instance();
    ~XRGlobalLocalizerManager();

    void Init(std::shared_ptr<Config> config);
    int IsInitialized();
    void QueryFrame();
    void QueryLocalization(XRSLAMImage *image, XRSLAMPose *pose);
    void SetGlobalLocalizationState(int state);
    XRSLAMPose TransformPose(const XRSLAMPose &pose);
    void Destroy();

  private:
    XRGlobalLocalizerManager();

  private:
    std::shared_ptr<Config> config_;
    std::shared_ptr<xrslam::Image> cur_image_;
    std::unique_ptr<Localizer> localizer_;
    int global_localization_state_;
};

} // namespace xrslam

#endif // XR_GLOBAL_LOCALIZER_MANAGER_H