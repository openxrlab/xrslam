#ifndef XRSLAM_REPROJECTION_FACTOR_H
#define XRSLAM_REPROJECTION_FACTOR_H

#include <xrslam/common.h>

namespace xrslam {

class ReprojectionErrorFactor {
  public:
    virtual ~ReprojectionErrorFactor() = default;

  protected:
    ReprojectionErrorFactor() = default;
};

class ReprojectionPriorFactor {
  public:
    virtual ~ReprojectionPriorFactor() = default;

  protected:
    ReprojectionPriorFactor() = default;
};

} // namespace xrslam

#endif // XRSLAM_REPROJECTION_FACTOR_H
