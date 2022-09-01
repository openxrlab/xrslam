#ifndef XRSLAM_ROTATION_FACTOR_H
#define XRSLAM_ROTATION_FACTOR_H

#include <xrslam/common.h>

namespace xrslam {

class RotationPriorFactor {
  public:
    virtual ~RotationPriorFactor() = default;

  protected:
    RotationPriorFactor() = default;
};

} // namespace xrslam

#endif // XRSLAM_ROTATION_FACTOR_H
