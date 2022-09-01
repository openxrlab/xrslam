#ifndef XRSLAM_PREINTEGRATION_FACTOR_H
#define XRSLAM_PREINTEGRATION_FACTOR_H

#include <xrslam/common.h>

namespace xrslam {

class PreIntegrationErrorFactor {
  public:
    virtual ~PreIntegrationErrorFactor() = default;

  protected:
    PreIntegrationErrorFactor() = default;
};

class PreIntegrationPriorFactor {
  public:
    virtual ~PreIntegrationPriorFactor() = default;

  protected:
    PreIntegrationPriorFactor() = default;
};

} // namespace xrslam

#endif // XRSLAM_PREINTEGRATION_FACTOR_H
