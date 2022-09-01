#ifndef XRSLAM_PREINTEGRATOR_H
#define XRSLAM_PREINTEGRATOR_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>

namespace xrslam {

class Frame;

struct PreIntegrator {
    struct Delta {
        double t;
        quaternion q;
        vector<3> p;
        vector<3> v;
        matrix<15> cov; // ordered in q, p, v, bg, ba
        matrix<15> sqrt_inv_cov;
    };

    struct Jacobian {
        matrix<3> dq_dbg;
        matrix<3> dp_dbg;
        matrix<3> dp_dba;
        matrix<3> dv_dbg;
        matrix<3> dv_dba;
    };

    void reset();
    void increment(double dt, const ImuData &data, const vector<3> &bg,
                   const vector<3> &ba, bool compute_jacobian,
                   bool compute_covariance);
    bool integrate(double t, const vector<3> &bg, const vector<3> &ba,
                   bool compute_jacobian, bool compute_covariance);
    void compute_sqrt_inv_cov();

    void predict(const Frame *old_frame, Frame *new_frame);

    matrix<3> cov_w; // continuous noise covariance
    matrix<3> cov_a;
    matrix<3> cov_bg; // continuous random walk noise covariance
    matrix<3> cov_ba;

    Delta delta;
    Jacobian jacobian;

    std::vector<ImuData> data;
};

} // namespace xrslam

#endif // XRSLAM_PREINTEGRATOR_H
