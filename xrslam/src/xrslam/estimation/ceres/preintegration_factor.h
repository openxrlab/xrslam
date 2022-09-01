#ifndef XRSLAM_CERES_PREINTEGRATION_FACTOR_H
#define XRSLAM_CERES_PREINTEGRATION_FACTOR_H

#include <ceres/ceres.h>
#include <xrslam/common.h>
#include <xrslam/estimation/preintegration_factor.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/map/frame.h>

namespace xrslam {

class CeresPreIntegrationErrorFactor
    : public PreIntegrationErrorFactor,
      public ceres::SizedCostFunction<15, 4, 3, 3, 3, 3, 4, 3, 3, 3, 3> {
  public:
    CeresPreIntegrationErrorFactor(Frame *frame_i, Frame *frame_j,
                                   const PreIntegrator &preintegration)
        : frame_i(frame_i), frame_j(frame_j), preintegration(preintegration) {}

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        static const vector<3> gravity = {0.0, 0.0, -XRSLAM_GRAVITY_NOMINAL};
        const_map<quaternion> q_center_i(parameters[0]);
        const_map<vector<3>> p_center_i(parameters[1]);
        const_map<vector<3>> v_i(parameters[2]);
        const_map<vector<3>> bg_i(parameters[3]);
        const_map<vector<3>> ba_i(parameters[4]);

        const_map<quaternion> q_center_j(parameters[5]);
        const_map<vector<3>> p_center_j(parameters[6]);
        const_map<vector<3>> v_j(parameters[7]);
        const_map<vector<3>> bg_j(parameters[8]);
        const_map<vector<3>> ba_j(parameters[9]);

        const PreIntegrator &pre = preintegration;
        const ExtrinsicParams &imu_i = frame_i->imu;
        const ExtrinsicParams &imu_j = frame_j->imu;
        const vector<3> &ba_i_0 = frame_i->motion.ba;
        const vector<3> &bg_i_0 = frame_i->motion.bg;

        const quaternion q_i = q_center_i * imu_i.q_cs;
        const vector<3> p_i = p_center_i + q_center_i * imu_i.p_cs;
        const quaternion q_j = q_center_j * imu_j.q_cs;
        const vector<3> p_j = p_center_j + q_center_j * imu_j.p_cs;

        const double &dt = pre.delta.t;
        const quaternion &dq = pre.delta.q;
        const vector<3> &dp = pre.delta.p;
        const vector<3> &dv = pre.delta.v;
        const vector<3> dbg = bg_i - bg_i_0;
        const vector<3> dba = ba_i - ba_i_0;

        const matrix<3> &dq_dbg = pre.jacobian.dq_dbg;
        const matrix<3> &dp_dbg = pre.jacobian.dp_dbg;
        const matrix<3> &dp_dba = pre.jacobian.dp_dba;
        const matrix<3> &dv_dbg = pre.jacobian.dv_dbg;
        const matrix<3> &dv_dba = pre.jacobian.dv_dba;

        map<vector<15>> r(residuals);
        r.segment<3>(ES_Q) = logmap((dq * expmap(dq_dbg * dbg)).conjugate() *
                                    q_i.conjugate() * q_j);
        r.segment<3>(ES_P) =
            q_i.conjugate() * (p_j - p_i - dt * v_i - 0.5 * dt * dt * gravity) -
            (dp + dp_dbg * dbg + dp_dba * dba);
        r.segment<3>(ES_V) = q_i.conjugate() * (v_j - v_i - dt * gravity) -
                             (dv + dv_dbg * dbg + dv_dba * dba);
        r.segment<3>(ES_BG) = bg_j - bg_i;
        r.segment<3>(ES_BA) = ba_j - ba_i;

        if (jacobians) {
            if (jacobians[0]) {
                map<matrix<15, 4, true>> dr_dq_i(jacobians[0]);
                dr_dq_i.setZero();
                dr_dq_i.block<3, 3>(ES_Q, 0) =
                    -right_jacobian(r.segment<3>(ES_Q)).inverse() *
                    q_j.conjugate().matrix() * q_center_i.matrix();
                dr_dq_i.block<3, 3>(ES_P, 0) =
                    imu_i.q_cs.conjugate().matrix() *
                    hat(q_center_i.conjugate() * (p_j - p_center_i - dt * v_i -
                                                  0.5 * dt * dt * gravity));
                dr_dq_i.block<3, 3>(ES_V, 0) =
                    imu_i.q_cs.conjugate().matrix() *
                    hat(q_center_i.conjugate() * (v_j - v_i - dt * gravity));
                dr_dq_i = pre.delta.sqrt_inv_cov * dr_dq_i;
            }
            if (jacobians[1]) {
                map<matrix<15, 3, true>> dr_dp_i(jacobians[1]);
                dr_dp_i.setZero();
                dr_dp_i.block<3, 3>(ES_P, 0) = -q_i.conjugate().matrix();
                dr_dp_i = pre.delta.sqrt_inv_cov * dr_dp_i;
            }
            if (jacobians[2]) {
                map<matrix<15, 3, true>> dr_dv_i(jacobians[2]);
                dr_dv_i.setZero();
                dr_dv_i.block<3, 3>(ES_P, 0) = -dt * q_i.conjugate().matrix();
                dr_dv_i.block<3, 3>(ES_V, 0) = -q_i.conjugate().matrix();
                dr_dv_i = pre.delta.sqrt_inv_cov * dr_dv_i;
            }
            if (jacobians[3]) {
                map<matrix<15, 3, true>> dr_dbg_i(jacobians[3]);
                dr_dbg_i.setZero();
                dr_dbg_i.block<3, 3>(ES_Q, 0) =
                    -right_jacobian(r.segment<3>(ES_Q)).inverse() *
                    expmap(r.segment<3>(ES_Q)).conjugate().matrix() *
                    right_jacobian(dq_dbg * dbg) * dq_dbg;
                dr_dbg_i.block<3, 3>(ES_P, 0) = -dp_dbg;
                dr_dbg_i.block<3, 3>(ES_V, 0) = -dv_dbg;
                dr_dbg_i.block<3, 3>(ES_BG, 0) = -matrix<3>::Identity();
                dr_dbg_i = pre.delta.sqrt_inv_cov * dr_dbg_i;
            }
            if (jacobians[4]) {
                map<matrix<15, 3, true>> dr_dba_i(jacobians[4]);
                dr_dba_i.setZero();
                dr_dba_i.block<3, 3>(ES_P, 0) = -dp_dba;
                dr_dba_i.block<3, 3>(ES_V, 0) = -dv_dba;
                dr_dba_i.block<3, 3>(ES_BA, 0) = -matrix<3>::Identity();
                dr_dba_i = pre.delta.sqrt_inv_cov * dr_dba_i;
            }
            if (jacobians[5]) {
                map<matrix<15, 4, true>> dr_dq_j(jacobians[5]);
                dr_dq_j.setZero();
                dr_dq_j.block<3, 3>(ES_Q, 0) =
                    right_jacobian(r.segment<3>(ES_Q)).inverse() *
                    imu_j.q_cs.conjugate().matrix();
                dr_dq_j.block<3, 3>(ES_P, 0) = -q_i.conjugate().matrix() *
                                               q_center_j.matrix() *
                                               hat(imu_j.p_cs);
                dr_dq_j = pre.delta.sqrt_inv_cov * dr_dq_j;
            }
            if (jacobians[6]) {
                map<matrix<15, 3, true>> dr_dp_j(jacobians[6]);
                dr_dp_j.setZero();
                dr_dp_j.block<3, 3>(ES_P, 0) = q_i.conjugate().matrix();
                dr_dp_j = pre.delta.sqrt_inv_cov * dr_dp_j;
            }
            if (jacobians[7]) {
                map<matrix<15, 3, true>> dr_dv_j(jacobians[7]);
                dr_dv_j.setZero();
                dr_dv_j.block<3, 3>(ES_V, 0) = q_i.conjugate().matrix();
                dr_dv_j = pre.delta.sqrt_inv_cov * dr_dv_j;
            }
            if (jacobians[8]) {
                map<matrix<15, 3, true>> dr_dbg_j(jacobians[8]);
                dr_dbg_j.setZero();
                dr_dbg_j.block<3, 3>(ES_BG, 0).setIdentity();
                dr_dbg_j = pre.delta.sqrt_inv_cov * dr_dbg_j;
            }
            if (jacobians[9]) {
                map<matrix<15, 3, true>> dr_dba_j(jacobians[9]);
                dr_dba_j.setZero();
                dr_dba_j.block<3, 3>(ES_BA, 0).setIdentity();
                dr_dba_j = pre.delta.sqrt_inv_cov * dr_dba_j;
            }
        }

        r = pre.delta.sqrt_inv_cov * r;

        return true;
    }

    Frame *frame_i;
    Frame *frame_j;
    const PreIntegrator &preintegration;
};

class CeresPreIntegrationPriorFactor
    : public PreIntegrationPriorFactor,
      public ceres::SizedCostFunction<15, 4, 3, 3, 3, 3> {
  public:
    CeresPreIntegrationPriorFactor(Frame *frame_i, Frame *frame_j,
                                   const PreIntegrator &preintegration)
        : piefactor(frame_i, frame_j, preintegration) {}

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        std::array<const double *, 10> params = {
            piefactor.frame_i->pose.q.coeffs().data(),
            piefactor.frame_i->pose.p.data(),
            piefactor.frame_i->motion.v.data(),
            piefactor.frame_i->motion.bg.data(),
            piefactor.frame_i->motion.ba.data(),
            parameters[0],
            parameters[1],
            parameters[2],
            parameters[3],
            parameters[4]};
        if (jacobians) {
            std::array<double *, 10> jacobs = {
                nullptr,      nullptr,      nullptr,      nullptr,
                nullptr,      jacobians[0], jacobians[1], jacobians[2],
                jacobians[3], jacobians[4]};
            return piefactor.Evaluate(params.data(), residuals, jacobs.data());
        } else {
            return piefactor.Evaluate(params.data(), residuals, nullptr);
        }
    }

    CeresPreIntegrationErrorFactor piefactor;
};

} // namespace xrslam

#endif // XRSLAM_CERES_PREINTEGRATION_FACTOR_H
