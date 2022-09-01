#ifndef XRSLAM_CERES_REPROJECTION_FACTOR_H
#define XRSLAM_CERES_REPROJECTION_FACTOR_H

#include <ceres/ceres.h>
#include <xrslam/common.h>
#include <xrslam/estimation/reprojection_factor.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/map/track.h>

namespace xrslam {

class CeresReprojectionErrorFactor
    : public ReprojectionErrorFactor,
      public ceres::SizedCostFunction<2, 4, 3, 4, 3, 1> {
  public:
    CeresReprojectionErrorFactor(Frame *frame, Track *track)
        : frame(frame), track(track),
          keypoint_index(track->get_keypoint_index(frame)) {
        const vector<3> &z = frame->get_keypoint(keypoint_index);
        local_tangent.leftCols<2>() = s2_tangential_basis(z);
        local_tangent.rightCols<1>() = z;
    }

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        const_map<quaternion> q_tgt_center(parameters[0]);
        const_map<vector<3>> p_tgt_center(parameters[1]);
        const_map<quaternion> q_ref_center(parameters[2]);
        const_map<vector<3>> p_ref_center(parameters[3]);
        const double &inv_depth(*parameters[4]);

        map<vector<2>> r(residuals);

        const auto &[frame_ref, keypoint_index_ref] = track->first_keypoint();

        const vector<3> &z_ref = frame_ref->get_keypoint(keypoint_index_ref);

        const ExtrinsicParams &camera_ref = frame_ref->camera;
        const ExtrinsicParams &camera_tgt = frame->camera;

        const matrix<2> &sqrt_inv_cov = frame->sqrt_inv_cov;
        vector<3> y_ref = z_ref / inv_depth;
        vector<3> y_ref_center = camera_ref.q_cs * y_ref + camera_ref.p_cs;
        vector<3> x = q_ref_center * y_ref_center + p_ref_center;
        vector<3> y_tgt_center = q_tgt_center.conjugate() * (x - p_tgt_center);
        vector<3> y_tgt =
            camera_tgt.q_cs.conjugate() * (y_tgt_center - camera_tgt.p_cs);
        vector<3> u_tgt = local_tangent.transpose() * y_tgt;
        r = u_tgt.hnormalized();

        if (jacobians) {
            matrix<2, 3> dr_dy_tgt =
                sqrt_inv_cov * dproj_dp(u_tgt) * local_tangent.transpose();
            matrix<2, 3> dr_dy_tgt_center =
                dr_dy_tgt * camera_tgt.q_cs.conjugate().matrix();
            matrix<2, 3> dr_dx =
                dr_dy_tgt_center * q_tgt_center.conjugate().matrix();
            matrix<2, 3> dr_dy_ref_center = dr_dx * q_ref_center.matrix();
            if (jacobians[0]) {
                map<matrix<2, 4, true>> dr_dq_tgt(jacobians[0]);
                dr_dq_tgt.block<2, 3>(0, 0) =
                    dr_dy_tgt_center * hat(y_tgt_center);
                dr_dq_tgt.col(3).setZero();
            }
            if (jacobians[1]) {
                map<matrix<2, 3, true>> dr_dp_tgt(jacobians[1]);
                dr_dp_tgt = -dr_dx;
            }
            if (jacobians[2]) {
                map<matrix<2, 4, true>> dr_dq_ref(jacobians[2]);
                dr_dq_ref.block<2, 3>(0, 0) =
                    -dr_dy_ref_center * hat(y_ref_center);
                dr_dq_ref.col(3).setZero();
            }
            if (jacobians[3]) {
                map<matrix<2, 3, true>> dr_dp_ref(jacobians[3]);
                dr_dp_ref = dr_dx;
            }
            if (jacobians[4]) {
                map<matrix<2, 1, true>> dr_dinv_depth(jacobians[4]);
                dr_dinv_depth = -dr_dy_ref_center * camera_ref.q_cs.matrix() *
                                y_ref / inv_depth;
            }
        }

        r = sqrt_inv_cov * r;

        return true;
    };

    Frame *frame;
    Track *track;
    const size_t keypoint_index;

  private:
    matrix<3> local_tangent;
};

class CeresReprojectionPriorFactor : public ReprojectionPriorFactor,
                                     public ceres::SizedCostFunction<2, 4, 3> {
  public:
    CeresReprojectionPriorFactor(Frame *frame, Track *track)
        : rpefactor(frame, track) {}

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        std::array<const double *, 5> params = {
            parameters[0], parameters[1],
            rpefactor.track->first_frame()->pose.q.coeffs().data(),
            rpefactor.track->first_frame()->pose.p.data(),
            &(rpefactor.track->landmark.inv_depth)};
        if (jacobians) {
            std::array<double *, 5> jacobs = {jacobians[0], jacobians[1],
                                              nullptr, nullptr, nullptr};
            return rpefactor.Evaluate(params.data(), residuals, jacobs.data());
        } else {
            return rpefactor.Evaluate(params.data(), residuals, nullptr);
        }
    }

    CeresReprojectionErrorFactor rpefactor;
};

} // namespace xrslam

#endif // XRSLAM_CERES_REPROJECTION_FACTOR_H
