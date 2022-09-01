#ifndef XRSLAM_CERES_ROTATION_FACTOR_H
#define XRSLAM_CERES_ROTATION_FACTOR_H

#include <xrslam/common.h>
#include <xrslam/estimation/rotation_factor.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/track.h>

namespace xrslam {

class CeresRotationPriorFactor : public RotationPriorFactor,
                                 public ceres::SizedCostFunction<2, 4> {
  public:
    CeresRotationPriorFactor(Frame *frame, Track *track)
        : frame(frame), track(track),
          keypoint_index(track->get_keypoint_index(frame)) {
        const vector<3> &z = frame->get_keypoint(keypoint_index);
        local_tangent.leftCols<2>() = s2_tangential_basis(z);
        local_tangent.rightCols<1>() = z;
    }

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        const_map<quaternion> q_tgt_center(parameters[0]);
        map<vector<2>> r(residuals);

        const auto &[frame_ref, keypoint_index_ref] = track->first_keypoint();
        const quaternion &q_ref_center = frame_ref->pose.q;
        const ExtrinsicParams &camera_ref = frame_ref->camera;
        const ExtrinsicParams &camera_tgt = frame->camera;
        const matrix<2> &sqrt_inv_cov = frame->sqrt_inv_cov;

        const vector<3> &z_ref = frame_ref->get_keypoint(keypoint_index_ref);
        vector<3> z_ref_center = camera_ref.q_cs * z_ref + camera_ref.p_cs;
        vector<3> z_tgt_center =
            q_tgt_center.conjugate() * q_ref_center * z_ref_center;
        vector<3> z_tgt =
            camera_tgt.q_cs.conjugate() * (z_tgt_center - camera_tgt.p_cs);
        vector<3> u_tgt = local_tangent.transpose() * z_tgt;
        r = u_tgt.hnormalized();

        if (jacobians) {
            if (jacobians[0]) {
                map<matrix<2, 4, true>> dr_dq_tgt(jacobians[0]);
                matrix<2, 3> dr_dz_tgt =
                    sqrt_inv_cov * dproj_dp(u_tgt) * local_tangent.transpose();
                matrix<2, 3> dr_dz_tgt_center =
                    dr_dz_tgt * camera_tgt.q_cs.conjugate().matrix();
                dr_dq_tgt.block<2, 3>(0, 0) =
                    dr_dz_tgt_center * hat(z_tgt_center);
                dr_dq_tgt.col(3).setZero();
            }
        }

        r = sqrt_inv_cov * r;

        return true;
    }

    Frame *frame;
    Track *track;
    const size_t keypoint_index;

  private:
    matrix<3> local_tangent;
};

} // namespace xrslam

#endif // XRSLAM_CERES_ROTATION_FACTOR_H
