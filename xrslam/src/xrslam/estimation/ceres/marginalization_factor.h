#ifndef XRSLAM_CERES_MARGINALIZATION_FACTOR_H
#define XRSLAM_CERES_MARGINALIZATION_FACTOR_H

#include <ceres/ceres.h>
#include <xrslam/common.h>
#include <xrslam/estimation/ceres/preintegration_factor.h>
#include <xrslam/estimation/ceres/reprojection_factor.h>
#include <xrslam/estimation/marginalization_factor.h>

namespace xrslam {

class CeresMarginalizationFactor : public MarginalizationFactor,
                                   public ceres::CostFunction {
  public:
    CeresMarginalizationFactor(Map *map) : MarginalizationFactor(map) {
        set_num_residuals((int)frames.size() * ES_SIZE);
        mutable_parameter_block_sizes()->clear();
        for (size_t i = 0; i < frames.size(); ++i) {
            mutable_parameter_block_sizes()->push_back(4); // q
            mutable_parameter_block_sizes()->push_back(3); // p
            mutable_parameter_block_sizes()->push_back(3); // v
            mutable_parameter_block_sizes()->push_back(3); // bg
            mutable_parameter_block_sizes()->push_back(3); // ba
        }
    }

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        for (size_t i = 0; i < frames.size(); ++i) {
            const_map<quaternion> q(parameters[5 * i + 0]);
            const_map<vector<3>> p(parameters[5 * i + 1]);
            const_map<vector<3>> v(parameters[5 * i + 2]);
            const_map<vector<3>> bg(parameters[5 * i + 3]);
            const_map<vector<3>> ba(parameters[5 * i + 4]);
            map<vector<3>> rq(&residuals[ES_SIZE * i + ES_Q]);
            map<vector<3>> rp(&residuals[ES_SIZE * i + ES_P]);
            map<vector<3>> rv(&residuals[ES_SIZE * i + ES_V]);
            map<vector<3>> rbg(&residuals[ES_SIZE * i + ES_BG]);
            map<vector<3>> rba(&residuals[ES_SIZE * i + ES_BA]);
            rq = logmap(pose_linearization_point[i].q.conjugate() * q);
            rp = p - pose_linearization_point[i].p;
            rv = v - motion_linearization_point[i].v;
            rbg = bg - motion_linearization_point[i].bg;
            rba = ba - motion_linearization_point[i].ba;
        }
        if (jacobians) {
            for (size_t i = 0; i < frames.size(); ++i) {
                if (jacobians[5 * i + 0]) {
                    map<matrix<Eigen::Dynamic, 4, true>> dr_dq(
                        jacobians[5 * i + 0], frames.size() * ES_SIZE, 4);
                    const_map<vector<3>> rq(&residuals[ES_SIZE * i + ES_Q]);
                    dr_dq.setZero();
                    dr_dq.block<3, 3>(ES_SIZE * i + ES_Q, 0) =
                        right_jacobian(rq).inverse();
                    dr_dq = sqrt_inv_cov * dr_dq;
                }
                for (size_t k = 1; k < 5; ++k) {
                    if (jacobians[5 * i + k]) {
                        map<matrix<Eigen::Dynamic, 3, true>> dr_dk(
                            jacobians[5 * i + k], frames.size() * ES_SIZE, 3);
                        dr_dk.setZero();
                        dr_dk.block<3, 3>(ES_SIZE * i + k * 3, 0).setIdentity();
                        dr_dk = sqrt_inv_cov * dr_dk;
                    }
                }
            }
        }
        map<vector<>> full_residual(residuals, frames.size() * ES_SIZE);
        full_residual = sqrt_inv_cov * full_residual + infovec;

        return true;
    }

    void marginalize(size_t index) override {
        struct LandmarkInfo {
            LandmarkInfo() {
                mat = 0;
                vec = 0;
            }
            double mat;
            double vec;
            std::unordered_map<size_t, matrix<1, 6>> h;
        };

        matrix<> pose_motion_infomat;
        vector<> pose_motion_infovec;
        std::map<Track *, LandmarkInfo, compare<Track *>> landmark_info;

        pose_motion_infomat.resize(base_map->frame_num() * ES_SIZE,
                                   base_map->frame_num() * ES_SIZE);
        pose_motion_infovec.resize(base_map->frame_num() * ES_SIZE);
        pose_motion_infomat.setZero();
        pose_motion_infovec.setZero();

        std::unordered_map<Frame *, size_t> frame_indices;
        for (size_t i = 0; i < base_map->frame_num(); ++i) {
            if (i < index) {
                frame_indices[base_map->get_frame(i)] = i;
            } else if (i > index) {
                frame_indices[base_map->get_frame(i)] = i - 1;
            } else {
                frame_indices[base_map->get_frame(i)] =
                    base_map->frame_num() - 1;
            }
        }

        /* scope: marginalization factor */
        {
            std::vector<const double *> marparameters(frames.size() * 5);
            std::vector<double *> marjacobian_ptrs(frames.size() * 5);
            std::vector<matrix<Eigen::Dynamic, Eigen::Dynamic, true>>
                marjacobians(frames.size() * 5);
            for (size_t i = 0; i < frames.size(); ++i) {
                marparameters[5 * i + 0] = frames[i]->pose.q.coeffs().data();
                marparameters[5 * i + 1] = frames[i]->pose.p.data();
                marparameters[5 * i + 2] = frames[i]->motion.v.data();
                marparameters[5 * i + 3] = frames[i]->motion.bg.data();
                marparameters[5 * i + 4] = frames[i]->motion.ba.data();
                marjacobians[5 * i + 0].resize(frames.size() * ES_SIZE, 4);
                marjacobians[5 * i + 1].resize(frames.size() * ES_SIZE, 3);
                marjacobians[5 * i + 2].resize(frames.size() * ES_SIZE, 3);
                marjacobians[5 * i + 3].resize(frames.size() * ES_SIZE, 3);
                marjacobians[5 * i + 4].resize(frames.size() * ES_SIZE, 3);
                marjacobian_ptrs[5 * i + 0] = marjacobians[5 * i + 0].data();
                marjacobian_ptrs[5 * i + 1] = marjacobians[5 * i + 1].data();
                marjacobian_ptrs[5 * i + 2] = marjacobians[5 * i + 2].data();
                marjacobian_ptrs[5 * i + 3] = marjacobians[5 * i + 3].data();
                marjacobian_ptrs[5 * i + 4] = marjacobians[5 * i + 4].data();
            }
            vector<> marresidual;
            marresidual.resize(frames.size() * ES_SIZE);
            Evaluate(marparameters.data(), marresidual.data(),
                     marjacobian_ptrs.data());

            std::vector<matrix<>> state_jacobians(frames.size());
            for (size_t i = 0; i < frames.size(); ++i) {
                matrix<> &dr_ds = state_jacobians[i];
                dr_ds.resize(frames.size() * ES_SIZE, ES_SIZE);
                dr_ds.block(0, ES_Q, frames.size() * ES_SIZE, 3) =
                    marjacobians[5 * i + 0].leftCols(3);
                dr_ds.block(0, ES_P, frames.size() * ES_SIZE, 3) =
                    marjacobians[5 * i + 1];
                dr_ds.block(0, ES_V, frames.size() * ES_SIZE, 3) =
                    marjacobians[5 * i + 2];
                dr_ds.block(0, ES_BG, frames.size() * ES_SIZE, 3) =
                    marjacobians[5 * i + 3];
                dr_ds.block(0, ES_BA, frames.size() * ES_SIZE, 3) =
                    marjacobians[5 * i + 4];
            }
            for (size_t i = 0; i < frames.size(); ++i) {
                size_t frame_index_i = frame_indices.at(frames[i]);
                for (size_t j = 0; j < frames.size(); ++j) {
                    size_t frame_index_j = frame_indices.at(frames[j]);
                    pose_motion_infomat.block<ES_SIZE, ES_SIZE>(
                        ES_SIZE * frame_index_i, ES_SIZE * frame_index_j) +=
                        state_jacobians[i].transpose() * state_jacobians[j];
                }
                pose_motion_infovec.segment<ES_SIZE>(ES_SIZE * frame_index_i) +=
                    state_jacobians[i].transpose() * marresidual;
            }
        }

        /* scope: preintegration factor */
        for (size_t j = index; j <= index + 1; ++j) {
            if (j == 0)
                continue;
            if (j >= base_map->frame_num())
                continue;
            size_t i = j - 1;
            Frame *frame_i = base_map->get_frame(i);
            Frame *frame_j = base_map->get_frame(j);

            // if (!frame_j->get_preintegration_factor()) continue;
            std::unique_ptr<CeresPreIntegrationErrorFactor> picost =
                std::make_unique<CeresPreIntegrationErrorFactor>(
                    frame_i, frame_j, frame_j->keyframe_preintegration);
            std::array<const double *, 10> piparameters = {
                frame_i->pose.q.coeffs().data(),
                frame_i->pose.p.data(),
                frame_i->motion.v.data(),
                frame_i->motion.bg.data(),
                frame_i->motion.ba.data(),
                frame_j->pose.q.coeffs().data(),
                frame_j->pose.p.data(),
                frame_j->motion.v.data(),
                frame_j->motion.bg.data(),
                frame_j->motion.ba.data()};
            vector<ES_SIZE> piresidual;
            matrix<ES_SIZE, 4, true> dr_dqi, dr_dqj;
            matrix<ES_SIZE, 3, true> dr_dpi, dr_dpj;
            matrix<ES_SIZE, 3, true> dr_dvi, dr_dvj;
            matrix<ES_SIZE, 3, true> dr_dbgi, dr_dbgj;
            matrix<ES_SIZE, 3, true> dr_dbai, dr_dbaj;
            std::array<double *, 10> pijacobians = {
                dr_dqi.data(),  dr_dpi.data(), dr_dvi.data(), dr_dbgi.data(),
                dr_dbai.data(), dr_dqj.data(), dr_dpj.data(), dr_dvj.data(),
                dr_dbgj.data(), dr_dbaj.data()};
            picost->Evaluate(piparameters.data(), piresidual.data(),
                             pijacobians.data());
            matrix<ES_SIZE, ES_SIZE> dr_dstates_i, dr_dstates_j;
            dr_dstates_i.block<ES_SIZE, 3>(0, ES_Q) =
                dr_dqi.block<ES_SIZE, 3>(0, 0);
            dr_dstates_i.block<ES_SIZE, 3>(0, ES_P) = dr_dpi;
            dr_dstates_i.block<ES_SIZE, 3>(0, ES_V) = dr_dvi;
            dr_dstates_i.block<ES_SIZE, 3>(0, ES_BG) = dr_dbgi;
            dr_dstates_i.block<ES_SIZE, 3>(0, ES_BA) = dr_dbai;
            dr_dstates_j.block<ES_SIZE, 3>(0, ES_Q) =
                dr_dqj.block<ES_SIZE, 3>(0, 0);
            dr_dstates_j.block<ES_SIZE, 3>(0, ES_P) = dr_dpj;
            dr_dstates_j.block<ES_SIZE, 3>(0, ES_V) = dr_dvj;
            dr_dstates_j.block<ES_SIZE, 3>(0, ES_BG) = dr_dbgj;
            dr_dstates_j.block<ES_SIZE, 3>(0, ES_BA) = dr_dbaj;
            size_t frame_i_index = frame_indices.at(frame_i);
            size_t frame_j_index = frame_indices.at(frame_j);
            pose_motion_infomat.block<ES_SIZE, ES_SIZE>(
                ES_SIZE * frame_i_index, ES_SIZE * frame_i_index) +=
                dr_dstates_i.transpose() * dr_dstates_i;
            pose_motion_infomat.block<ES_SIZE, ES_SIZE>(
                ES_SIZE * frame_i_index, ES_SIZE * frame_j_index) +=
                dr_dstates_i.transpose() * dr_dstates_j;
            pose_motion_infomat.block<ES_SIZE, ES_SIZE>(
                ES_SIZE * frame_j_index, ES_SIZE * frame_i_index) +=
                dr_dstates_j.transpose() * dr_dstates_i;
            pose_motion_infomat.block<ES_SIZE, ES_SIZE>(
                ES_SIZE * frame_j_index, ES_SIZE * frame_j_index) +=
                dr_dstates_j.transpose() * dr_dstates_j;
            pose_motion_infovec.segment<ES_SIZE>(ES_SIZE * frame_i_index) +=
                dr_dstates_i.transpose() * piresidual;
            pose_motion_infovec.segment<ES_SIZE>(ES_SIZE * frame_j_index) +=
                dr_dstates_j.transpose() * piresidual;
        }

        /* scope: reprojection error factor */
        Frame *frame_victim = base_map->get_frame(index);
        for (size_t j = 0; j < frame_victim->keypoint_num(); ++j) {
            Track *track = frame_victim->get_track(j);
            if (!track || !track->tag(TT_VALID))
                continue;
            Frame *frame_ref = track->first_frame();
            if (!frame_ref->tag(FT_KEYFRAME))
                continue;
            size_t frame_index_ref = frame_indices.at(frame_ref);
            for (const auto &[frame_tgt, keypoint_index] :
                 track->keypoint_map()) {
                if (frame_tgt == frame_ref)
                    continue;
                if (frame_indices.count(frame_tgt) == 0)
                    continue;
                size_t frame_index_tgt = frame_indices.at(frame_tgt);
                CeresReprojectionErrorFactor *rpcost =
                    static_cast<CeresReprojectionErrorFactor *>(
                        frame_tgt->reprojection_error_factors[keypoint_index]
                            .get());
                std::array<const double *, 5> rpparameters = {
                    frame_tgt->pose.q.coeffs().data(), frame_tgt->pose.p.data(),
                    frame_ref->pose.q.coeffs().data(), frame_ref->pose.p.data(),
                    &(track->landmark.inv_depth)};
                vector<2> rpresidual;
                matrix<2, 4, true> dr_dq_tgt;
                matrix<2, 3, true> dr_dp_tgt;
                matrix<2, 4, true> dr_dq_ref;
                matrix<2, 3, true> dr_dp_ref;
                vector<2> dr_dinv_depth;
                std::array<double *, 5> rpjacobians = {
                    dr_dq_tgt.data(), dr_dp_tgt.data(), dr_dq_ref.data(),
                    dr_dp_ref.data(), dr_dinv_depth.data()};
                rpcost->Evaluate(rpparameters.data(), rpresidual.data(),
                                 rpjacobians.data());
                matrix<2, 3, true> dr_dq_tgt_local =
                    dr_dq_tgt.block<2, 3>(0, 0);
                matrix<2, 3, true> dr_dq_ref_local =
                    dr_dq_ref.block<2, 3>(0, 0);

                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_Q,
                    ES_SIZE * frame_index_tgt + ES_Q) +=
                    dr_dq_tgt_local.transpose() * dr_dq_tgt_local;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_P,
                    ES_SIZE * frame_index_tgt + ES_P) +=
                    dr_dp_tgt.transpose() * dr_dp_tgt;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_Q,
                    ES_SIZE * frame_index_tgt + ES_P) +=
                    dr_dq_tgt_local.transpose() * dr_dp_tgt;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_P,
                    ES_SIZE * frame_index_tgt + ES_Q) +=
                    dr_dp_tgt.transpose() * dr_dq_tgt_local;

                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_Q,
                    ES_SIZE * frame_index_tgt + ES_Q) +=
                    dr_dq_ref_local.transpose() * dr_dq_tgt_local;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_P,
                    ES_SIZE * frame_index_tgt + ES_P) +=
                    dr_dp_ref.transpose() * dr_dp_tgt;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_Q,
                    ES_SIZE * frame_index_tgt + ES_P) +=
                    dr_dq_ref_local.transpose() * dr_dp_tgt;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_P,
                    ES_SIZE * frame_index_tgt + ES_Q) +=
                    dr_dp_ref.transpose() * dr_dq_tgt_local;

                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_Q,
                    ES_SIZE * frame_index_ref + ES_Q) +=
                    dr_dq_tgt_local.transpose() * dr_dq_ref_local;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_P,
                    ES_SIZE * frame_index_ref + ES_P) +=
                    dr_dp_tgt.transpose() * dr_dp_ref;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_Q,
                    ES_SIZE * frame_index_ref + ES_P) +=
                    dr_dq_tgt_local.transpose() * dr_dp_ref;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_tgt + ES_P,
                    ES_SIZE * frame_index_ref + ES_Q) +=
                    dr_dp_tgt.transpose() * dr_dq_ref_local;

                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_Q,
                    ES_SIZE * frame_index_ref + ES_Q) +=
                    dr_dq_ref_local.transpose() * dr_dq_ref_local;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_P,
                    ES_SIZE * frame_index_ref + ES_P) +=
                    dr_dp_ref.transpose() * dr_dp_ref;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_Q,
                    ES_SIZE * frame_index_ref + ES_P) +=
                    dr_dq_ref_local.transpose() * dr_dp_ref;
                pose_motion_infomat.block<3, 3>(
                    ES_SIZE * frame_index_ref + ES_P,
                    ES_SIZE * frame_index_ref + ES_Q) +=
                    dr_dp_ref.transpose() * dr_dq_ref_local;

                pose_motion_infovec.segment<3>(ES_SIZE * frame_index_tgt +
                                               ES_Q) +=
                    dr_dq_tgt_local.transpose() * rpresidual;
                pose_motion_infovec.segment<3>(ES_SIZE * frame_index_tgt +
                                               ES_P) +=
                    dr_dp_tgt.transpose() * rpresidual;
                pose_motion_infovec.segment<3>(ES_SIZE * frame_index_ref +
                                               ES_Q) +=
                    dr_dq_ref_local.transpose() * rpresidual;
                pose_motion_infovec.segment<3>(ES_SIZE * frame_index_ref +
                                               ES_P) +=
                    dr_dp_ref.transpose() * rpresidual;

                LandmarkInfo &linfo = landmark_info[track];

                linfo.mat += dr_dinv_depth.transpose() * dr_dinv_depth;
                linfo.vec += dr_dinv_depth.transpose() * rpresidual;
                if (linfo.h.count(frame_index_tgt) == 0) {
                    linfo.h[frame_index_tgt].setZero();
                }
                if (linfo.h.count(frame_index_ref) == 0) {
                    linfo.h[frame_index_ref].setZero();
                }
                /* scope */ {
                    matrix<1, 6> &h = linfo.h.at(frame_index_tgt);
                    h.segment<3>(ES_Q - ES_Q) +=
                        dr_dinv_depth.transpose() * dr_dq_tgt_local;
                    h.segment<3>(ES_P - ES_Q) +=
                        dr_dinv_depth.transpose() * dr_dp_tgt;
                }
                /* scope */ {
                    matrix<1, 6> &h = linfo.h.at(frame_index_ref);
                    h.segment<3>(ES_Q - ES_Q) +=
                        dr_dinv_depth.transpose() * dr_dq_ref_local;
                    h.segment<3>(ES_P - ES_Q) +=
                        dr_dinv_depth.transpose() * dr_dp_ref;
                }
            }
        }

        /* scope: marginalize landmarks */
        for (const auto &[track, info] : landmark_info) {
            double inv_infomat = 1.0 / info.mat;
            if (!std::isfinite(inv_infomat))
                continue;
            for (const auto &[frame_index_i, h_i] : info.h) {
                for (const auto &[frame_index_j, h_j] : info.h) {
                    pose_motion_infomat.block<6, 6>(
                        ES_SIZE * frame_index_i + ES_Q,
                        ES_SIZE * frame_index_j + ES_Q) -=
                        h_i.transpose() * inv_infomat * h_j;
                }
                pose_motion_infovec.segment<6>(ES_SIZE * frame_index_i +
                                               ES_Q) -=
                    h_i.transpose() * inv_infomat * info.vec;
            }
        }

        /* scope: marginalize the corresponding frame */ {
            size_t last_index = base_map->frame_num() - 1;
            matrix<15, 15> inv_infomat =
                pose_motion_infomat
                    .block<ES_SIZE, ES_SIZE>(ES_SIZE * last_index,
                                             ES_SIZE * last_index)
                    .inverse();
            matrix<> complement_infomat;
            vector<> complement_infovec;
            complement_infomat.resize(ES_SIZE * (base_map->frame_num() - 1),
                                      ES_SIZE * (base_map->frame_num() - 1));
            complement_infovec.resize(ES_SIZE * (base_map->frame_num() - 1));
            complement_infomat.setZero();
            complement_infovec.setZero();

            matrix<> complement_infomat_block = pose_motion_infomat.block(
                0, 0, ES_SIZE * last_index, ES_SIZE * last_index);
            vector<> complement_infovec_segment =
                pose_motion_infovec.segment(0, ES_SIZE * last_index);
            complement_infomat_block -=
                pose_motion_infomat.block(0, ES_SIZE * last_index,
                                          ES_SIZE * last_index, ES_SIZE) *
                inv_infomat *
                pose_motion_infomat.block(ES_SIZE * last_index, 0, ES_SIZE,
                                          ES_SIZE * last_index);
            complement_infovec_segment -=
                pose_motion_infomat.block(0, ES_SIZE * last_index,
                                          ES_SIZE * last_index, ES_SIZE) *
                inv_infomat *
                pose_motion_infovec.segment(ES_SIZE * last_index, ES_SIZE);
            complement_infomat.block(0, 0, ES_SIZE * last_index,
                                     ES_SIZE * last_index) =
                complement_infomat_block;
            complement_infovec.segment(0, ES_SIZE * last_index) =
                complement_infovec_segment;

            pose_motion_infomat = complement_infomat;
            pose_motion_infovec = complement_infovec;
        }

        /* scope: create marginalization factor */ {
            Eigen::SelfAdjointEigenSolver<matrix<>> saesolver(
                pose_motion_infomat);

            vector<> lambdas = (saesolver.eigenvalues().array() > 1.0e-8)
                                   .select(saesolver.eigenvalues(), 0);
            vector<> lambdas_inv =
                (saesolver.eigenvalues().array() > 1.0e-8)
                    .select(saesolver.eigenvalues().cwiseInverse(), 0);

            sqrt_inv_cov = lambdas.cwiseSqrt().asDiagonal() *
                           saesolver.eigenvectors().transpose();
            infovec = lambdas_inv.cwiseSqrt().asDiagonal() *
                      saesolver.eigenvectors().transpose() *
                      pose_motion_infovec;

            frames.resize(base_map->frame_num() - 1);
            pose_linearization_point.resize(base_map->frame_num() - 1);
            motion_linearization_point.resize(base_map->frame_num() - 1);
            set_num_residuals((int)frames.size() * ES_SIZE);
            mutable_parameter_block_sizes()->clear();
            for (size_t i = 0; i < base_map->frame_num(); ++i) {
                if (i == index)
                    continue;
                size_t j = i > index ? i - 1 : i;
                frames[j] = base_map->get_frame(i);
                pose_linearization_point[j] = frames[j]->pose;
                motion_linearization_point[j] = frames[j]->motion;
                mutable_parameter_block_sizes()->push_back(4); // q
                mutable_parameter_block_sizes()->push_back(3); // p
                mutable_parameter_block_sizes()->push_back(3); // v
                mutable_parameter_block_sizes()->push_back(3); // bg
                mutable_parameter_block_sizes()->push_back(3); // ba
            }
        }
    }
};

} // namespace xrslam

#endif // XRSLAM_CERES_MARGINALIZATION_FACTOR_H
