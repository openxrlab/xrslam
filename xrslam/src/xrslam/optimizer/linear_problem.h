#ifndef __OPTIMIZER_LINEAR_PROBLEM_H__
#define __OPTIMIZER_LINEAR_PROBLEM_H__

#include <xrslam/common.h>
#include <ceres/ceres.h>
#include <xrslam/optimizer/loss_functin.h>
#include <xrslam/optimizer/tiny_reprojection_factor.h>
#include <xrslam/optimizer/tiny_preintegration_factor.h>
#include <xrslam/optimizer/linear_base.h>
#include <xrslam/optimizer/linear_solving.h>

using namespace ceres::internal;

// #define CFG_OPTIMIZER_DEBUG 0

namespace xrslam{

class LinearProblem{

    double m_radius = 1.0e4;
    double m_min_radius = 1.0e-32;
    double m_max_radius = 1.0e+16;
    double m_decrease_factor = 2.0;

    double m_current_cost = 0.0;
    double m_last_cost = 0.0;
    double m_cost_change = 0.0;
    double m_last_wall_time = 0.0;
    double m_start_wall_time = 0.0;

    SparseBlockStorage real_lhs;
    vectorX real_rhs;
    vectorX real_dxc;

    vectorX x_f;
    vectorX x_e;

    matrixX J;
    matrixX r;
    matrixX JtJ;
    matrixX Jtr;

    size_t num_landmarks = 0;
    size_t num_residuals = 0;
    size_t num_camera_bundles = 0;
    size_t num_camera_bundles_const = 0;

    std::unique_ptr<SparseCholesky> sparse_cholesky_;

public:

    LinearProblem(){
        LinearSolver::Options sparse_cholesky_options;
        sparse_cholesky_options.use_postordering = true;
        sparse_cholesky_ = SparseCholesky::Create(sparse_cholesky_options);
    }

    ~LinearProblem() = default;

    ParameterBlockId AddParameterBlock(double* values, int size, ParamType type, ceres::LocalParameterization* local_parameterization=nullptr){
        //TODO: maybe need to check the parameterization type in deifferent add process
        auto it = pointer_to_param_id_map.find(values);
        if(it == pointer_to_param_id_map.end()){
            auto pb = std::make_unique<ParameterBlock>(values, size, type, local_parameterization);
            param_blocks.push_back(std::move(pb));
            pointer_to_param_id_map[values] = param_blocks.back().get();
        }
        return pointer_to_param_id_map[values];
    }

    ResidualBlockId AddResidualBlock(TinyReprojectionErrorFactor* cost_function, LossFunction* loss_function){
        
        auto tgt_frame = cost_function->frame;
        auto ref_frame = cost_function->track->first_frame();
        auto track = cost_function->track;

        ParameterBlockId tgt_frame_q_pb = AddParameterBlock(tgt_frame->pose.q.coeffs().data(), 4, PARAM_CAMERA); // need local parameterization
        ParameterBlockId tgt_frame_p_pb = AddParameterBlock(tgt_frame->pose.p.data(), 3, PARAM_CAMERA);
        ParameterBlockId ref_frame_q_pb = AddParameterBlock(ref_frame->pose.q.coeffs().data(), 4, PARAM_CAMERA);
        ParameterBlockId ref_frame_p_pb = AddParameterBlock(ref_frame->pose.p.data(), 3, PARAM_CAMERA);
        ParameterBlockId landmark_pb = AddParameterBlock(&track->landmark.inv_depth, 1, PARAM_LANDMARK);

        LandmarkId lm = AddLandmark(landmark_pb);

        CameraBundleId cb_tgt = AddCameraBundle(tgt_frame_p_pb, tgt_frame_q_pb);
        CameraBundleId cb_ref = AddCameraBundle(ref_frame_p_pb, ref_frame_q_pb);

        std::unique_ptr<VisualResidualBlock> rb = std::make_unique<VisualResidualBlock>(cost_function);
        rb->landmark = lm;
        rb->camera_bundle_tgt = cb_tgt;
        rb->camera_bundle_ref = cb_ref;

        rb->jacobian_ptr = {
            rb->jacobain_tgt_q.data(),
            rb->jacobain_tgt_p.data(),
            rb->jacobain_ref_q.data(),
            rb->jacobain_ref_p.data(),
            rb->jacobain_inv_depth.data()
        };
        rb->param_block_ptr = {
            rb->factor->frame->pose.q.coeffs().data(),
            rb->factor->frame->pose.p.data(),
            rb->factor->track->first_frame()->pose.q.coeffs().data(),
            rb->factor->track->first_frame()->pose.p.data(),
            &rb->factor->track->landmark.inv_depth
        };
        rb->param_block_candidate_ptr = {
            rb->camera_bundle_tgt->frame_q->param_new.data(),
            rb->camera_bundle_tgt->frame_p->param_new.data(),
            rb->camera_bundle_ref->frame_q->param_new.data(),
            rb->camera_bundle_ref->frame_p->param_new.data(),
            rb->landmark->landmark->param_new.data()
        };

        rb->loss_func = loss_function;

        visual_residual_blocks.push_back(std::move(rb));

        lm->camera_bundles.push_back(std::make_pair(cb_tgt, cb_ref));
        lm->residual_blocks.push_back(visual_residual_blocks.back().get());
        cb_tgt->landmarks.push_back(lm);
        cb_tgt->visual_residual_blocks.push_back(visual_residual_blocks.back().get());
        cb_ref->landmarks.push_back(lm);
        cb_ref->visual_residual_blocks.push_back(visual_residual_blocks.back().get());

        return visual_residual_blocks.back().get();
    }

    ResidualBlockId AddResidualBlock(TinyPreIntegrationErrorFactor* cost_function, LossFunction* loss_function){
        
        auto frame_i = cost_function->frame_i;
        auto frame_j = cost_function->frame_j;

        ParameterBlockId frame_i_q_pb = AddParameterBlock(frame_i->pose.q.coeffs().data(), 4, PARAM_CAMERA); // need local parameterization
        ParameterBlockId frame_i_p_pb = AddParameterBlock(frame_i->pose.p.data(), 3, PARAM_CAMERA);
        ParameterBlockId frame_i_v_pb = AddParameterBlock(frame_i->motion.v.data(), 3, PARAM_MOTION_VELOCITY);
        ParameterBlockId frame_i_bg_pb = AddParameterBlock(frame_i->motion.bg.data(), 3, PARAM_MOTION_BIAS_GYR);
        ParameterBlockId frame_i_ba_pb = AddParameterBlock(frame_i->motion.ba.data(), 3, PARAM_MOTION_BIAS_ACC);
        ParameterBlockId frame_j_q_pb = AddParameterBlock(frame_j->pose.q.coeffs().data(), 4, PARAM_CAMERA);
        ParameterBlockId frame_j_p_pb = AddParameterBlock(frame_j->pose.p.data(), 3, PARAM_CAMERA);
        ParameterBlockId frame_j_v_pb = AddParameterBlock(frame_j->motion.v.data(), 3, PARAM_MOTION_VELOCITY);
        ParameterBlockId frame_j_bg_pb = AddParameterBlock(frame_j->motion.bg.data(), 3, PARAM_MOTION_BIAS_GYR);
        ParameterBlockId frame_j_ba_pb = AddParameterBlock(frame_j->motion.ba.data(), 3, PARAM_MOTION_BIAS_ACC);

        CameraBundleId cb_tgt = AddCameraBundle(frame_i_p_pb, frame_i_q_pb, frame_i_v_pb, frame_i_bg_pb, frame_i_ba_pb);
        CameraBundleId cb_ref = AddCameraBundle(frame_j_p_pb, frame_j_q_pb, frame_j_v_pb, frame_j_bg_pb, frame_j_ba_pb);

        std::unique_ptr<MotionResidualBlock> rb = std::make_unique<MotionResidualBlock>(cost_function);
        rb->camera_bundle_tgt = cb_tgt;
        rb->camera_bundle_ref = cb_ref;

        rb->jacobian_ptr = {
            rb->jacobian_tgt_q.data(),
            rb->jacobian_tgt_p.data(),
            rb->jacobian_tgt_v.data(),
            rb->jacobian_tgt_bg.data(),
            rb->jacobian_tgt_ba.data(),
            rb->jacobian_ref_q.data(),
            rb->jacobian_ref_p.data(),
            rb->jacobian_ref_v.data(),
            rb->jacobian_ref_bg.data(),
            rb->jacobian_ref_ba.data()
        };
        rb->param_block_ptr = {
            rb->factor->frame_i->pose.q.coeffs().data(),
            rb->factor->frame_i->pose.p.data(),
            rb->factor->frame_i->motion.v.data(),
            rb->factor->frame_i->motion.bg.data(),
            rb->factor->frame_i->motion.ba.data(),
            rb->factor->frame_j->pose.q.coeffs().data(),
            rb->factor->frame_j->pose.p.data(),
            rb->factor->frame_j->motion.v.data(),
            rb->factor->frame_j->motion.bg.data(),
            rb->factor->frame_j->motion.ba.data()
        };
        rb->param_block_candidate_ptr = {
            rb->camera_bundle_tgt->frame_q->param_new.data(),
            rb->camera_bundle_tgt->frame_p->param_new.data(),
            rb->camera_bundle_tgt->velocity->param_new.data(),
            rb->camera_bundle_tgt->bias_gyr->param_new.data(),
            rb->camera_bundle_tgt->bias_acc->param_new.data(),
            rb->camera_bundle_ref->frame_q->param_new.data(),
            rb->camera_bundle_ref->frame_p->param_new.data(),
            rb->camera_bundle_ref->velocity->param_new.data(),
            rb->camera_bundle_ref->bias_gyr->param_new.data(),
            rb->camera_bundle_ref->bias_acc->param_new.data()
        };

        rb->loss_func = loss_function;

        motion_residual_blocks.push_back(std::move(rb));
        
        cb_tgt->motion_residual_blocks.push_back(motion_residual_blocks.back().get());
        cb_ref->motion_residual_blocks.push_back(motion_residual_blocks.back().get());

        return motion_residual_blocks.back().get();
    }


    CameraBundleId AddCameraBundle(ParameterBlockId p, ParameterBlockId q){

        bool assert1 = pointer_to_camera_bundle_id_map.find(p->param_ptr) == pointer_to_camera_bundle_id_map.end();
        bool assert2 = pointer_to_camera_bundle_id_map.find(q->param_ptr) == pointer_to_camera_bundle_id_map.end();

        if (assert1 && assert2)
        {
            std::unique_ptr<CameraBundle> cb = std::make_unique<CameraBundle>(p, q, p->type);
            camera_bundles.push_back(std::move(cb));

            pointer_to_camera_bundle_id_map[p->param_ptr] = camera_bundles.back().get();
            pointer_to_camera_bundle_id_map[q->param_ptr] = camera_bundles.back().get();
        }else if(assert1 || assert2){
            std::cout << ">>> error: one of the camera bundle has been added" << std::endl;
        }else{
            // std::cout << ">>> error: both of the camera bundle has been added" << std::endl;
        }

        return pointer_to_camera_bundle_id_map[p->param_ptr];
    }

    CameraBundleId AddCameraBundle(ParameterBlockId p, ParameterBlockId q, ParameterBlockId v, ParameterBlockId bg, ParameterBlockId ba){

        auto cb = AddCameraBundle(p, q);
        cb->velocity = v;
        cb->bias_gyr = bg;
        cb->bias_acc = ba;

        return cb;
    }

    LandmarkId AddLandmark(ParameterBlockId landmark_pb){
        auto it = pointer_to_landmark_id_map.find(landmark_pb->param_ptr);
        if(it == pointer_to_landmark_id_map.end()){
            std::unique_ptr<LandmarkBlock> lm = std::make_unique<LandmarkBlock>(landmark_pb, PARAM_LANDMARK);
            landmarks.push_back(std::move(lm));
            pointer_to_landmark_id_map[landmark_pb->param_ptr] = landmarks.back().get();
        }
        return pointer_to_landmark_id_map[landmark_pb->param_ptr];
    }

    void PreprocessData(){
        for(auto& pb: param_blocks){
            if(pb->type == PARAM_LANDMARK){
                num_landmarks++;
            }
        }

        for(auto& cb: camera_bundles){
            cb->type == PARAM_CAMERA ? num_camera_bundles++ : num_camera_bundles_const++;
        }

        num_residuals = visual_residual_blocks.size();

        std::cout << "  -- num_camera_bundles: " << num_camera_bundles << std::endl;
        std::cout << "  -- num_camera_bundles_const: " << num_camera_bundles_const << std::endl;
        std::cout << "  -- num_landmarks: " << num_landmarks << std::endl;
        std::cout << "  -- num_residuals: " << num_residuals << std::endl;
    }

    void SetParameterBlockConstant(const double* values){
        pointer_to_param_id_map[values]->type = PARAM_CAMERA_CONST;
    }

    void PrintLog(){
        std::cout << std::endl;
        std::cout << ">>> linear problem information" << std::endl;
        std::cout << "  -- parameter blocks: " << param_blocks.size() << std::endl;  
        std::cout << "  -- residual blocks: " << visual_residual_blocks.size() << std::endl;
    }

    void Linearization(){

        for(size_t i = 0; i < visual_residual_blocks.size(); ++i){
            auto& rb = visual_residual_blocks[i];
            rb->factor->Evaluate(rb->param_block_ptr.data(), rb->residual.data(), rb->jacobian_ptr.data());
        }

        for(size_t i = 0; i < motion_residual_blocks.size(); ++i){
            auto& rb = motion_residual_blocks[i];
            rb->factor->Evaluate(rb->param_block_ptr.data(), rb->residual.data(), rb->jacobian_ptr.data());
        }

        ComputeResiduals();
    }

    void SchurComplement(){

        real_lhs.resize(num_camera_bundles);
        real_rhs.resize(num_camera_bundles * 6);
        real_dxc.resize(num_camera_bundles * 6);
        x_f.resize(num_camera_bundles * 6);
        x_e.resize(num_landmarks);

        for(size_t i = 0; i < num_camera_bundles; ++i){
            for(size_t j = 0; j < num_camera_bundles; ++j){
                real_lhs[i][j] = matrix6::Zero();
            }

            real_rhs.segment<6>(i * 6) = vector6::Zero();
            real_dxc.segment<6>(i * 6) = vector6::Zero();
        }

        etfs[0].assign(num_residuals, matrix6x1::Zero());
        etfs[1].assign(num_residuals, matrix6x1::Zero());
        etes.assign(num_landmarks, matrix1::Zero());
        etbs.assign(num_landmarks, vector1::Zero());

        id_to_camera.resize(num_camera_bundles);
        id_to_landmark.resize(num_landmarks);
        
        for(auto& rb: visual_residual_blocks){
            rb->PreComputeSchurComplement();
        }

        for(auto& rb: motion_residual_blocks){
            rb->PreComputeSchurComplement();
        }

        size_t curr_camera_id = 0;
        for(auto& cb: camera_bundles){
            if(cb->type == PARAM_CAMERA){
                id_to_camera[curr_camera_id] = cb.get();
                cb->id = curr_camera_id;
                curr_camera_id++;
            }
        }

        size_t curr_landmark_id = 0;
        for(auto& lm: landmarks){
            id_to_landmark[curr_landmark_id] = lm.get();
            lm->id = curr_landmark_id;
            curr_landmark_id++;
        }

        size_t curr_residual_block = 0;
        for(auto& rb: visual_residual_blocks){
            rb->id = curr_residual_block;
            curr_residual_block++;
        }

#ifdef CFG_OPTIMIZER_DEBUG
        size_t dim = num_camera_bundles * 6 + num_landmarks;
        J.resize(num_residuals * 2, dim);
        J.setZero();
        r.resize(num_residuals * 2, 1);
        r.setZero();
        Jtr.resize(dim, 1);
        Jtr.setZero();

        for(size_t i = 0; i < num_residuals; ++i){
            auto& rb = visual_residual_blocks[i];
            if(rb->camera_bundle_tgt->type != PARAM_CAMERA_CONST){
                J.block<2, 6>(i * 2, rb->camera_bundle_tgt->id * 6) = rb->jacobain_camera[0];
            }
            if(rb->camera_bundle_ref->type != PARAM_CAMERA_CONST){
                J.block<2, 6>(i * 2, rb->camera_bundle_ref->id * 6) = rb->jacobain_camera[1];
            }
            J.block<2, 1>(i * 2, num_camera_bundles * 6 + rb->landmark->id) = rb->jacobain_inv_depth;
            r.block<2, 1>(i * 2, 0) = rb->residual;
        }

        JtJ = J.transpose() * J;
        Jtr = J.transpose() * r;

        std::cout << "J" << std::endl;
        std::cout << J << std::endl;
#endif

        // maybe here need to assert the vector 
        
        //etei, etb
        for(size_t i = 0; i < num_landmarks; ++i){
            const auto lm = id_to_landmark[i];
            matrix1 ete = matrix1::Identity() / m_radius;
            vector1 etb = vector1::Zero();
            for (const auto rb : lm->residual_blocks) {
                ete.noalias() += rb->ete;
                etb.noalias() += rb->etb;
            }
            
            etes[i] = ete.inverse();
            etbs[i] = etb;
        }

        // etf
        for(size_t i = 0; i < num_residuals; ++i){
            if(visual_residual_blocks[i]->camera_bundle_tgt->type != PARAM_CAMERA_CONST){
                etfs[0][i].noalias() = visual_residual_blocks[i]->etf[0];
            }
            if(visual_residual_blocks[i]->camera_bundle_ref->type != PARAM_CAMERA_CONST){
                etfs[1][i].noalias() = visual_residual_blocks[i]->etf[1];
            }
        }

        // etf * etei * (etf)t, etf * etei * r
        for(size_t i = 0; i < num_camera_bundles; ++i){
            const auto cam1 = id_to_camera[i];
            for(auto& rb1: cam1->visual_residual_blocks){
                size_t cam1_type = cam1 == rb1->camera_bundle_tgt ? 0 : 1;
                auto lm = rb1->landmark;
                for(auto rb2: lm->residual_blocks){
                    CameraBundleId cam2;
                    cam2 = rb2->camera_bundle_tgt;
                    if(cam2->type != PARAM_CAMERA_CONST){
                        real_lhs[cam1->id][cam2->id].noalias() -= etfs[cam1_type][rb1->id] * etes[lm->id] * etfs[0][rb2->id].transpose();
                    }
                    cam2 = rb2->camera_bundle_ref;
                    if(cam2->type != PARAM_CAMERA_CONST){
                        real_lhs[cam1->id][cam2->id].noalias() -= etfs[cam1_type][rb1->id] * etes[lm->id] * etfs[1][rb2->id].transpose();
                    }
                }
                real_rhs.segment<6>(i * 6).noalias() -= etfs[cam1_type][rb1->id] * etes[lm->id] * etbs[lm->id];
            }
        }

        // diag_radius + ftf
        for(size_t i = 0; i < num_camera_bundles; ++i){
            real_lhs[i][i].noalias() += matrix6::Identity() / m_radius;
            const auto cam1 = id_to_camera[i];
            for(const auto rb: cam1->visual_residual_blocks){
                if(rb->camera_bundle_tgt->type != PARAM_CAMERA_CONST){
                    if(cam1 == rb->camera_bundle_tgt)
                        real_lhs[cam1->id][cam1->id].noalias() += rb->ftf[0];
                    else
                        real_lhs[cam1->id][rb->camera_bundle_tgt->id].noalias() += rb->ftf[2];
                }
                if(rb->camera_bundle_ref->type != PARAM_CAMERA_CONST){
                    if(cam1 == rb->camera_bundle_ref)
                        real_lhs[cam1->id][cam1->id].noalias() += rb->ftf[3];
                    else
                        real_lhs[cam1->id][rb->camera_bundle_ref->id].noalias() += rb->ftf[1];
                }
            }
        }

        // ftb - etf * etei * r
        for(size_t i = 0; i < num_camera_bundles; ++i){
            const auto cam1 = id_to_camera[i];
            for(const auto rb: cam1->visual_residual_blocks){
                if(cam1->type == PARAM_CAMERA_CONST)
                    continue;
                if(cam1 == rb->camera_bundle_tgt)
                    real_rhs.segment<6>(i * 6).noalias() += rb->ftb[0];
                if(cam1 == rb->camera_bundle_ref)
                    real_rhs.segment<6>(i * 6).noalias() += rb->ftb[1];
            }
        }

#ifdef CFG_OPTIMIZER_DEBUG
        // std::cout << "ftf: " << std::endl;
        // for(size_t i = 0; i < num_camera_bundles; ++i){
        //     for(size_t j = 0; j < num_camera_bundles; ++j){
        //         std::cout << "camera pair: " << i << " " << j << std::endl; 
        //         std::cout << real_lhs[i][j] << std::endl;
        //     }
        // }

        // std::cout << "ete: " << std::endl;
        // for(size_t i = 0; i < etes.size(); ++i){
        //     std::cout << std::setprecision(10) << etes[i].inverse() << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "etf 0: " << std::endl;
        // for(size_t i = 0; i < etfs[0].size(); ++i){
        //     std::cout << etfs[0][i].transpose() << std::endl;
        // }
        // std::cout << std::endl;
        // std::cout << "etf 1: " << std::endl;
        // for(size_t i = 0; i < etfs[1].size(); ++i){
        //     std::cout << etfs[1][i].transpose() << std::endl;
        // }
        // std::cout << std::endl;

        // std::cout << "real rhs: " << std::endl;
        // std::cout << real_rhs.transpose() << std::endl;


        // std::cout << "etbs: " << std::endl;
        // for(size_t i = 0; i < etbs.size(); ++i){
        //     std::cout << etbs[i].transpose() << " ";
        // }
        // std::cout << std::endl;

        // std::cout << "ftb" << std::endl;
        // std::cout << real_rhs.transpose() << std::endl;

        // for(size_t i = 0; i < num_camera_bundles; ++i){
        //     for(size_t j = 0; j < num_camera_bundles; ++j){
        //         std::cout << "lhs: " << i << " " << j << std::endl;
        //         std::cout << real_lhs[i][j] << std::endl;
        //     }
        // }

        // std::cout << "rhs: " << std::endl;
        // for(size_t i = 0; i < num_camera_bundles; ++i){
        //     std::cout << real_rhs.segment<6>(i * 6).transpose() << std::endl;
        // }
#endif

    }

    void ComputeDelta(){

        SolveLinearSystemSparse(real_lhs, real_rhs, real_dxc, sparse_cholesky_.get());
        // SolveLinearSystemDense(real_lhs, real_rhs, real_dxc);
        // SolveLinearSystemPCG(real_lhs, real_rhs.data(), real_dxc.data());

        x_f.setZero();
        for (int i = 0; i < num_camera_bundles; ++i) {
            x_f.segment<6>(i * 6) = real_dxc.segment<6>(i * 6);
        }

        x_e.setZero();
        for (int i = 0; i < num_landmarks; ++i) {
            auto lm = id_to_landmark[i];
            vector1 tmp = vector1::Zero();
            for(auto& rb: lm->residual_blocks){
                if(rb->camera_bundle_tgt->type != PARAM_CAMERA_CONST)
                    tmp.noalias() += rb->etf[0].transpose() * x_f.segment<6>(rb->camera_bundle_tgt->id * 6);
                if(rb->camera_bundle_ref->type != PARAM_CAMERA_CONST)
                    tmp.noalias() += rb->etf[1].transpose() * x_f.segment<6>(rb->camera_bundle_ref->id * 6);
            }

            x_e.segment<1>(1 * i) = etes[i] * (etbs[i] - tmp);
        }

        x_f = -x_f;
        x_e = -x_e;

        // std::cout << "      --x_f: " << x_f.transpose() << std::endl;
        // std::cout << "      --x_e: " << x_e.transpose() << std::endl;
    }

    double ComputeModelCostChange(){

        double delta = 0.0;
        for(size_t i = 0; i < num_residuals; ++i){
            auto& rb = visual_residual_blocks[i];
            vector<2> model_residual = vector<2>::Zero();

            if(rb->camera_bundle_tgt->type != PARAM_CAMERA_CONST){
                model_residual += rb->jacobain_camera[0] * x_f.segment<6>(rb->camera_bundle_tgt->id * 6);
            }
            if(rb->camera_bundle_ref->type != PARAM_CAMERA_CONST){
                model_residual += rb->jacobain_camera[1] * x_f.segment<6>(rb->camera_bundle_ref->id * 6);
            }

            model_residual += rb->jacobain_inv_depth * x_e.segment<1>(rb->landmark->id * 1);

            vector<2> tmp = model_residual / 2.0 + rb->residual;

            delta -= model_residual.dot(tmp);
        }
        return delta;
    }

    void ParamPlusDelta(){
        for(size_t i = 0; i < num_camera_bundles; ++i){
            auto& cb = id_to_camera[i];
            double* delta = x_f.segment<6>(i * 6).data();
            cb->frame_p->PlusDelta(delta);
            cb->frame_q->PlusDelta(delta + 3);
        }

        for(size_t i = 0; i < num_landmarks; ++i){
            auto& lm = id_to_landmark[i];
            lm->landmark->PlusDelta(x_e.segment<1>(i).data());
        }
    }

    double ComputeRelativeDecrease(double model_cost_change){
        
        ParamPlusDelta();

        double candidate_cost = ComputeCandidateResiduals();

        double relative_decrease = (m_current_cost - candidate_cost) / model_cost_change;

        return relative_decrease;
    }

    void ComputeStepNormXNorm(double step_norm, double x_norm){

        for(size_t i = 0; i < num_camera_bundles; ++i){
            step_norm += id_to_camera[i]->frame_p->StepSquareNorm();
            step_norm += id_to_camera[i]->frame_q->StepSquareNorm();
            x_norm += id_to_camera[i]->frame_p->XSquareNorm();
            x_norm += id_to_camera[i]->frame_q->XSquareNorm();
        }
        step_norm = std::sqrt(step_norm);
        x_norm = std::sqrt(x_norm);
    }

    void UpdateParameterBlock(){

        for(size_t i = 0; i < num_camera_bundles; ++i){
            auto& cb = id_to_camera[i];
            cb->frame_p->Update();
            cb->frame_q->Update();
        }

        for(size_t i = 0; i < num_landmarks; ++i){
            auto& lm = id_to_landmark[i];
            lm->landmark->Update();
        }
    }

    double ComputeResiduals(){
        m_current_cost = 0.0;
        for(size_t i = 0; i < visual_residual_blocks.size(); ++i){
            m_current_cost += visual_residual_blocks[i]->ComputeCost();
        }

        for(size_t i = 0; i < motion_residual_blocks.size(); ++i){
            m_current_cost += motion_residual_blocks[i]->ComputeCost();
        }

        m_current_cost /= 2.0;

        return m_current_cost;
    }

    double ComputeCandidateResiduals(){
        double cost = 0.0;
        for(size_t i = 0; i < visual_residual_blocks.size(); ++i){
            vector<2> res;
            visual_residual_blocks[i]->factor->Evaluate(visual_residual_blocks[i]->param_block_candidate_ptr.data(), res.data(), nullptr);
            cost += visual_residual_blocks[i]->ComputeCost(res);
        }

        for(size_t i = 0; i < motion_residual_blocks.size(); ++i){
            vector<15> res;
            motion_residual_blocks[i]->factor->Evaluate(motion_residual_blocks[i]->param_block_candidate_ptr.data(), res.data(), nullptr);
            cost += motion_residual_blocks[i]->ComputeCost(res);
        }

        cost /= 2.0;
        return cost;
    }

    void StepRejected(double step_quality){
        m_radius = m_radius / m_decrease_factor;
        m_radius = std::max(m_min_radius, std::min(m_max_radius, m_radius));
        m_decrease_factor *= 2.0;
    }

    void StepAccepted(double step_quality){
        m_radius = m_radius / std::max(1.0 / 3.0, 1.0 - std::pow(2.0 * step_quality - 1.0, 3));
        m_radius = std::max(m_min_radius, std::min(m_max_radius, m_radius));
        m_decrease_factor = 2.0;
    }

    void SolveLinearSystemDense(const SparseBlockStorage &lhs, vectorX &rhs, vectorX &dx)
    {
        const int num_block = lhs.size();
        matrixX dense_lhs = matrixX::Zero(num_block * 6, num_block * 6);
        for (int id_row_block = 0; id_row_block < num_block; ++id_row_block){
            for (auto &[id_col_block, block_matrix] : lhs[id_row_block]){
                dense_lhs.block<6, 6>(id_row_block * 6, id_col_block * 6) = block_matrix;
            }
        }

        Eigen::LLT<matrixX, Eigen::Upper> llt = dense_lhs.selfadjointView<Eigen::Upper>().llt();
        dx = llt.solve(rhs);
    }

    void SolveLinearSystemSparse(const SparseBlockStorage &lhs, vectorX &rhs, vectorX &dx, SparseCholesky *sparse_cholesky_){
        const int num_block = lhs.size();
        std::unique_ptr<CompressedRowSparseMatrix> _lhs;
        _lhs.reset(ConvertToCRSM(num_block, lhs));
        _lhs->set_storage_type(CompressedRowSparseMatrix::UPPER_TRIANGULAR);
        std::string message;
        sparse_cholesky_->FactorAndSolve(_lhs.get(), rhs.data(), dx.data(), &message); // 1000 4s
    }

    void SolveLinearSystemPCG(SparseBlockStorage &lhs, double *rhs, double *dx){
        using MatrixRef = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>;
        using VectorRef = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1>>;

        std::unique_ptr<BlockRandomAccessSparseMatrix> sc;
        if (!sc)
        {
            const int num_blocks = lhs.size();
            std::vector<int> blocks_(num_blocks, 6);
            std::set<std::pair<int, int>> block_pairs;
            for (int id_row_block = 0; id_row_block < num_blocks; ++id_row_block)
            {
                for (auto &[id_col_block, block_matrix] : lhs[id_row_block])
                {
                    block_pairs.insert(std::make_pair(id_row_block, id_col_block));
                }
            }
            sc.reset(new BlockRandomAccessSparseMatrix(blocks_, block_pairs));
        }
        ConvertToBRSM(lhs, sc.get());

        const int num_blocks = lhs.size();
        const int num_rows = 6 * num_blocks;
        std::vector<int> blocks_(num_blocks, 6);
        // Size of the blocks in the Schur complement.
        std::unique_ptr<BlockRandomAccessDiagonalMatrix> preconditioner_;
        preconditioner_.reset(new BlockRandomAccessDiagonalMatrix(blocks_));

        // Extract block diagonal from the Schur complement to construct the
        // schur_jacobi preconditioner.
        for (int i = 0; i < blocks_.size(); ++i)
        {
            const int block_size = blocks_[i];
            int sc_r, sc_c, sc_row_stride, sc_col_stride;
            CellInfo *sc_cell_info = sc->GetCell(i, i, &sc_r, &sc_c, &sc_row_stride, &sc_col_stride);
            CHECK(sc_cell_info != nullptr);
            MatrixRef sc_m(sc_cell_info->values, sc_row_stride, sc_col_stride);
            int pre_r, pre_c, pre_row_stride, pre_col_stride;
            CellInfo *pre_cell_info = preconditioner_->GetCell(i, i, &pre_r, &pre_c, &pre_row_stride, &pre_col_stride);
            CHECK(pre_cell_info != nullptr);
            MatrixRef pre_m(pre_cell_info->values, pre_row_stride, pre_col_stride);
            pre_m.block(pre_r, pre_c, block_size, block_size) = sc_m.block(sc_r, sc_c, block_size, block_size);
        }
        preconditioner_->Invert();

        VectorRef(dx, num_rows).setZero();

        std::unique_ptr<LinearOperator> lhs_adapter(new BlockRandomAccessSparseMatrixAdapter(*sc));
        std::unique_ptr<LinearOperator> preconditioner_adapter(new BlockRandomAccessDiagonalMatrixAdapter(*preconditioner_));

        LinearSolver::Options cg_options;
        cg_options.min_num_iterations = 0;
        cg_options.max_num_iterations = 100;
        ConjugateGradientsSolver cg_solver(cg_options);

        LinearSolver::PerSolveOptions cg_per_solve_options;
        cg_per_solve_options.r_tolerance = -1;
        cg_per_solve_options.q_tolerance = 0.1;
        cg_per_solve_options.preconditioner = preconditioner_adapter.get();

        cg_solver.Solve(lhs_adapter.get(), rhs, cg_per_solve_options, dx);
    }

    void StepInfo(size_t iter, double current_time){
        
        m_cost_change = m_last_cost - m_current_cost;
        m_last_cost = m_current_cost;
        
        static bool output_table_header = false;

        if(!output_table_header){
            std::cout << std::setw(4) << "iter"
                    << std::setw(10) << "cost"
                    << std::setw(17) << "cost_change"
                    << std::setw(12) << "tr_radius"
                    << std::setw(12) << "iter_time"
                    << std::setw(12) << "total_time"
                    << std::endl;

            std::cout << std::setw(4) << iter
                    << std::setw(14) << std::scientific << std::setprecision(6) << std::scientific << m_current_cost
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << 0.0
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << m_radius
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << 0.0
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << 0.0
                    << std::endl;

            output_table_header = true;
            m_start_wall_time = current_time;
            m_last_wall_time = current_time;
        }else{
            std::cout << std::setw(4) << iter
                    << std::setw(14) << std::scientific << std::setprecision(6) << std::scientific << m_current_cost
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << m_cost_change
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << m_radius
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << current_time - m_last_wall_time
                    << std::setw(12) << std::scientific << std::setprecision(2) << std::scientific << current_time - m_start_wall_time
                    << std::endl;
            
            m_last_wall_time = current_time;
        }
    }

    std::vector<std::unique_ptr<ParameterBlock>> param_blocks;
    std::vector<std::unique_ptr<VisualResidualBlock>> visual_residual_blocks;
    std::vector<std::unique_ptr<MotionResidualBlock>> motion_residual_blocks;
    std::vector<std::unique_ptr<CameraBundle>> camera_bundles;
    std::vector<std::unique_ptr<LandmarkBlock>> landmarks;

    std::unordered_map<const double*, ParameterBlockId> pointer_to_param_id_map;
    std::unordered_map<const double*, CameraBundleId> pointer_to_camera_bundle_id_map;
    std::unordered_map<const double*, LandmarkId> pointer_to_landmark_id_map;

    std::vector<CameraBundleId> id_to_camera;
    std::vector<LandmarkId> id_to_landmark;

    std::array<std::vector<matrix6x1>, 2> etfs;
    std::vector<matrix1> etes;
    std::vector<vector1> etbs;
};

}

#endif