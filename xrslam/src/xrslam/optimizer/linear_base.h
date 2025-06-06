#ifndef __OPTIMIZER_LINEAR_BASE_H__
#define __OPTIMIZER_LINEAR_BASE_H__

#include <ceres/ceres.h>
#include <xrslam/common.h>
#include <xrslam/optimizer/tiny_reprojection_factor.h>
#include <xrslam/optimizer/tiny_preintegration_factor.h>
#include <xrslam/optimizer/loss_functin.h>

namespace xrslam{

enum ParamType{
    PARAM_CAMERA,
    PARAM_CAMERA_CONST,
    PARAM_LANDMARK,
    PARAM_LANDMARK_CONST,   
    PARAM_MOTION_BIAS_GYR,
    PARAM_MOTION_BIAS_ACC,
    PARAM_MOTION_VELOCITY
};

class ResidualBlockBase;
class VisualResidualBlock;
class MotionResidualBlock;
class ParameterBlock;
class CameraBundle;
class LandmarkBlock;

using ResidualBlockId = ResidualBlockBase*;
using VisualResidualBlockId = VisualResidualBlock*;
using MotionResidualBlockId = MotionResidualBlock*;
using ParameterBlockId = ParameterBlock*;
using CameraBundleId = CameraBundle*;
using LandmarkId = LandmarkBlock*;

using SparseBlockStorage = std::vector<std::map<int, matrix6>>;

struct ResidualBlockIndex{
    ResidualBlockId residual_block;
    size_t index;
};

class ParameterBlock{

public:
    size_t id = 0;
    size_t size;
    double *param_ptr;
    std::vector<double> param_new;
    ParamType type;
    ceres::LocalParameterization *parameterization;

    ParameterBlock(double *_ptr, size_t _size, ParamType _type, ceres::LocalParameterization *_p):
    param_ptr(_ptr), size(_size), type(_type), parameterization(_p){
        param_new.resize(size);
        for(size_t i = 0; i < size; i++)
            param_new[i] = param_ptr[i];
    }   

    void Update() {
        for (size_t i = 0; i < size; ++i) 
            param_ptr[i] = param_new[i];
    }

    void PlusDelta(const double *delta){
        double* param_ptr_new = param_new.data();
        if(parameterization)
            parameterization->Plus(param_ptr, delta, param_ptr_new);
        else
            for(size_t i = 0; i < size; i++)
                param_ptr_new[i] = param_ptr[i] + delta[i];
    }

    double StepSquareNorm(){
        double sn = 0;
        for(size_t i = 0; i < size; i++)
            sn += (param_new[i] - param_ptr[i]) * (param_new[i] - param_ptr[i]);
        return sn;
    }

    double XSquareNorm(){
        double sn = 0;
        for(size_t i = 0; i < size; i++)
            sn += param_ptr[i] * param_ptr[i];
        return sn;
    }
};

class ResidualBlockBase{
public:
    size_t id = 0;
    LossFunction* loss_func = nullptr;
    virtual ~ResidualBlockBase() = default;
    virtual double ComputeCost() = 0;
    virtual void PreComputeSchurComplement() = 0;
};;

class MotionResidualBlock: public ResidualBlockBase{

public:
    TinyPreIntegrationErrorFactor* factor;

    CameraBundleId camera_bundle_tgt;
    CameraBundleId camera_bundle_ref;

    std::array<double*, 10> jacobian_ptr;
    std::array<double*, 10> param_block_ptr;
    std::array<double*, 10> param_block_candidate_ptr;

    matrix<15, 4, true> jacobian_tgt_q;
    matrix<15, 3, true> jacobian_tgt_p;
    matrix<15, 3, true> jacobian_tgt_v;
    matrix<15, 3, true> jacobian_tgt_bg;
    matrix<15, 3, true> jacobian_tgt_ba;
    matrix<15, 4, true> jacobian_ref_q;
    matrix<15, 3, true> jacobian_ref_p;
    matrix<15, 3, true> jacobian_ref_v;
    matrix<15, 3, true> jacobian_ref_bg;
    matrix<15, 3, true> jacobian_ref_ba;

    std::array<matrix<15, 6>, 2> jacobian_camera;
    std::array<matrix<15, 9>, 2> jacobian_motion;

    matrix<15, 1> residual;

    MotionResidualBlock(TinyPreIntegrationErrorFactor* factor): factor(factor){}


    double ComputeCost() override {
        return loss_func->Compute(residual.squaredNorm());
    }
    
    double ComputeCost(vector<15> res) {
        return loss_func->Compute(res.squaredNorm());
    }

    void PreComputeSchurComplement() override {
        
        jacobian_camera[0] << jacobian_tgt_p, jacobian_tgt_q.leftCols(3);
        jacobian_camera[1] << jacobian_ref_p, jacobian_ref_q.leftCols(3);

        jacobian_motion[0] << jacobian_tgt_v, jacobian_tgt_bg, jacobian_tgt_ba;
        jacobian_motion[1] << jacobian_ref_v, jacobian_ref_bg, jacobian_ref_ba;
    }

};

class VisualResidualBlock: public ResidualBlockBase{

public:

    TinyReprojectionErrorFactor* factor;

    // important to use RowMajor
    matrix<2, 4, true> jacobain_ref_q;
    matrix<2, 3, true> jacobain_ref_p;
    matrix<2, 4, true> jacobain_tgt_q;
    matrix<2, 3, true> jacobain_tgt_p;
    matrix<2, 1, true> jacobain_inv_depth;

    matrix<2, 1> residual;

    // J = [F, E]
    matrix1 ete;
    vector1 etb;
    std::array<matrix6x1, 2> etf;
    std::array<matrix6, 4> ftf;
    std::array<vector6, 2> ftb;
    std::array<matrix<2, 6>, 2> jacobain_camera;

    LandmarkId landmark;
    CameraBundleId camera_bundle_tgt;
    CameraBundleId camera_bundle_ref;

    std::array<double*, 5> jacobian_ptr;
    std::array<double*, 5> param_block_ptr;
    std::array<double*, 5> param_block_candidate_ptr;

    VisualResidualBlock(TinyReprojectionErrorFactor* factor): factor(factor){}

    double ComputeCost() override {
        return loss_func->Compute(residual.squaredNorm());
    }
    
    double ComputeCost(vector<2> res) {
        return loss_func->Compute(res.squaredNorm());
    }

    void PreComputeSchurComplement() override {
            
        if(0){
            // error
            vector1 res = vector1(loss_func->Compute(residual.squaredNorm()));

            matrix<1, 2> jac_kernel = loss_func->ComputeDerivative(residual.squaredNorm()) * matrix<1, 2>::Ones();

            ete.noalias() = (jac_kernel * jacobain_inv_depth).transpose() * (jac_kernel * jacobain_inv_depth);
            etb.noalias() = (jac_kernel * jacobain_inv_depth).transpose() * res;
        
            jacobain_camera[0] << jacobain_tgt_p, jacobain_tgt_q.leftCols(3);
            jacobain_camera[1] << jacobain_ref_p, jacobain_ref_q.leftCols(3);

            etf[0].noalias() = (jac_kernel * jacobain_camera[0]).transpose() * (jac_kernel * jacobain_inv_depth);
            etf[1].noalias() = (jac_kernel * jacobain_camera[1]).transpose() * (jac_kernel * jacobain_inv_depth);

            ftf[0].noalias() = (jac_kernel * jacobain_camera[0]).transpose() * (jac_kernel * jacobain_camera[0]);
            ftf[1].noalias() = (jac_kernel * jacobain_camera[0]).transpose() * (jac_kernel * jacobain_camera[1]);
            ftf[2].noalias() = (jac_kernel * jacobain_camera[1]).transpose() * (jac_kernel * jacobain_camera[0]);
            ftf[3].noalias() = (jac_kernel * jacobain_camera[1]).transpose() * (jac_kernel * jacobain_camera[1]);

            ftb[0].noalias() = (jac_kernel * jacobain_camera[0]).transpose() * res;
            ftb[1].noalias() = (jac_kernel * jacobain_camera[1]).transpose() * res;

        }else{
            ete.noalias() = jacobain_inv_depth.transpose() * jacobain_inv_depth;
            etb.noalias() = jacobain_inv_depth.transpose() * residual;
        
            jacobain_camera[0] << jacobain_tgt_p, jacobain_tgt_q.leftCols(3);
            jacobain_camera[1] << jacobain_ref_p, jacobain_ref_q.leftCols(3);

            etf[0].noalias() = jacobain_camera[0].transpose() * jacobain_inv_depth;
            etf[1].noalias() = jacobain_camera[1].transpose() * jacobain_inv_depth;

            ftf[0].noalias() = jacobain_camera[0].transpose() * jacobain_camera[0];
            ftf[1].noalias() = jacobain_camera[0].transpose() * jacobain_camera[1];
            ftf[2].noalias() = jacobain_camera[1].transpose() * jacobain_camera[0];
            ftf[3].noalias() = jacobain_camera[1].transpose() * jacobain_camera[1];

            ftb[0].noalias() = jacobain_camera[0].transpose() * residual;
            ftb[1].noalias() = jacobain_camera[1].transpose() * residual;
        }
    }
};



struct CameraBundle{
    size_t id = 0;
    ParamType type;
    ParameterBlockId frame_p;
    ParameterBlockId frame_q;
    ParameterBlockId velocity;
    ParameterBlockId bias_gyr;
    ParameterBlockId bias_acc;

    CameraBundle(ParameterBlockId _frame_p, ParameterBlockId _frame_q, ParamType type): 
    frame_p(_frame_p), frame_q(_frame_q), type(type){}

    std::list<LandmarkId> landmarks;
    std::list<VisualResidualBlockId> visual_residual_blocks;
    std::list<MotionResidualBlockId> motion_residual_blocks;
};


struct LandmarkBlock{
    size_t id = 0;
    ParamType type = PARAM_LANDMARK;
    ParameterBlockId landmark;

    LandmarkBlock(ParameterBlockId _landmark, ParamType type):landmark(_landmark), type(type){}

    std::list<std::pair<CameraBundleId, CameraBundleId>> camera_bundles;
    std::list<VisualResidualBlockId> residual_blocks;
};


}

#endif