#ifndef __OPTIMIZER_SOLVER_H__
#define __OPTIMIZER_SOLVER_H__

#include <xrslam/common.h>
#include <xrslam/estimation/ceres/quaternion_parameterization.h>
#include <xrslam/optimizer/tiny_reprojection_factor.h>
#include <xrslam/optimizer/tiny_preintegration_factor.h>
#include <xrslam/optimizer/linear_problem.h>
#include <xrslam/optimizer/loss_functin.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/track.h>
#include <xrslam/map/map.h>

namespace xrslam{

class TinySolver{

    bool m_verbose = false;

    Timer m_timer; 

    struct SolverDetails{
        static Config *&config() {
            static Config *s_config = nullptr;
            return s_config;
        }

        std::unique_ptr<LinearProblem> problem;
        std::unique_ptr<QuaternionParameterization> quaternion_parameterization;
        std::unique_ptr<LossFunction> cauchy_loss;

        std::vector<std::unique_ptr<TinyReprojectionErrorFactor>> managed_rpefactors;
        std::vector<std::unique_ptr<TinyPreIntegrationErrorFactor>> managed_piefactors;

    };

    TinySolver(): details(std::make_unique<SolverDetails>()){
        details->problem = std::make_unique<LinearProblem>();
        details->quaternion_parameterization = std::make_unique<QuaternionParameterization>();
        // details->cauchy_loss = std::make_unique<CauchyLoss>(1.0);
        details->cauchy_loss = std::make_unique<TrivialLoss>();

        m_timer = Timer();
    }

public:

    ~TinySolver() = default;

    static void init(Config *config){
        SolverDetails::config() = config;
    }

    static std::unique_ptr<TinySolver> create(){
        return std::unique_ptr<TinySolver>(new TinySolver());
    }

    bool solve(bool verbose = false){

        m_verbose = verbose;
        
        int iter_num = 0;
        int step_invalid_number = 0;
        int max_iter = 10;

        // details->problem->PrintLog();

        details->problem->PreprocessData();

        // this can be optimized
        details->problem->Linearization();
        
        while(iter_num++ < max_iter){

            if(step_invalid_number >= 5) 
                break;

            if(m_verbose)
                details->problem->StepInfo(iter_num, m_timer.get_time());

            details->problem->Linearization();

            details->problem->SchurComplement();

            details->problem->ComputeDelta();

            double delta = details->problem->ComputeModelCostChange();

            if(delta < 0.0){
                step_invalid_number ++;
                details->problem->StepRejected(0);
                continue;
            }

            double rel_dec = details->problem->ComputeRelativeDecrease(delta);

            if(rel_dec < 1e-3){
                step_invalid_number ++;
                details->problem->StepRejected(0);
                continue;
            }

            // double step_norm = 0.0, x_norm = 0.0;
            // details->problem->ComputeStepNormXNorm(step_norm, x_norm);
            
            details->problem->UpdateParameterBlock();

            details->problem->StepAccepted(rel_dec);

        }

        return false;
    }

    void add_frame_states(Frame *frame, bool with_motion = true){
        details->problem->AddParameterBlock(frame->pose.q.coeffs().data(), 4, PARAM_CAMERA, details->quaternion_parameterization.get());
        details->problem->AddParameterBlock(frame->pose.p.data(), 3, PARAM_CAMERA);
        if (frame->tag(FT_FIX_POSE)) {
            details->problem->SetParameterBlockConstant(frame->pose.q.coeffs().data());
            details->problem->SetParameterBlockConstant(frame->pose.p.data());
        }
        // if (with_motion) {
        //     details->problem->AddParameterBlock(frame->motion.v.data(), 3);
        //     details->problem->AddParameterBlock(frame->motion.bg.data(), 3);
        //     details->problem->AddParameterBlock(frame->motion.ba.data(), 3);
        //     if (frame->tag(FT_FIX_MOTION)) {
        //         details->problem->SetParameterBlockConstant(frame->motion.v.data());
        //         details->problem->SetParameterBlockConstant(frame->motion.bg.data());
        //         details->problem->SetParameterBlockConstant(frame->motion.ba.data());
        //     }
        // }
    }

    static std::unique_ptr<TinyReprojectionErrorFactor> create_reprojection_error_factor(Frame *frame, Track *track){
        return std::make_unique<TinyReprojectionErrorFactor>(frame, track);
    }

    static std::unique_ptr<TinyPreIntegrationErrorFactor> create_preintegration_error_factor(Frame *frame_i, Frame *frame_j, const PreIntegrator &preintegration){
        return std::make_unique<TinyPreIntegrationErrorFactor>(frame_i, frame_j, preintegration);
    }

    void add_factor(TinyReprojectionErrorFactor *rpefactor){
        details->problem->AddResidualBlock(rpefactor, details->cauchy_loss.get());
    }

    void add_factor(TinyPreIntegrationErrorFactor *piefactor){
        details->problem->AddResidualBlock(piefactor, details->cauchy_loss.get()); // remove this kernel func.
    }

    template <typename T> void put_factor(std::unique_ptr<T> &&factor) {
        add_factor(factor.get());
        manage_factor(std::move(factor));
    }

    void manage_factor(std::unique_ptr<TinyReprojectionErrorFactor> &&factor) {
        details->managed_rpefactors.emplace_back(std::move(factor));
    }

    void manage_factor(
        std::unique_ptr<TinyPreIntegrationErrorFactor> &&factor) {
        details->managed_piefactors.emplace_back(std::move(factor));
    }

protected:
    std::unique_ptr<SolverDetails> details;

};
}

#endif