#include <ceres/ceres.h>
#include <xrslam/estimation/ceres/marginalization_factor.h>
#include <xrslam/estimation/ceres/preintegration_factor.h>
#include <xrslam/estimation/ceres/quaternion_parameterization.h>
#include <xrslam/estimation/ceres/reprojection_factor.h>
#include <xrslam/estimation/ceres/rotation_factor.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/estimation/state.h>
#include <xrslam/map/frame.h>

namespace xrslam {

struct Solver::SolverDetails {
    static Config *&config() {
        static Config *s_config = nullptr;
        return s_config;
    }
    std::unique_ptr<ceres::Problem> problem;
    std::unique_ptr<ceres::LossFunction> cauchy_loss;
    std::unique_ptr<ceres::LocalParameterization> quaternion_parameterization;
    std::vector<std::unique_ptr<ReprojectionErrorFactor>> managed_rpefactors;
    std::vector<std::unique_ptr<ReprojectionPriorFactor>> managed_rppfactors;
    std::vector<std::unique_ptr<RotationPriorFactor>> managed_ropfactors;
    std::vector<std::unique_ptr<PreIntegrationErrorFactor>> managed_piefactors;
    std::vector<std::unique_ptr<PreIntegrationPriorFactor>> managed_pipfactors;
    std::vector<std::unique_ptr<MarginalizationFactor>> managed_marfactors;
};

Solver::Solver() : details(std::make_unique<SolverDetails>()) {
    ceres::Problem::Options problem_options;
    problem_options.cost_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.loss_function_ownership = ceres::DO_NOT_TAKE_OWNERSHIP;
    problem_options.local_parameterization_ownership =
        ceres::DO_NOT_TAKE_OWNERSHIP;
    details->problem = std::make_unique<ceres::Problem>(problem_options);
    details->cauchy_loss = std::make_unique<ceres::CauchyLoss>(
        1.0); // TODO(jinyu): make configurable
    details->quaternion_parameterization =
        std::make_unique<QuaternionParameterization>();
}

Solver::~Solver() = default;

void Solver::init(Config *config) { SolverDetails::config() = config; }

std::unique_ptr<Solver> Solver::create() {
    return std::unique_ptr<Solver>(new Solver());
}

std::unique_ptr<ReprojectionErrorFactor>
Solver::create_reprojection_error_factor(Frame *frame, Track *track) {
    return std::make_unique<CeresReprojectionErrorFactor>(frame, track);
}

std::unique_ptr<ReprojectionPriorFactor>
Solver::create_reprojection_prior_factor(Frame *frame, Track *track) {
    return std::make_unique<CeresReprojectionPriorFactor>(frame, track);
}

std::unique_ptr<RotationPriorFactor>
Solver::create_rotation_prior_factor(Frame *frame, Track *track) {
    return std::make_unique<CeresRotationPriorFactor>(frame, track);
}

std::unique_ptr<PreIntegrationErrorFactor>
Solver::create_preintegration_error_factor(
    Frame *frame_i, Frame *frame_j, const PreIntegrator &preintegration) {
    return std::make_unique<CeresPreIntegrationErrorFactor>(frame_i, frame_j,
                                                            preintegration);
}

std::unique_ptr<PreIntegrationPriorFactor>
Solver::create_preintegration_prior_factor(
    Frame *frame_i, Frame *frame_j, const PreIntegrator &preintegration) {
    return std::make_unique<CeresPreIntegrationPriorFactor>(frame_i, frame_j,
                                                            preintegration);
}

std::unique_ptr<MarginalizationFactor>
Solver::create_marginalization_factor(Map *map) {
    return std::make_unique<CeresMarginalizationFactor>(map);
}

void Solver::add_frame_states(Frame *frame, bool with_motion) {
    details->problem->AddParameterBlock(
        frame->pose.q.coeffs().data(), 4,
        details->quaternion_parameterization.get());
    details->problem->AddParameterBlock(frame->pose.p.data(), 3);
    if (frame->tag(FT_FIX_POSE)) {
        details->problem->SetParameterBlockConstant(
            frame->pose.q.coeffs().data());
        details->problem->SetParameterBlockConstant(frame->pose.p.data());
    }
    if (with_motion) {
        details->problem->AddParameterBlock(frame->motion.v.data(), 3);
        details->problem->AddParameterBlock(frame->motion.bg.data(), 3);
        details->problem->AddParameterBlock(frame->motion.ba.data(), 3);
        if (frame->tag(FT_FIX_MOTION)) {
            details->problem->SetParameterBlockConstant(frame->motion.v.data());
            details->problem->SetParameterBlockConstant(
                frame->motion.bg.data());
            details->problem->SetParameterBlockConstant(
                frame->motion.ba.data());
        }
    }
}

void Solver::add_track_states(Track *track) {
    details->problem->AddParameterBlock(&(track->landmark.inv_depth), 1);
}

void Solver::add_factor(ReprojectionErrorFactor *rpefactor) {
    CeresReprojectionErrorFactor *rpecost =
        static_cast<CeresReprojectionErrorFactor *>(rpefactor);
    details->problem->AddResidualBlock(
        rpecost, nullptr,
        rpecost->frame->pose.q.coeffs().data(), rpecost->frame->pose.p.data(),
        rpecost->track->first_frame()->pose.q.coeffs().data(),
        rpecost->track->first_frame()->pose.p.data(),
        &(rpecost->track->landmark.inv_depth));
}

void Solver::add_factor(ReprojectionPriorFactor *rppfactor) {
    CeresReprojectionPriorFactor *rppcost =
        static_cast<CeresReprojectionPriorFactor *>(rppfactor);
    details->problem->AddResidualBlock(
        rppcost, details->cauchy_loss.get(),
        rppcost->rpefactor.frame->pose.q.coeffs().data(),
        rppcost->rpefactor.frame->pose.p.data());
}

void Solver::add_factor(RotationPriorFactor *ropfactor) {
    CeresRotationPriorFactor *ropcost =
        static_cast<CeresRotationPriorFactor *>(ropfactor);
    details->problem->AddResidualBlock(ropcost, details->cauchy_loss.get(),
                                       ropcost->frame->pose.q.coeffs().data());
}

void Solver::add_factor(PreIntegrationErrorFactor *piefactor) {
    CeresPreIntegrationErrorFactor *piecost =
        static_cast<CeresPreIntegrationErrorFactor *>(piefactor);
    details->problem->AddResidualBlock(
        piecost, nullptr, piecost->frame_i->pose.q.coeffs().data(),
        piecost->frame_i->pose.p.data(), piecost->frame_i->motion.v.data(),
        piecost->frame_i->motion.bg.data(), piecost->frame_i->motion.ba.data(),
        piecost->frame_j->pose.q.coeffs().data(),
        piecost->frame_j->pose.p.data(), piecost->frame_j->motion.v.data(),
        piecost->frame_j->motion.bg.data(), piecost->frame_j->motion.ba.data());
}

void Solver::add_factor(PreIntegrationPriorFactor *pipfactor) {
    CeresPreIntegrationPriorFactor *pipcost =
        static_cast<CeresPreIntegrationPriorFactor *>(pipfactor);
    details->problem->AddResidualBlock(
        pipcost, nullptr, pipcost->piefactor.frame_j->pose.q.coeffs().data(),
        pipcost->piefactor.frame_j->pose.p.data(),
        pipcost->piefactor.frame_j->motion.v.data(),
        pipcost->piefactor.frame_j->motion.bg.data(),
        pipcost->piefactor.frame_j->motion.ba.data());
}

void Solver::add_factor(MarginalizationFactor *factor) {
    std::vector<double *> params;
    for (size_t i = 0; i < factor->linearization_frames().size(); ++i) {
        Frame *frame = factor->linearization_frames()[i];
        params.emplace_back(frame->pose.q.coeffs().data());
        params.emplace_back(frame->pose.p.data());
        params.emplace_back(frame->motion.v.data());
        params.emplace_back(frame->motion.bg.data());
        params.emplace_back(frame->motion.ba.data());
    }
    details->problem->AddResidualBlock(
        static_cast<CeresMarginalizationFactor *>(factor), nullptr, params);
}

bool Solver::solve(bool verbose) {
    ceres::Solver::Options solver_options;
    ceres::Solver::Summary solver_summary;
    solver_options.linear_solver_type = ceres::SPARSE_SCHUR;
    solver_options.trust_region_strategy_type = ceres::DOGLEG;
    solver_options.max_num_iterations =
        (int)details->config()->solver_iteration_limit();
    solver_options.max_solver_time_in_seconds =
        details->config()->solver_time_limit();
    solver_options.num_threads = 1;
    solver_options.minimizer_progress_to_stdout = verbose;
    solver_options.update_state_every_iteration = true;
    ceres::Solve(solver_options, details->problem.get(), &solver_summary);
    return solver_summary.IsSolutionUsable();
}

void Solver::manage_factor(std::unique_ptr<ReprojectionErrorFactor> &&factor) {
    details->managed_rpefactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(std::unique_ptr<ReprojectionPriorFactor> &&factor) {
    details->managed_rppfactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(std::unique_ptr<RotationPriorFactor> &&factor) {
    details->managed_ropfactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(
    std::unique_ptr<PreIntegrationErrorFactor> &&factor) {
    details->managed_piefactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(
    std::unique_ptr<PreIntegrationPriorFactor> &&factor) {
    details->managed_pipfactors.emplace_back(std::move(factor));
}

void Solver::manage_factor(std::unique_ptr<MarginalizationFactor> &&factor) {
    details->managed_marfactors.emplace_back(std::move(factor));
}

} // namespace xrslam
