#ifndef XRSLAM_COST_FUNCTION_VALIDATOR_H
#define XRSLAM_COST_FUNCTION_VALIDATOR_H

#include <ceres/ceres.h>
#include <xrslam/common.h>

namespace xrslam {

class CostFunctionValidator {
    struct ParameterBlockInfo {
        int size;
        const ceres::LocalParameterization *local_parameterization = nullptr;
        bool checked = true;
    };
    struct ResidualBlockInfo {
        std::vector<const double *> parameter_blocks;
        const ceres::LossFunction *loss_function;
        bool checked = true;
    };

  public:
    struct Options {
        double fd_epsilon = 1.0e-9;
        double max_jacobian_error = 1.0e-7; // report all
        bool return_on_error = false;
    };

    CostFunctionValidator(const Options &options) : options(options) {
        if (!std::numeric_limits<double>::has_quiet_NaN) {
            log_message(XRSLAM_LOG_NOTICE,
                        "Implementation does not support quiet NaN. Detection "
                        "of uninitialized values will be unavailable.");
        }
    }

    bool AddParameterBlock(
        const double *values, int size,
        const ceres::LocalParameterization *local_parameterization = nullptr) {
        if (parameter_block_info.count(values)) {
            log_message(XRSLAM_LOG_WARNING, "repeated.");
            return false;
        }

        if (local_parameterization) {
            if (size != local_parameterization->GlobalSize()) {
                log_message(XRSLAM_LOG_WARNING, "size mismatch");
                return false;
            }
        }

        ParameterBlockInfo &info = parameter_block_info[values];
        info.size = size;
        info.local_parameterization = local_parameterization;
        info.checked = true;

        return true;
    }

    bool AddResidualBlock(const ceres::CostFunction *cost_function,
                          const ceres::LossFunction *loss_function,
                          const std::vector<const double *> &parameters) {
        if (residual_block_info.count(cost_function)) {
            log_message(XRSLAM_LOG_WARNING, "size mismatch");
            return false;
        }

        if (parameters.size() !=
            cost_function->parameter_block_sizes().size()) {
            log_message(XRSLAM_LOG_WARNING, "repeated.");
            return false;
        }

        for (size_t i = 0; i < parameters.size(); ++i) {
            const double *values = parameters[i];
            if (parameter_block_info.count(values)) {
                if (parameter_block_info.at(values).size !=
                    cost_function->parameter_block_sizes()[i]) {
                    log_message(XRSLAM_LOG_WARNING, "size mismatch");
                    return false;
                }
            } else {
                ParameterBlockInfo &pinfo = parameter_block_info[values];
                pinfo.size = cost_function->parameter_block_sizes()[i];
                pinfo.local_parameterization = nullptr;
                pinfo.checked = true;
            }
        }

        ResidualBlockInfo &info = residual_block_info[cost_function];
        info.parameter_blocks = parameters;
        info.loss_function = loss_function;
        info.checked = true;

        return true;
    }

    template <typename... P>
    bool AddResidualBlock(const ceres::CostFunction *cost_function,
                          const ceres::LossFunction *loss_function,
                          P &&...param) {
        return AddResidualBlock(cost_function, loss_function,
                                {std::forward<P>(param)...});
    }

    void SetParameterBlockChecked(const double *values) {
        if (parameter_block_info.count(values)) {
            parameter_block_info.at(values).checked = true;
        }
    }

    void SetParameterBlockIgnored(const double *values) {
        if (parameter_block_info.count(values)) {
            parameter_block_info.at(values).checked = false;
        }
    }

    void SetResidualBlockChecked(const ceres::CostFunction *cost_function) {
        if (residual_block_info.count(cost_function)) {
            residual_block_info.at(cost_function).checked = true;
        }
    }

    void SetResidualBlockIgnored(const ceres::CostFunction *cost_function) {
        if (residual_block_info.count(cost_function)) {
            residual_block_info.at(cost_function).checked = false;
        }
    }

    bool Validate() const {
        bool success = true;
        for (const auto &[cost_function, rbinfo] : residual_block_info) {
            if (!rbinfo.checked)
                continue;
            if (!validate_residual_block(cost_function, rbinfo)) {
                if (options.return_on_error) {
                    return false;
                } else {
                    success = false;
                }
            }
        }
        return success;
    }

  private:
    static double generate_nan() { return std::nan("0xFEEDCAFEC0C0"); }
    static bool identify_nan(const double &nan) {
        std::uint64_t ret;
        memcpy(&ret, &nan, sizeof(nan));
        return (ret == 0x7ff8feedcafec0c0);
    }

    static bool has_uninitialized(const matrix<> &value) {
        for (int i = 0; i < value.rows(); ++i) {
            for (int j = 0; j < value.cols(); ++j) {
                if (identify_nan(value(i, j))) {
                    return true;
                }
            }
        }
        return false;
    }

    static bool has_abnormal(const matrix<> &value) {
        for (int i = 0; i < value.rows(); ++i) {
            for (int j = 0; j < value.cols(); ++j) {
                if (!std::isfinite(value(i, j)) && !identify_nan(value(i, j))) {
                    return true;
                }
            }
        }
        return false;
    }

    bool validate_residual_block(const ceres::CostFunction *cost_function,
                                 const ResidualBlockInfo &rbinfo) const {
        bool has_residual_central = true;

        vector<> residual_central;
        residual_central.resize(cost_function->num_residuals());
        residual_central.setConstant(generate_nan());

        std::vector<matrix<Eigen::Dynamic, Eigen::Dynamic, true>>
            global_jacobians(rbinfo.parameter_blocks.size());
        std::vector<double *> global_jacobians_ptr(
            rbinfo.parameter_blocks.size());
        for (size_t i = 0; i < rbinfo.parameter_blocks.size(); ++i) {
            global_jacobians[i].resize(
                cost_function->num_residuals(),
                cost_function->parameter_block_sizes()[i]);
            global_jacobians[i].setConstant(generate_nan());
            global_jacobians_ptr[i] = global_jacobians[i].data();
        }

        if (!cost_function->Evaluate(rbinfo.parameter_blocks.data(),
                                     residual_central.data(),
                                     global_jacobians_ptr.data())) {
            log_message(XRSLAM_LOG_WARNING,
                        "Cost function failed to evaluate.");
            return false; // in this case we can only quit.
        }
        if (has_uninitialized(residual_central)) {
            log_message(XRSLAM_LOG_WARNING,
                        "Residual contains uninitialized value.");
            has_residual_central = false;
        }
        if (has_abnormal(residual_central)) {
            log_message(XRSLAM_LOG_WARNING, "Residual contains Inf/NaN.");
            has_residual_central = false;
        }

        for (size_t i = 0; i < rbinfo.parameter_blocks.size(); ++i) {
            if (has_uninitialized(global_jacobians[i])) {
                log_message(XRSLAM_LOG_WARNING,
                            "Global Jacobian contains uninitialized value.");
            }
            if (has_abnormal(global_jacobians[i])) {
                log_message(XRSLAM_LOG_WARNING,
                            "Global Jacobian contains Inf/NaN.");
            }
        }

        std::vector<matrix<>> local_jacobians(rbinfo.parameter_blocks.size());
        for (size_t i = 0; i < rbinfo.parameter_blocks.size(); ++i) {
            const ParameterBlockInfo &pbinfo =
                parameter_block_info.at(rbinfo.parameter_blocks[i]);
            if (pbinfo.local_parameterization) {
                matrix<Eigen::Dynamic, Eigen::Dynamic, true> local_jacobian;
                local_jacobian.resize(
                    cost_function->num_residuals(),
                    pbinfo.local_parameterization->LocalSize());
                local_jacobian.setConstant(generate_nan());

                pbinfo.local_parameterization->MultiplyByJacobian(
                    rbinfo.parameter_blocks[i], cost_function->num_residuals(),
                    global_jacobians[i].data(), local_jacobian.data());

                if (has_uninitialized(local_jacobian)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Local Jacobian contains uninitialized value.");
                }
                if (has_abnormal(local_jacobian)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Local Jacobian contains Inf/NaN.");
                }

                local_jacobians[i] = local_jacobian;
            } else {
                local_jacobians[i] = global_jacobians[i];
            }
        }

        if (!has_residual_central) {
            log_message(XRSLAM_LOG_WARNING,
                        "Cannot compute finite difference: cost function "
                        "failed to evaluate.");
            return false;
        }

        bool success = true;
        for (size_t i = 0; i < rbinfo.parameter_blocks.size(); ++i) {
            const ParameterBlockInfo &pbinfo =
                parameter_block_info.at(rbinfo.parameter_blocks[i]);
            if (!pbinfo.checked)
                continue;
            matrix<> jacobian_fd;
            if (finite_difference(cost_function, rbinfo, pbinfo, i,
                                  residual_central, jacobian_fd)) {
                double err = (local_jacobians[i] - jacobian_fd)
                                 .lpNorm<Eigen::Infinity>();
                if (err > options.max_jacobian_error) {
                    std::cout << "!!! Evaluating Jacobian for parameter " << i
                              << " of " << rbinfo.parameter_blocks.size()
                              << std::endl;
                    std::cout << "!!! Result inconsistent: max error = " << err
                              << std::endl;
                    std::cout << "Analytical:" << std::endl;
                    std::cout << local_jacobians[i] << std::endl;
                    std::cout << "Finite Difference:" << std::endl;
                    std::cout << jacobian_fd << std::endl;
                    std::cout << std::endl;
                    success = false;
                }
            }
        }
        return success;
    }

    bool finite_difference(const ceres::CostFunction *cost_function,
                           const ResidualBlockInfo &rbinfo,
                           const ParameterBlockInfo &pbinfo,
                           size_t parameter_index,
                           const vector<> &residual_central,
                           matrix<> &jacobian_fd) const {
        const double *x = rbinfo.parameter_blocks[parameter_index];
        vector<> active_parameter = const_map<vector<>>(x, pbinfo.size);

        std::vector<const double *> parameter_blocks = rbinfo.parameter_blocks;
        parameter_blocks[parameter_index] = active_parameter.data();

        vector<> residual_forward;
        residual_forward.resize(cost_function->num_residuals());
        if (!pbinfo.local_parameterization) {
            jacobian_fd.resize(cost_function->num_residuals(), pbinfo.size);
            for (int i = 0; i < pbinfo.size; ++i) {
                double x = active_parameter(i);
                active_parameter(i) += options.fd_epsilon;
                residual_forward.setConstant(generate_nan());
                if (!cost_function->Evaluate(parameter_blocks.data(),
                                             residual_forward.data(),
                                             nullptr)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Cannot compute finite difference: cost "
                                "function failed to evaluate.");
                    return false;
                }
                if (has_uninitialized(residual_forward)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Cannot compute finite difference: residual "
                                "contains uninitialized value.");
                    return false;
                }
                if (has_abnormal(residual_forward)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Residual contains Inf/NaN.");
                }
                vector<> jacobian_i =
                    (residual_forward - residual_central) / options.fd_epsilon;
                active_parameter(i) = x;
                jacobian_fd.col(i) = jacobian_i;
            }
        } else {
            jacobian_fd.resize(cost_function->num_residuals(),
                               pbinfo.local_parameterization->LocalSize());
            for (int i = 0; i < pbinfo.local_parameterization->LocalSize();
                 ++i) {
                vector<> dx =
                    vector<>::Zero(pbinfo.local_parameterization->LocalSize());
                dx(i) = options.fd_epsilon;
                pbinfo.local_parameterization->Plus(x, dx.data(),
                                                    active_parameter.data());
                residual_forward.setConstant(generate_nan());
                if (!cost_function->Evaluate(parameter_blocks.data(),
                                             residual_forward.data(),
                                             nullptr)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Cannot compute finite difference: cost "
                                "function failed to evaluate.");
                    return false;
                }
                if (has_uninitialized(residual_forward)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Cannot compute finite difference: residual "
                                "contains uninitialized value.");
                    return false;
                }
                if (has_abnormal(residual_forward)) {
                    log_message(XRSLAM_LOG_WARNING,
                                "Residual contains Inf/NaN.");
                }
                vector<> jacobian_i =
                    (residual_forward - residual_central) / options.fd_epsilon;
                jacobian_fd.col(i) = jacobian_i;
            }
        }
        return true;
    }

    std::map<const double *, ParameterBlockInfo> parameter_block_info;
    std::map<const ceres::CostFunction *, ResidualBlockInfo>
        residual_block_info;
    Options options;
};

} // namespace xrslam

#endif // XRSLAM_COST_FUNCTION_VALIDATOR_H
