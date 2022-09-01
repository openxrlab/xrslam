#include <ceres/ceres.h>
#include <gtest/gtest.h>
#include <xrslam/common.h>
#include <xrslam/estimation/ceres/cost_function_validator.h>
#include <xrslam/estimation/ceres/quaternion_parameterization.h>
#include <xrslam/utility/random.h>

using namespace xrslam;

struct SE3CostFunction : ceres::SizedCostFunction<3, 4, 3> {
    SE3CostFunction(vector<3> p1, vector<3> p2) : p1(p1), p2(p2) {}

    bool Evaluate(const double *const *parameters, double *residuals,
                  double **jacobians) const override {
        const_map<quaternion> q(parameters[0]);
        const_map<vector<3>> p(parameters[1]);
        map<vector<3>> r(residuals);

        r = p1 - (q * p2 + p);

        if (jacobians) {
            if (jacobians[0]) {
                map<matrix<3, 4, true>> dr_dq(jacobians[0]);
                dr_dq.setZero();
                dr_dq.block<3, 3>(0, 0) = q.toRotationMatrix() * hat(p2);
            }
            if (jacobians[1]) {
                map<matrix<3, 3, true>> dr_dp(jacobians[1]);
                dr_dp.setIdentity();
                dr_dp *= -1;
            }
        }

        return true;
    }
    vector<3> p1, p2;
};

TEST(test_SE3CostFunction, se3_cost_function) {
    UniformNoise<double> noise;
    vector<3> p2 = vector<3>::Random();
    quaternion q =
        Eigen::AngleAxisd(noise.next() * M_PI, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(noise.next() * M_PI, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(noise.next() * M_PI, Eigen::Vector3d::UnitZ());
    vector<3> p = vector<3>::Random();
    vector<3> p1 = q * p2 + p;

    vector<3> n = {noise.next() * 0.1, noise.next() * 0.1, noise.next() * 0.1};
    p1 = p1 + n;

    std::cout << "Before:" << std::endl;
    std::cout << "p2: " << p2.transpose() << std::endl;
    std::cout << "q: " << q.coeffs().transpose() << std::endl;
    std::cout << "p: " << p.transpose() << std::endl;
    std::cout << "p1: " << p.transpose() << std::endl;
    std::cout << "n: " << n.transpose() << std::endl;
    vector<3> r = p1 - (q * p2 + p);
    std::cout << "r: " << r.transpose() << std::endl;

    CostFunctionValidator::Options validate_options;
    validate_options.max_jacobian_error = 2.0e-6;
    CostFunctionValidator validator(validate_options);
    validator.AddParameterBlock(q.coeffs().data(), 4,
                                new QuaternionParameterization());
    validator.AddParameterBlock(p.data(), 3);
    validator.AddResidualBlock(new SE3CostFunction(p1, p2), nullptr,
                               q.coeffs().data(), p.data());

    ASSERT_TRUE(validator.Validate());

    ceres::Problem::Options problem_options;
    ceres::Problem problem(problem_options);
    problem.AddParameterBlock(q.coeffs().data(), 4,
                              new QuaternionParameterization());
    problem.AddParameterBlock(p.data(), 3);
    problem.AddResidualBlock(new SE3CostFunction(p1, p2), nullptr,
                             q.coeffs().data(), p.data());
    ceres::Solver::Options solver_options;
    ceres::Solver::Summary solver_summary;
    ceres::Solve(solver_options, &problem, &solver_summary);

    std::cout << "After:" << std::endl;
    std::cout << "p2: " << p2.transpose() << std::endl;
    std::cout << "q: " << q.coeffs().transpose() << std::endl;
    std::cout << "p: " << p.transpose() << std::endl;
    std::cout << "p1: " << p.transpose() << std::endl;
    std::cout << "n: " << n.transpose() << std::endl;
    r = p1 - (q * p2 + p);
    std::cout << "r: " << r.transpose() << std::endl;

    ASSERT_TRUE(validator.Validate());
}
