#ifndef XRSLAM_PNP_H
#define XRSLAM_PNP_H

#include <xrslam/common.h>
#include <xrslam/utility/ransac.h>
#include <xrslam/utility/parsac.h>
#include <xrslam/utility/imu_parsac.h>
#include <opencv2/calib3d/calib3d_c.h>

namespace xrslam {

std::vector<matrix<4>> solve_pnp_6pt(const std::array<vector<3>, 6> &Xs,
                                     const std::array<vector<2>, 6> &xs) {
    cv::Mat rvec, tvec;

    std::vector<cv::Point3f> P3D;
    std::vector<cv::Point2f> P2D;
    for (auto &X : Xs)
        P3D.emplace_back(X[0], X[1], X[2]);
    for (auto &x : xs)
        P2D.emplace_back(x[0], x[1]);

    cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1);

    cv::solvePnP(P3D, P2D, K, cv::noArray(), rvec, tvec, false, CV_EPNP);

    rvec.convertTo(rvec, CV_32F);
    tvec.convertTo(tvec, CV_32F);

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_32FC1);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));

    T.convertTo(T, CV_64FC1);
    matrix<4> pose =
        (matrix<4>() << T.at<double>(0, 0), T.at<double>(0, 1),
         T.at<double>(0, 2), T.at<double>(0, 3), T.at<double>(1, 0),
         T.at<double>(1, 1), T.at<double>(1, 2), T.at<double>(1, 3),
         T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2),
         T.at<double>(2, 3), T.at<double>(3, 0), T.at<double>(3, 1),
         T.at<double>(3, 2), T.at<double>(3, 3))
            .finished();

    std::vector<matrix<4>> poses;
    poses.push_back(pose);
    return poses;
}

std::vector<matrix<4>> solve_pnp_4pt(const std::array<vector<3>, 4> &Xs,
                                     const std::array<vector<2>, 4> &xs) {
    cv::Mat rvec, tvec;

    std::vector<cv::Point3f> P3D;
    std::vector<cv::Point2f> P2D;
    for (auto &X : Xs)
        P3D.emplace_back(X[0], X[1], X[2]);
    for (auto &x : xs)
        P2D.emplace_back(x[0], x[1]);

    cv::Mat K = cv::Mat::eye(3, 3, CV_32FC1);

    cv::solvePnP(P3D, P2D, K, cv::noArray(), rvec, tvec, false, CV_EPNP);

    rvec.convertTo(rvec, CV_32F);
    tvec.convertTo(tvec, CV_32F);

    cv::Mat R;
    cv::Rodrigues(rvec, R);
    cv::Mat T = cv::Mat::eye(4, 4, CV_32FC1);
    R.copyTo(T(cv::Rect(0, 0, 3, 3)));
    tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));

    T.convertTo(T, CV_64FC1);
    matrix<4> pose =
        (matrix<4>() << T.at<double>(0, 0), T.at<double>(0, 1),
         T.at<double>(0, 2), T.at<double>(0, 3), T.at<double>(1, 0),
         T.at<double>(1, 1), T.at<double>(1, 2), T.at<double>(1, 3),
         T.at<double>(2, 0), T.at<double>(2, 1), T.at<double>(2, 2),
         T.at<double>(2, 3), T.at<double>(3, 0), T.at<double>(3, 1),
         T.at<double>(3, 2), T.at<double>(3, 3))
            .finished();

    std::vector<matrix<4>> poses;
    poses.push_back(pose);
    return poses;
}

inline double pnp_reproject_error(const matrix<4> &T, const vector<3> &P1,
                                  const vector<2> &p2) {
    return (p2 - (T.block<3, 3>(0, 0) * P1 + T.block<3, 1>(0, 3)).hnormalized())
        .squaredNorm();
}

matrix<4> find_pnp_matrix(const std::vector<vector<3>> &Xs,
                          const std::vector<vector<2>> &xs,
                          std::vector<char> &inlier_mask,
                          double threshold = 1.0, double confidence = 0.999,
                          size_t max_iteration = 1000, int seed = 0) {
    struct PnpSolver {
        std::vector<matrix<4>>
        operator()(const std::array<vector<3>, 6> &samples1,
                   const std::array<vector<2>, 6> &samples2) const {
            return solve_pnp_6pt(samples1, samples2);
        }
        // std::vector<matrix<4>> operator()(const std::array<vector<3>, 4>
        // &samples1, const std::array<vector<2>, 4> &samples2) const {
        //     return solve_pnp_4pt(samples1, samples2);
        // }
    };
    struct PnpEvaluator {
        const matrix<4> &T;
        PnpEvaluator(const matrix<4> &T) : T(T) {}
        double operator()(const vector<3> &p1, const vector<2> &p2) const {
            return pnp_reproject_error(T, p1, p2);
        }
    };

    static const double t2 = 5.99;

    Ransac<6, matrix<4>, PnpSolver, PnpEvaluator> ransac(
        2.0 * t2 * threshold * threshold, confidence, max_iteration, seed);
    matrix<4> pose = ransac.solve(Xs, xs);
    inlier_mask.swap(ransac.inlier_mask);

    return pose;
}

matrix<4> find_pnp_matrix_parsac(const std::vector<vector<3>> &Xs,
                                 const std::vector<vector<2>> &xs,
                                 std::vector<char> &inlier_mask,
                                 double threshold = 1.0,
                                 double confidence = 0.999,
                                 size_t max_iteration = 1000, int seed = 0) {
    struct PnpSolver {
        std::vector<matrix<4>>
        operator()(const std::array<vector<3>, 6> &samples1,
                   const std::array<vector<2>, 6> &samples2) const {
            return solve_pnp_6pt(samples1, samples2);
        }
        // std::vector<matrix<4>> operator()(const std::array<vector<3>, 4>
        // &samples1, const std::array<vector<2>, 4> &samples2) const {
        //     return solve_pnp_4pt(samples1, samples2);
        // }
    };
    struct PnpEvaluator {
        const matrix<4> &T;
        PnpEvaluator(const matrix<4> &T) : T(T) {}
        double operator()(const vector<3> &p1, const vector<2> &p2) const {
            return pnp_reproject_error(T, p1, p2);
        }
    };

    static const double t2 = 5.99;
    static std::vector<float> binConfidences(400, 0.5);
    Parsac<6, matrix<4>, PnpSolver, PnpEvaluator> parsac(
        2.0 * t2 * threshold * threshold, confidence, max_iteration, seed);
    matrix<4> pose = parsac.solve(binConfidences, Xs, xs);

    inlier_mask.swap(parsac.inlier_mask);

    return pose;
}

matrix<4> find_pnp_matrix_parsac_imu(
    const std::vector<vector<3>> &Xs, const std::vector<vector<2>> &xs,
    const std::vector<size_t> &lens, const matrix<3> &R, const vector<3> &t,
    const double &dynamic_prob, const double &scale,
    std::vector<char> &inlier_mask, double threshold = 1.0,
    double confidence = 0.999, size_t max_iteration = 1000, int seed = 0) {
    struct PnpSolver {
        std::vector<matrix<4>>
        operator()(const std::array<vector<3>, 6> &samples1,
                   const std::array<vector<2>, 6> &samples2) const {
            return solve_pnp_6pt(samples1, samples2);
        }
        // std::vector<matrix<4>> operator()(const std::array<vector<3>, 4>
        // &samples1, const std::array<vector<2>, 4> &samples2) const {
        //     return solve_pnp_4pt(samples1, samples2);
        // }
    };
    struct PnpEvaluator {
        const matrix<4> &T;
        PnpEvaluator(const matrix<4> &T) : T(T) {}
        double operator()(const vector<3> &p1, const vector<2> &p2) const {
            return pnp_reproject_error(T, p1, p2);
        }
    };

    static const double t2 = 5.99;
    static std::vector<float> binConfidences(400, 0.5);
    IMU_Parsac<6, matrix<4>, PnpSolver, PnpEvaluator> imu_parsac(
        2.0 * t2 * threshold * threshold, confidence, max_iteration, seed);

    imu_parsac.SetPriorPose(R, t);
    imu_parsac.SetNormScale(scale);
    imu_parsac.SetLens(lens);
    imu_parsac.SetDynamicProbability(dynamic_prob);
    matrix<4> pose = imu_parsac.solve(binConfidences, Xs, xs);
    inlier_mask.swap(imu_parsac.inlier_mask);

    return pose;
}

} // namespace xrslam

#endif
