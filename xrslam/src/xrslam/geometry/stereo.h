#ifndef XRSLAM_STEREO_H
#define XRSLAM_STEREO_H

#include <xrslam/common.h>

namespace xrslam {

inline vector<2> apply_k(const vector<3> &p, const matrix<3> &K) {
    return {p(0) / p(2) * K(0, 0) + K(0, 2), p(1) / p(2) * K(1, 1) + K(1, 2)};
}

inline vector<3> remove_k(const vector<2> &p, const matrix<3> &K) {
    return vector<3>{(p(0) - K(0, 2)) / K(0, 0), (p(1) - K(1, 2)) / K(1, 1), 1}
        .normalized();
}

inline matrix<2, 3> dproj_dp(const vector<3> &p) {
    return (matrix<2, 3>() << 1.0 / p.z(), 0.0, -p.x() / (p.z() * p.z()), 0.0,
            1.0 / p.z(), -p.y() / (p.z() * p.z()))
        .finished();
}

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1,
                                const std::vector<vector<2>> &points2,
                                double threshold = 1.0,
                                double confidence = 0.999,
                                size_t max_iteration = 1000, int seed = 0);
matrix<3> find_rotation_matrix(const std::vector<vector<3>> &points1,
                               const std::vector<vector<3>> &points2,
                               double threshold = 1.0,
                               double confidence = 0.999,
                               size_t max_iteration = 1000, int seed = 0);
matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1,
                                 const std::vector<vector<2>> &points2,
                                 double threshold = 1.0,
                                 double confidence = 0.999,
                                 size_t max_iteration = 1000, int seed = 0);
matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1,
                                const std::vector<vector<2>> &points2,
                                std::vector<char> &inlier_mask,
                                double threshold = 1.0,
                                double confidence = 0.999,
                                size_t max_iteration = 1000, int seed = 0);
matrix<3> find_rotation_matrix(const std::vector<vector<3>> &points1,
                               const std::vector<vector<3>> &points2,
                               std::vector<char> &inlier_mask,
                               double threshold = 1.0,
                               double confidence = 0.999,
                               size_t max_iteration = 1000, int seed = 0);
matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1,
                                 const std::vector<vector<2>> &points2,
                                 std::vector<char> &inlier_mask,
                                 double threshold = 1.0,
                                 double confidence = 0.999,
                                 size_t max_iteration = 1000, int seed = 0);

matrix<3> find_essential_matrix_parsac(const std::vector<vector<2>> &points1,
                                       const std::vector<vector<2>> &points2,
                                       std::vector<char> &inlier_mask,
                                       double threshold = 1.0,
                                       double confidence = 0.999,
                                       size_t max_iteration = 1000,
                                       int seed = 0);
matrix<3> find_homography_matrix_parsac(const std::vector<vector<2>> &points1,
                                        const std::vector<vector<2>> &points2,
                                        std::vector<char> &inlier_mask,
                                        double threshold = 1.0,
                                        double confidence = 0.999,
                                        size_t max_iteration = 1000,
                                        int seed = 0);

inline vector<4> triangulate_point(const matrix<3, 4> &P1,
                                   const matrix<3, 4> &P2,
                                   const vector<3> &point1,
                                   const vector<3> &point2) {
    matrix<4> A;
    A.row(0) = point1(0) * P1.row(2) - point1(2) * P1.row(0);
    A.row(1) = point1(1) * P1.row(2) - point1(2) * P1.row(1);
    A.row(2) = point2(0) * P2.row(2) - point2(2) * P2.row(0);
    A.row(3) = point2(1) * P2.row(2) - point2(2) * P2.row(1);
    return A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
}

inline vector<4> triangulate_point(const std::vector<matrix<3, 4>> &Ps,
                                   const std::vector<vector<3>> &points) {
    matrix<Eigen::Dynamic, 4> A(points.size() * 2, 4);
    for (size_t i = 0; i < points.size(); ++i) {
        A.row(i * 2 + 0) =
            points[i](0) * Ps[i].row(2) - points[i](2) * Ps[i].row(0);
        A.row(i * 2 + 1) =
            points[i](1) * Ps[i].row(2) - points[i](2) * Ps[i].row(1);
    }
    return A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(3);
}

} // namespace xrslam

#endif // XRSLAM_STEREO_H
