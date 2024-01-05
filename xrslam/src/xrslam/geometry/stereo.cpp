#include <xrslam/geometry/essential.h>
#include <xrslam/geometry/homography.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/geometry/wahba.h>
#include <xrslam/utility/ransac.h>
#include <xrslam/utility/parsac.h>

namespace xrslam {

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1,
                                const std::vector<vector<2>> &points2,
                                double threshold, double confidence,
                                size_t max_iteration, int seed) {
    std::vector<char> _;
    return find_essential_matrix(points1, points2, _, threshold, confidence,
                                 max_iteration, seed);
}

matrix<3> find_rotation_matrix(const std::vector<vector<3>> &points1,
                               const std::vector<vector<3>> &points2,
                               double threshold, double confidence,
                               size_t max_iteration, int seed) {
    std::vector<char> _;
    return find_rotation_matrix(points1, points2, _, threshold, confidence,
                                max_iteration, seed);
}

matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1,
                                 const std::vector<vector<2>> &points2,
                                 double threshold, double confidence,
                                 size_t max_iteration, int seed) {
    std::vector<char> _;
    return find_homography_matrix(points1, points2, _, threshold, confidence,
                                  max_iteration, seed);
}

matrix<3> find_essential_matrix(const std::vector<vector<2>> &points1,
                                const std::vector<vector<2>> &points2,
                                std::vector<char> &inlier_mask,
                                double threshold, double confidence,
                                size_t max_iteration, int seed) {
    struct EssentialSolver {
        std::vector<matrix<3>>
        operator()(const std::array<vector<2>, 5> &samples1,
                   const std::array<vector<2>, 5> &samples2) const {
            return solve_essential_5pt(samples1, samples2);
        }
    };
    struct EssentialEvaluator {
        const matrix<3> &E;
        const matrix<3> Et;
        EssentialEvaluator(const matrix<3> &E) : E(E), Et(E.transpose()) {}
        double operator()(const vector<2> &p1, const vector<2> &p2) const {
            return essential_geometric_error(E, p1, p2) +
                   essential_geometric_error(Et, p2, p1);
        }
    };
    static const double t1 = 3.84;
    Ransac<5, matrix<3>, EssentialSolver, EssentialEvaluator> ransac(
        2.0 * t1 * threshold * threshold, confidence, max_iteration, seed);
    matrix<3> E = ransac.solve(points1, points2);
    inlier_mask.swap(ransac.inlier_mask);
    return E;
}

matrix<3> find_rotation_matrix(const std::vector<vector<3>> &points1,
                               const std::vector<vector<3>> &points2,
                               std::vector<char> &inlier_mask, double threshold,
                               double confidence, size_t max_iteration,
                               int seed) {
    struct RotationSolver {
        matrix<3> operator()(const std::array<vector<3>, 2> &samples1,
                             const std::array<vector<3>, 2> &samples2) const {
            return solve_rotation_2pt(samples1, samples2);
        }
    };
    struct RotationEvaluator {
        const matrix<3> &R;
        RotationEvaluator(const matrix<3> &R) : R(R) {}
        double operator()(const vector<3> &p1, const vector<3> &p2) const {
            return acos((R * p1).dot(p2));
        }
    };
    static const double t2 = 5.99;
    Ransac<2, matrix<3>, RotationSolver, RotationEvaluator> ransac(
        t2 * threshold * threshold, confidence, max_iteration, seed);
    matrix<3> R = ransac.solve(points1, points2);
    inlier_mask.swap(ransac.inlier_mask);
    return R;
}

matrix<3> find_homography_matrix(const std::vector<vector<2>> &points1,
                                 const std::vector<vector<2>> &points2,
                                 std::vector<char> &inlier_mask,
                                 double threshold, double confidence,
                                 size_t max_iteration, int seed) {
    struct HomographySolver {
        matrix<3> operator()(const std::array<vector<2>, 4> &samples1,
                             const std::array<vector<2>, 4> &samples2) const {
            return solve_homography_4pt(samples1, samples2);
        }
    };
    struct HomographyEvaluator {
        const matrix<3> &H;
        const matrix<3> Hinv;
        HomographyEvaluator(const matrix<3> &H) : H(H), Hinv(H.inverse()) {}
        double operator()(const vector<2> &p1, const vector<2> &p2) const {
            return homography_geometric_error(H, p1, p2) +
                   homography_geometric_error(Hinv, p2, p1);
        }
    };
    static const double t2 = 5.99;
    Ransac<4, matrix<3>, HomographySolver, HomographyEvaluator> ransac(
        2.0 * t2 * threshold * threshold, confidence, max_iteration, seed);
    matrix<3> H = ransac.solve(points1, points2);
    inlier_mask.swap(ransac.inlier_mask);
    return H;
}

// -----------------------------
//        parsac version
// -----------------------------
matrix<3> find_essential_matrix_parsac(const std::vector<vector<2>> &points1,
                                       const std::vector<vector<2>> &points2,
                                       std::vector<char> &inlier_mask,
                                       double threshold, double confidence,
                                       size_t max_iteration, int seed) {
    struct EssentialSolver {
        std::vector<matrix<3>>
        operator()(const std::array<vector<2>, 5> &samples1,
                   const std::array<vector<2>, 5> &samples2) const {
            return solve_essential_5pt(samples1, samples2);
        }
    };
    struct EssentialEvaluator {
        const matrix<3> &E;
        const matrix<3> Et;
        EssentialEvaluator(const matrix<3> &E) : E(E), Et(E.transpose()) {}
        double operator()(const vector<2> &p1, const vector<2> &p2) const {
            float error = essential_geometric_error(E, p1, p2) +
                          essential_geometric_error(Et, p2, p1);
            // std::cout << error << std::endl;
            return essential_geometric_error(E, p1, p2) +
                   essential_geometric_error(Et, p2, p1);
        }
    };
    static const double t1 = 3.84;
    static std::vector<float> binConfidences(400, 0.5);
    Parsac<5, matrix<3>, EssentialSolver, EssentialEvaluator> parsac(
        2.0 * t1 * threshold * threshold, confidence, max_iteration, seed);
    matrix<3> E = parsac.solve(binConfidences, points1, points2);
    inlier_mask.swap(parsac.inlier_mask);
    return E;
}

matrix<3> find_homography_matrix_parsac(const std::vector<vector<2>> &points1,
                                        const std::vector<vector<2>> &points2,
                                        std::vector<char> &inlier_mask,
                                        double threshold, double confidence,
                                        size_t max_iteration, int seed) {
    struct HomographySolver {
        matrix<3> operator()(const std::array<vector<2>, 4> &samples1,
                             const std::array<vector<2>, 4> &samples2) const {
            return solve_homography_4pt(samples1, samples2);
        }
    };
    struct HomographyEvaluator {
        const matrix<3> &H;
        const matrix<3> Hinv;
        HomographyEvaluator(const matrix<3> &H) : H(H), Hinv(H.inverse()) {}
        double operator()(const vector<2> &p1, const vector<2> &p2) const {
            return homography_geometric_error(H, p1, p2) +
                   homography_geometric_error(Hinv, p2, p1);
        }
    };
    static const double t2 = 5.99;
    static std::vector<float> binConfidences(400, 0.5);
    Parsac<4, matrix<3>, HomographySolver, HomographyEvaluator> parsac(
        2.0 * t2 * threshold * threshold, confidence, max_iteration, seed);
    matrix<3> H = parsac.solve(binConfidences, points1, points2);
    inlier_mask.swap(parsac.inlier_mask);
    return H;
}

} // namespace xrslam
