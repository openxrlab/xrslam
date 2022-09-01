#include <xrslam/geometry/homography.h>

namespace xrslam {

bool decompose_homography(const matrix<3> &H, matrix<3> &R1, matrix<3> &R2,
                          vector<3> &T1, vector<3> &T2, vector<3> &n1,
                          vector<3> &n2) {
    matrix<3> Hn = H / H.jacobiSvd().singularValues()(1);
    matrix<3> S = Hn.transpose() * Hn - matrix<3>::Identity();

    bool is_pure_rotation = true;
    for (int i = 0; i < 3 && is_pure_rotation; ++i) {
        for (int j = 0; j < 3 && is_pure_rotation; ++j) {
            if (abs(S(i, j)) > 1e-3) {
                is_pure_rotation = false;
            }
        }
    }

    if (is_pure_rotation) {
        // Pure rotation
        Eigen::JacobiSVD<matrix<3>> svd(H, Eigen::ComputeFullU |
                                               Eigen::ComputeFullV);
        R1 = svd.matrixU() * matrix<3>::Identity() * svd.matrixV().transpose();
        if (R1.determinant() < 0) {
            R1 = -R1;
        }
        R2 = R1;
        // R1 = R2 = Hn;
        T1 = T2 = vector<3>::Zero();
        n1 = n2 = vector<3>::Zero();
    } else {
        double Ms00 = S(1, 2) * S(1, 2) - S(1, 1) * S(2, 2);
        double Ms11 = S(0, 2) * S(0, 2) - S(0, 0) * S(2, 2);
        double Ms22 = S(0, 1) * S(0, 1) - S(0, 0) * S(1, 1);

        double sqrtMs00 = sqrt(Ms00);
        double sqrtMs11 = sqrt(Ms11);
        double sqrtMs22 = sqrt(Ms22);

        double nu = 2.0 * sqrt(1 + S.trace() - Ms00 - Ms11 - Ms22);
        double tenormsq = 2 + S.trace() - nu;

        vector<3> tstar1, tstar2;

        if (S(0, 0) > S(1, 1) && S(0, 0) > S(2, 2)) {
            double epslMs12 =
                (((S(0, 1) * S(0, 2) - S(0, 0) * S(1, 2)) < 0) ? -1 : 1);
            n1 << S(0, 0), S(0, 1) + sqrtMs22, S(0, 2) + epslMs12 * sqrtMs11;
            n2 << S(0, 0), S(0, 1) - sqrtMs22, S(0, 2) - epslMs12 * sqrtMs11;
            tstar1 = n1.norm() * n2 / S(0, 0);
            tstar2 = n2.norm() * n1 / S(0, 0);
        } else if (S(1, 1) > S(0, 0) && S(1, 1) > S(2, 2)) {
            double epslMs02 =
                (((S(1, 1) * S(0, 2) - S(0, 1) * S(1, 2)) < 0) ? -1 : 1);
            n1 << S(0, 1) + sqrtMs22, S(1, 1), S(1, 2) - epslMs02 * sqrtMs00;
            n2 << S(0, 1) - sqrtMs22, S(1, 1), S(1, 2) + epslMs02 * sqrtMs00;
            tstar2 = n2.norm() * n1 / S(1, 1);
            tstar1 = n1.norm() * n2 / S(1, 1);
        } else {
            double epslMs01 =
                (((S(1, 2) * S(0, 2) - S(0, 1) * S(2, 2)) < 0) ? -1 : 1);
            n1 << S(0, 2) + epslMs01 * sqrtMs11, S(1, 2) + sqrtMs00, S(2, 2);
            n2 << S(0, 2) - epslMs01 * sqrtMs11, S(1, 2) - sqrtMs00, S(2, 2);
            tstar1 = n1.norm() * n2 / S(2, 2);
            tstar2 = n2.norm() * n1 / S(2, 2);
        }
        n1.normalize();
        n2.normalize();
        tstar1 -= tenormsq * n1;
        tstar2 -= tenormsq * n2;
        R1 = Hn * (matrix<3>::Identity() - (tstar1 / nu) * n1.transpose());
        R2 = Hn * (matrix<3>::Identity() - (tstar2 / nu) * n2.transpose());
        tstar1 *= 0.5;
        tstar2 *= 0.5;
        T1 = R1 * tstar1;
        T2 = R2 * tstar2;
    }
    return !is_pure_rotation;
}

namespace {

inline matrix<3> to_matrix(const vector<9> &vec) {
    return (matrix<3>() << vec.segment<3>(0), vec.segment<3>(3),
            vec.segment<3>(6))
        .finished();
}

inline matrix<3>
solve_homography_normalized(const std::array<vector<2>, 4> &pa,
                            const std::array<vector<2>, 4> &pb) {
    matrix<8, 9> A = matrix<8, 9>::Zero();

    for (size_t i = 0; i < 4; ++i) {
        const vector<2> &a = pa[i];
        const vector<2> &b = pb[i];
        A(i * 2, 1) = -a(0);
        A(i * 2, 2) = a(0) * b(1);
        A(i * 2, 4) = -a(1);
        A(i * 2, 5) = a(1) * b(1);
        A(i * 2, 7) = -1;
        A(i * 2, 8) = b(1);
        A(i * 2 + 1, 0) = a(0);
        A(i * 2 + 1, 2) = -a(0) * b(0);
        A(i * 2 + 1, 3) = a(1);
        A(i * 2 + 1, 5) = -a(1) * b(0);
        A(i * 2 + 1, 6) = 1;
        A(i * 2 + 1, 8) = -b(0);
    }

    vector<9> h = A.jacobiSvd(Eigen::ComputeFullV).matrixV().col(8);
    return to_matrix(h);
}

} // namespace

matrix<3> solve_homography_4pt(const std::array<vector<2>, 4> &points1,
                               const std::array<vector<2>, 4> &points2) {
    static const double sqrt2 = sqrt(2.0);

    vector<2> pa_mean = vector<2>::Zero();
    vector<2> pb_mean = vector<2>::Zero();
    for (size_t i = 0; i < 4; ++i) {
        pa_mean += points1[i];
        pb_mean += points2[i];
    }
    pa_mean /= 4;
    pb_mean /= 4;

    double sa = 0;
    double sb = 0;

    for (size_t i = 0; i < 4; ++i) {
        sa += (points1[i] - pa_mean).norm();
        sb += (points2[i] - pb_mean).norm();
    }

    sa = 1.0 / (sqrt2 * sa);
    sb = 1.0 / (sqrt2 * sb);

    std::array<vector<2>, 4> na;
    std::array<vector<2>, 4> nb;
    for (size_t i = 0; i < 4; ++i) {
        na[i] = (points1[i] - pa_mean) * sa;
        nb[i] = (points2[i] - pb_mean) * sb;
    }

    matrix<3> NH = solve_homography_normalized(na, nb);

    matrix<3> Na, Nb;
    Nb << 1 / sb, 0, pb_mean(0), 0, 1 / sb, pb_mean(1), 0, 0, 1;
    Na << sa, 0, -sa * pa_mean(0), 0, sa, -sa * pa_mean(1), 0, 0, 1;

    return Nb * NH * Na;
}

} // namespace xrslam
