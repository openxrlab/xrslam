#include <xrslam/geometry/lie_algebra.h>

namespace xrslam {

matrix<3> right_jacobian(const vector<3> &w) {
    static const double root2_eps =
        sqrt(std::numeric_limits<double>::epsilon());
    static const double root4_eps = sqrt(root2_eps);
    static const double qdrt720 = sqrt(sqrt(720.0));
    static const double qdrt5040 = sqrt(sqrt(5040.0));
    static const double sqrt24 = sqrt(24.0);
    static const double sqrt120 = sqrt(120.0);

    double angle = w.norm();
    double cangle = cos(angle);
    double sangle = sin(angle);
    double angle2 = angle * angle;

    double cos_term;
    // compute (1-cos(x))/x^2, its taylor expansion around 0 is
    // 1/2-x^2/24+x^4/720+o(x^6)
    if (angle > root4_eps * qdrt720) {
        cos_term = (1 - cangle) / angle2;
    } else { // use taylor expansion to avoid singularity
        cos_term = 0.5;
        if (angle > root2_eps * sqrt24) { // we have to include x^2 term
            cos_term -= angle2 / 24.0;
        }
    }

    double sin_term;
    // compute (x-sin(x))/x^3, its taylor expansion around 0 is
    // 1/6-x^2/120+x^4/5040+o(x^6)
    if (angle > root4_eps * qdrt5040) {
        sin_term = (angle - sangle) / (angle * angle2);
    } else {
        sin_term = 1.0 / 6.0;
        if (angle > root2_eps * sqrt120) { // we have to include x^2 term
            sin_term -= angle2 / 120.0;
        }
    }

    matrix<3> hat_w = hat(w);
    return matrix<3>::Identity() - cos_term * hat_w + sin_term * hat_w * hat_w;
}

matrix<3, 2> s2_tangential_basis(const vector<3> &x) {
    int d = 0;
    for (int i = 1; i < 3; ++i) {
        if (abs(x[i]) > abs(x[d]))
            d = i;
    }
    vector<3> b1 = x.cross(vector<3>::Unit((d + 1) % 3)).normalized();
    vector<3> b2 = x.cross(b1).normalized();
    return (matrix<3, 2>() << b1, b2).finished();
}

matrix<3, 2> s2_tangential_basis_barrel(const vector<3> &x) {
    vector<3> b1 =
        x.cross(abs(x.z()) < 0.866 ? vector<3>::UnitZ() : vector<3>::UnitY())
            .normalized();
    vector<3> b2 = x.cross(b1).normalized();
    return (matrix<3, 2>() << b1, b2).finished();
}

} // namespace xrslam
