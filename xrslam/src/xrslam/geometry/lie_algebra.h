#ifndef XRSLAM_LIE_ALGEBRA_H
#define XRSLAM_LIE_ALGEBRA_H

#include <xrslam/common.h>

namespace xrslam {

inline matrix<3> hat(const vector<3> &w) {
    return (matrix<3>() << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0)
        .finished();
}

inline quaternion expmap(const vector<3> &w) {
    Eigen::AngleAxisd aa(w.norm(), w.stableNormalized());
    quaternion q;
    q = aa;
    return q;
}

inline vector<3> logmap(const quaternion &q) {
    Eigen::AngleAxisd aa(q);
    return aa.angle() * aa.axis();
}

matrix<3> right_jacobian(const vector<3> &w);
matrix<3, 2> s2_tangential_basis(const vector<3> &x);
matrix<3, 2> s2_tangential_basis_barrel(const vector<3> &x);

} // namespace xrslam

#endif // XRSLAM_LIE_ALGEBRA_H
