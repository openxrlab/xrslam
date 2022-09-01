#ifndef XRSLAM_ESSENTIAL_H
#define XRSLAM_ESSENTIAL_H

#include <xrslam/common.h>

namespace xrslam {

void decompose_essential(const matrix<3> &E, matrix<3> &R1, matrix<3> &R2,
                         vector<3> &T);

std::vector<matrix<3>>
solve_essential_5pt(const std::array<vector<2>, 5> &points1,
                    const std::array<vector<2>, 5> &points2);

inline double essential_geometric_error(const matrix<3> &E, const vector<2> &p1,
                                        const vector<2> &p2) {
    vector<3> Ep1 = E * p1.homogeneous();
    double r = p2.homogeneous().transpose() * Ep1;
    return r * r / Ep1.segment<2>(0).squaredNorm();
}

} // namespace xrslam

#endif // XRSLAM_ESSENTIAL_H
