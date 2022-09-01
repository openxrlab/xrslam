#ifndef XRSLAM_HOMOGRAPHY_H
#define XRSLAM_HOMOGRAPHY_H

#include <xrslam/common.h>

namespace xrslam {

bool decompose_homography(const matrix<3> &H, matrix<3> &R1, matrix<3> &R2,
                          vector<3> &T1, vector<3> &T2, vector<3> &n1,
                          vector<3> &n2);

// p2 = H * p1
matrix<3> solve_homography_4pt(const std::array<vector<2>, 4> &points1,
                               const std::array<vector<2>, 4> &points2);

// d(p2, H * p1)
inline double homography_geometric_error(const matrix<3> &H,
                                         const vector<2> &p1,
                                         const vector<2> &p2) {
    return (p2 - (H * p1.homogeneous()).hnormalized()).squaredNorm();
}

} // namespace xrslam

#endif // XRSLAM_HOMOGRAPHY_H
