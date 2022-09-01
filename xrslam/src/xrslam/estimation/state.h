#ifndef XRSLAM_STATE_H
#define XRSLAM_STATE_H

#include <xrslam/common.h>

namespace xrslam {

class Frame;
class Track;
class Map;

enum ErrorStateLocation {
    ES_Q = 0,
    ES_P = 3,
    ES_V = 6,
    ES_BG = 9,
    ES_BA = 12,
    ES_SIZE = 15
};

struct ExtrinsicParams {
    quaternion q_cs;
    vector<3> p_cs;
};

using PoseState = Pose;

struct MotionState {
    MotionState() {
        v.setZero();
        bg.setZero();
        ba.setZero();
    }

    vector<3> v;
    vector<3> bg;
    vector<3> ba;
};

struct LandmarkState {
    LandmarkState() {
        inv_depth = 0;
        reprojection_error = 0;
    }

    double inv_depth;
    double reprojection_error;
};

} // namespace xrslam

#endif // XRSLAM_STATE_H
