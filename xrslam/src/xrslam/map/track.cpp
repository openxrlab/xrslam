#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/map/track.h>

namespace xrslam {

Track::Track() = default;

Track::~Track() = default;

const vector<3> &Track::get_keypoint(Frame *frame) const {
    return frame->get_keypoint(keypoint_refs.at(frame));
}

void Track::add_keypoint(Frame *frame, size_t keypoint_index) {
    keypoint_refs[frame] = keypoint_index;
    frame->tracks[keypoint_index] = this;
    frame->reprojection_error_factors[keypoint_index] =
        Solver::create_reprojection_error_factor(frame, this);
}

void Track::remove_keypoint(Frame *frame, bool suicide_if_empty) {
    size_t keypoint_index = keypoint_refs.at(frame);
    std::optional<vector<3>> landmark;
    if (frame == first_frame()) {
        landmark = get_landmark_point();
    }
    frame->tracks[keypoint_index] = nullptr;
    frame->reprojection_error_factors[keypoint_index].reset();
    keypoint_refs.erase(frame);
    if (keypoint_refs.size() > 0) {
        if (landmark.has_value()) {
            set_landmark_point(landmark.value());
        }
    } else {
        tag(TT_VALID) = false;
        if (suicide_if_empty) {
            map->recycle_track(this);
        }
    }
}

std::optional<vector<3>> Track::triangulate() const {
    std::vector<matrix<3, 4>> Ps;
    std::vector<vector<3>> ps;
    for (const auto &[frame, keypoint_index] : keypoint_map()) {
        matrix<3, 4> P;
        matrix<3, 3> R;
        vector<3> T;
        auto pose = frame->get_pose(frame->camera);
        R = pose.q.conjugate().matrix();
        T = -(R * pose.p);
        P << R, T;
        Ps.push_back(P);
        ps.push_back(frame->get_keypoint(keypoint_index));
    }

    bool is_valid = true;
    vector<4> hlandmark = triangulate_point(Ps, ps);
    for (size_t i = 0; i < ps.size(); ++i) {
        vector<3> qi = Ps[i] * hlandmark;
        if (!(qi[2] * hlandmark[3] > 0)) {
            is_valid = false;
            break;
        }
    }
    if (is_valid) {
        return hlandmark.hnormalized();
    } else {
        return {};
    }
}

double Track::triangulation_angle(const vector<3> &p) const {
    vector<3> nref = (p - first_frame()->get_pose(first_frame()->camera).p)
                         .stableNormalized();
    double max_angle = 0;
    for (const auto &[frame, keypoint_index] : keypoint_map()) {
        auto pose = frame->get_pose(frame->camera);
        vector<3> n = (p - pose.p).stableNormalized();
        max_angle = std::max(max_angle, acos(n.dot(nref)));
    }
    return max_angle;
}

vector<3> Track::get_landmark_point() const {
    const auto &[frame, keypoint_index] = first_keypoint();
    auto camera = frame->get_pose(frame->camera);
    return camera.q * frame->get_keypoint(keypoint_index) / landmark.inv_depth +
           camera.p;
}

void Track::set_landmark_point(const vector<3> &p) {
    const auto [frame, keypoint_index] = first_keypoint();
    auto camera = frame->get_pose(frame->camera);
    landmark.inv_depth = 1.0 / (camera.q.conjugate() * (p - camera.p)).norm();
}

} // namespace xrslam
