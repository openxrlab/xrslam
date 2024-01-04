#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/geometry/pnp.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/unique_timer.h>

namespace xrslam {

SlidingWindowTracker::SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
                                           std::shared_ptr<Config> config)
    : map(std::move(keyframe_map)), config(config) {
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        frame_j->preintegration.integrate(frame_j->image->t, frame_i->motion.bg,
                                          frame_i->motion.ba, true, true);
    }
}

SlidingWindowTracker::~SlidingWindowTracker() = default;

void SlidingWindowTracker::mirror_frame(Map *feature_tracking_map,
                                        size_t frame_id) {
    Frame *keyframe = map->get_frame(map->frame_num() - 1);
    Frame *new_frame_i = keyframe;
    if (!keyframe->subframes.empty()) {
        new_frame_i = keyframe->subframes.back().get();
    }

    size_t frame_index_i =
        feature_tracking_map->frame_index_by_id(new_frame_i->id());
    size_t frame_index_j = feature_tracking_map->frame_index_by_id(frame_id);

    if (frame_index_i == nil() || frame_index_j == nil())
        return;

    Frame *old_frame_i = feature_tracking_map->get_frame(frame_index_i);
    Frame *old_frame_j = feature_tracking_map->get_frame(frame_index_j);

    std::unique_ptr<Frame> curr_frame = std::move(old_frame_j->clone());
    std::vector<ImuData> &new_data = curr_frame->preintegration.data;
    for (size_t index = frame_index_j - 1; index > frame_index_i; --index) {
        std::vector<ImuData> old_data =
            feature_tracking_map->get_frame(index)->preintegration.data;
        new_data.insert(new_data.begin(), old_data.begin(), old_data.end());
    }

    map->attach_frame(curr_frame->clone());
    Frame *new_frame_j = map->get_frame(map->frame_num() - 1);

    for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
        if (Track *track = old_frame_i->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(old_frame_j);
                kj != nil()) {
                Track *new_track = new_frame_i->get_track(ki, map.get());
                new_track->add_keypoint(new_frame_j, kj);
                track->tag(TT_TRASH) =
                    new_track->tag(TT_TRASH) && !new_track->tag(TT_STATIC);
            }
        }
    }

    map->prune_tracks([](const Track *track) {
        return track->tag(TT_TRASH) && !track->tag(TT_STATIC);
    });

    new_frame_j->preintegration.integrate(new_frame_j->image->t,
                                          new_frame_i->motion.bg,
                                          new_frame_i->motion.ba, true, true);
    new_frame_j->preintegration.predict(new_frame_i, new_frame_j);
}

bool SlidingWindowTracker::track() {

    if (config->parsac_flag()) {
        if (judge_track_status()) {
            update_track_status();
        }
    }

    localize_newframe();

    if (manage_keyframe()) {
        track_landmark();
        refine_window();
        slide_window();
    } else {
        refine_subwindow();
    }

    inspect_debug(sliding_window_landmarks, landmarks) {
        std::vector<Landmark> points;
        points.reserve(map->track_num());
        for (size_t i = 0; i < map->track_num(); ++i) {
            if (Track *track = map->get_track(i)) {
                if (track->tag(TT_VALID)) {
                    Landmark point;
                    point.p = track->get_landmark_point();
                    point.triangulated = track->tag(TT_TRIANGULATED);
                    points.push_back(point);
                }
            }
        }
        landmarks = std::move(points);
    }

    inspect_debug(sliding_window_current_bg, bg) {
        bg = std::get<2>(get_latest_state()).bg;
    }

    inspect_debug(sliding_window_current_ba, ba) {
        ba = std::get<2>(get_latest_state()).ba;
    }

    return true;
}

void SlidingWindowTracker::localize_newframe() {
    auto solver = Solver::create();

    Frame *frame_i = map->get_frame(map->frame_num() - 2);
    if (!frame_i->subframes.empty()) {
        frame_i = frame_i->subframes.back().get();
    }
    Frame *frame_j = map->get_frame(map->frame_num() - 1);

    solver->add_frame_states(frame_j);

    solver->put_factor(Solver::create_preintegration_prior_factor(
        frame_i, frame_j, frame_j->preintegration));

    for (size_t k = 0; k < frame_j->keypoint_num(); ++k) {
        if (Track *track = frame_j->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC)) {
                solver->put_factor(
                    Solver::create_reprojection_prior_factor(frame_j, track));
            }
        }
    }

    solver->solve();
}

bool SlidingWindowTracker::manage_keyframe() {
    Frame *keyframe_i = map->get_frame(map->frame_num() - 2);
    Frame *newframe_j = map->get_frame(map->frame_num() - 1);

    if (!keyframe_i->subframes.empty()) {
        if (keyframe_i->subframes.back()->tag(FT_NO_TRANSLATION)) {
            if (newframe_j->tag(FT_NO_TRANSLATION)) {
                // [T]...........<-[R]
                //  +-[R]-[R]-[R]
                // ==>
                // [T]
                //  +-[R-R]-[R]-[R]
            } else {
                // [T]...........<-[T]
                //  +-[R]-[R]-[R]
                // ==>
                // [T]........[R]-[T]
                //  +-[R]-[R]
                keyframe_i->subframes.back()->tag(FT_KEYFRAME) = true;
                map->attach_frame(std::move(keyframe_i->subframes.back()),
                                  map->frame_num() - 1);
                keyframe_i->subframes.pop_back();
                newframe_j->tag(FT_KEYFRAME) = true;
                return true;
            }
        } else {
            if (newframe_j->tag(FT_NO_TRANSLATION)) {
                // [T]...........<-[R]
                //  +-[T]-[T]-[T]
                // ==>
                // [T]........[T]
                //  +-[T]-[T]  +-[R]
                std::unique_ptr<Frame> frame_lifted =
                    std::move(keyframe_i->subframes.back());
                keyframe_i->subframes.pop_back();
                frame_lifted->tag(FT_KEYFRAME) = true;
                frame_lifted->subframes.emplace_back(
                    map->detach_frame(map->frame_num() - 1));
                map->attach_frame(std::move(frame_lifted));
                return true;
            } else {
                if (keyframe_i->subframes.size() >=
                    config->sliding_window_subframe_size()) {
                    // [T]...........<-[T]
                    //  +-[T]-[T]-[T]
                    // ==>
                    // [T]............[T]
                    //  +-[T]-[T]-[T]
                    newframe_j->tag(FT_KEYFRAME) = true;
                    return true;
                }
            }
        }
    }
    size_t mapped_landmark_count = 0;
    for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
        if (Track *track = newframe_j->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC)) {
                mapped_landmark_count++;
            }
        }
    }

    bool is_keyframe = mapped_landmark_count <
                       config->sliding_window_force_keyframe_landmarks();

#if defined(XRSLAM_IOS)
    is_keyframe = is_keyframe || !newframe_j->tag(FT_NO_TRANSLATION);
#endif

    if (is_keyframe) {
        newframe_j->tag(FT_KEYFRAME) = true;
        return true;
    } else {
        keyframe_i->subframes.emplace_back(
            map->detach_frame(map->frame_num() - 1));
        return false;
    }
}

void SlidingWindowTracker::track_landmark() {
    Frame *newframe_j = map->get_frame(map->frame_num() - 1);

    for (size_t k = 0; k < newframe_j->keypoint_num(); ++k) {
        if (Track *track = newframe_j->get_track(k)) {
            if (!track->tag(TT_TRIANGULATED)) {
                if (auto p = track->triangulate()) {
                    track->set_landmark_point(p.value());
                    track->tag(TT_TRIANGULATED) = true;
                    track->tag(TT_VALID) = true;
                    track->tag(TT_STATIC) = true;
                } else {
                    // outlier
                    track->landmark.inv_depth = -1.0;
                    track->tag(TT_TRIANGULATED) = false;
                    track->tag(TT_VALID) = false;
                }
            }
        }
    }
}

void SlidingWindowTracker::refine_window() {
    Frame *keyframe_i = map->get_frame(map->frame_num() - 2);
    Frame *keyframe_j = map->get_frame(map->frame_num() - 1);

    auto solver = Solver::create();
    if (!map->marginalization_factor) {
        map->marginalization_factor =
            Solver::create_marginalization_factor(map.get());
    }
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        solver->add_frame_states(frame);
    }
    std::unordered_set<Track *> visited_tracks;
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (visited_tracks.count(track) > 0)
                continue;
            visited_tracks.insert(track);
            if (!track->tag(TT_VALID))
                continue;
            if (!track->tag(TT_STATIC))
                continue;
            if (!track->first_frame()->tag(FT_KEYFRAME))
                continue;
            solver->add_track_states(track);
        }
    }

    solver->add_factor(map->marginalization_factor.get());

    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->all_tagged(TT_VALID, TT_TRIANGULATED, TT_STATIC))
                continue;
            if (!track->first_frame()->tag(FT_KEYFRAME))
                continue;
            if (frame == track->first_frame())
                continue;
            solver->add_factor(frame->reprojection_error_factors[j].get());
        }
    }

    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);

        frame_j->keyframe_preintegration = frame_j->preintegration;
        if (!frame_i->subframes.empty()) {
            std::vector<ImuData> imu_data;
            for (size_t k = 0; k < frame_i->subframes.size(); ++k) {
                auto &sub_imu_data = frame_i->subframes[k]->preintegration.data;
                imu_data.insert(imu_data.end(), sub_imu_data.begin(),
                                sub_imu_data.end());
            }
            frame_j->keyframe_preintegration.data.insert(
                frame_j->keyframe_preintegration.data.begin(), imu_data.begin(),
                imu_data.end());
        }

        if (frame_j->keyframe_preintegration.integrate(
                frame_j->image->t, frame_i->motion.bg, frame_i->motion.ba, true,
                true)) {
            solver->put_factor(Solver::create_preintegration_error_factor(
                frame_i, frame_j, frame_j->keyframe_preintegration));
        }
    }

    solver->solve();

    for (size_t k = 0; k < map->track_num(); ++k) {
        Track *track = map->get_track(k);
        if (track->tag(TT_TRIANGULATED)) {
            bool is_valid = true;
            auto x = track->get_landmark_point();
            double rpe = 0.0;
            double rpe_count = 0.0;
            for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
                if (!frame->tag(FT_KEYFRAME))
                    continue;
                PoseState pose = frame->get_pose(frame->camera);
                vector<3> y = pose.q.conjugate() * (x - pose.p);
                if (y.z() <= 1.0e-3 || y.z() > 50) { // todo
                    is_valid = false;
                    break;
                }
                rpe += (apply_k(y, frame->K) -
                        apply_k(frame->get_keypoint(keypoint_index), frame->K))
                           .norm();
                rpe_count += 1.0;
            }
            is_valid = is_valid && (rpe / std::max(rpe_count, 1.0) < 3.0);
            track->tag(TT_VALID) = is_valid;
        } else {
            track->landmark.inv_depth = -1.0;
        }
    }

    for (size_t k = 0; k < map->track_num(); ++k) {
        Track *track = map->get_track(k);
        if (!track->tag(TT_VALID))
            track->tag(TT_TRASH) = true;
    }
}

void SlidingWindowTracker::slide_window() {
    while (map->frame_num() > config->sliding_window_size()) {
        Frame *frame = map->get_frame(0);
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            map->untrack_frame(frame->subframes[i].get());
        }
        map->marginalize_frame(0);
    }
}

void SlidingWindowTracker::refine_subwindow() {
    Frame *frame = map->get_frame(map->frame_num() - 1);
    if (frame->subframes.empty())
        return;
    if (frame->subframes[0]->tag(FT_NO_TRANSLATION)) {
        if (frame->subframes.size() >= 9) {
            for (size_t i = frame->subframes.size() / 3; i > 0; --i) {
                Frame *tgt_frame = frame->subframes[i * 3 - 1].get();
                std::vector<ImuData> imu_data;
                for (size_t j = i * 3 - 1; j > (i - 1) * 3; --j) {
                    Frame *src_frame = frame->subframes[j - 1].get();
                    imu_data.insert(imu_data.begin(),
                                    src_frame->preintegration.data.begin(),
                                    src_frame->preintegration.data.end());
                    map->untrack_frame(src_frame);
                    frame->subframes.erase(frame->subframes.begin() + (j - 1));
                }
                tgt_frame->preintegration.data.insert(
                    tgt_frame->preintegration.data.begin(), imu_data.begin(),
                    imu_data.end());
            }
        }

        auto solver = Solver::create();
        frame->tag(FT_FIX_POSE) = true;
        frame->tag(FT_FIX_MOTION) = true;

        solver->add_frame_states(frame);
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            Frame *subframe = frame->subframes[i].get();
            solver->add_frame_states(subframe);
            Frame *prev_frame =
                (i == 0 ? frame : frame->subframes[i - 1].get());
            subframe->preintegration.integrate(
                subframe->image->t, prev_frame->motion.bg,
                prev_frame->motion.ba, true, true);
            solver->put_factor(Solver::create_preintegration_error_factor(
                prev_frame, subframe, subframe->preintegration));
        }

        Frame *last_subframe = frame->subframes.back().get();
        for (size_t k = 0; k < last_subframe->keypoint_num(); ++k) {
            if (Track *track = last_subframe->get_track(k)) {
                if (track->tag(TT_VALID)) {
                    if (track->tag(TT_TRIANGULATED)) {
                        if (track->tag(TT_STATIC))
                            solver->put_factor(
                                Solver::create_reprojection_prior_factor(
                                    last_subframe, track));
                    } else {
                        solver->put_factor(Solver::create_rotation_prior_factor(
                            last_subframe, track));
                    }
                }
            }
        }

        solver->solve();
        frame->tag(FT_FIX_POSE) = false;
        frame->tag(FT_FIX_MOTION) = false;
    } else {
        auto solver = Solver::create();
        frame->tag(FT_FIX_POSE) = true;
        frame->tag(FT_FIX_MOTION) = true;
        solver->add_frame_states(frame);
        for (size_t i = 0; i < frame->subframes.size(); ++i) {
            Frame *subframe = frame->subframes[i].get();
            solver->add_frame_states(subframe);
            Frame *prev_frame =
                (i == 0 ? frame : frame->subframes[i - 1].get());
            subframe->preintegration.integrate(
                subframe->image->t, prev_frame->motion.bg,
                prev_frame->motion.ba, true, true);
            solver->put_factor(Solver::create_preintegration_error_factor(
                prev_frame, subframe, subframe->preintegration));
            for (size_t k = 0; k < subframe->keypoint_num(); ++k) {
                if (Track *track = subframe->get_track(k)) {
                    if (track->all_tagged(TT_VALID, TT_TRIANGULATED,
                                          TT_STATIC)) {
                        if (track->first_frame()->tag(FT_KEYFRAME)) {
                            solver->put_factor(
                                Solver::create_reprojection_prior_factor(
                                    subframe, track));
                        } else if (track->first_frame()->id() > frame->id()) {
                            solver->add_factor(
                                frame->reprojection_error_factors[k].get());
                        }
                    }
                }
            }
        }
        solver->solve();
        frame->tag(FT_FIX_POSE) = false;
        frame->tag(FT_FIX_MOTION) = false;
    }
} // namespace xrslam

std::tuple<double, PoseState, MotionState>
SlidingWindowTracker::get_latest_state() const {
    const Frame *frame = map->get_frame(map->frame_num() - 1);
    if (!frame->subframes.empty()) {
        frame = frame->subframes.back().get();
    }
    return {frame->image->t, frame->pose, frame->motion};
}

matrix<3> compute_essential_matrix(matrix<3> &R, vector<3> &t) {
    matrix<3> t_ = matrix<3>::Zero();

    t_(0, 1) = -t(2);
    t_(0, 2) = t(1);
    t_(1, 0) = t(2);
    t_(1, 2) = -t(0);
    t_(2, 0) = -t(1);
    t_(2, 1) = t(0);

    matrix<3> E = t_ * R;
    return E;
}

double compute_epipolar_dist(matrix<3> F, vector<2> &pt1, vector<2> &pt2) {
    vector<3> l = F * pt1.homogeneous();
    double dist =
        std::abs(pt2.homogeneous().transpose() * l) / l.segment<2>(0).norm();
    return dist;
}

bool SlidingWindowTracker::check_frames_rpe(Track *track, const vector<3> &x) {
    std::vector<matrix<3, 4>> Ps;
    std::vector<vector<3>> ps;

    bool is_valid = true;
    double rpe = 0.0;
    double rpe_count = 0.0;
    for (const auto &[frame, keypoint_index] : track->keypoint_map()) {
        if (!frame->tag(FT_KEYFRAME))
            continue;
        PoseState pose = frame->get_pose(frame->camera);
        vector<3> y = pose.q.conjugate() * (x - pose.p);
        if (y.z() <= 1.0e-3 || y.z() > 50) { // todo
            is_valid = false;
            break;
        }
        rpe += (apply_k(y, frame->K) -
                apply_k(frame->get_keypoint(keypoint_index), frame->K))
                   .norm();
        rpe_count += 1.0;
    }
    is_valid = is_valid && (rpe / std::max(rpe_count, 1.0) < 3.0);

    return is_valid;
}

bool SlidingWindowTracker::filter_parsac_2d2d(
    Frame *frame_i, Frame *frame_j, std::vector<char> &mask,
    std::vector<size_t> &pts_to_index) {

    std::vector<vector<2>> pts1, pts2;

    for (size_t ki = 0; ki < frame_i->keypoint_num(); ++ki) {
        if (Track *track = frame_i->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(frame_j)) {
                if (kj != nil()) {
                    pts1.push_back(frame_i->get_keypoint(ki).hnormalized());
                    pts2.push_back(frame_j->get_keypoint(kj).hnormalized());
                    pts_to_index.push_back(kj);
                }
            }
        }
    }

    if (pts1.size() < 10)
        return false;

    matrix<3> E =
        find_essential_matrix_parsac(pts1, pts2, mask, m_th / frame_i->K(0, 0));

    return true;
}

void SlidingWindowTracker::predict_RT(Frame *frame_i, Frame *frame_j,
                                      matrix<3> &R, vector<3> &t) {

    auto camera = frame_i->camera;
    auto imu = frame_i->imu;

    matrix<4> Pwc = matrix<4>::Identity();
    matrix<4> PwI = matrix<4>::Identity();
    matrix<4> Pwi = matrix<4>::Identity();
    matrix<4> Pwj = matrix<4>::Identity();

    Pwc.block<3, 3>(0, 0) = camera.q_cs.toRotationMatrix();
    Pwc.block<3, 1>(0, 3) = camera.p_cs;
    PwI.block<3, 3>(0, 0) = imu.q_cs.toRotationMatrix();
    PwI.block<3, 1>(0, 3) = imu.p_cs;
    Pwi.block<3, 3>(0, 0) = frame_i->pose.q.toRotationMatrix();
    Pwi.block<3, 1>(0, 3) = frame_i->pose.p;
    Pwj.block<3, 3>(0, 0) = frame_j->pose.q.toRotationMatrix();
    Pwj.block<3, 1>(0, 3) = frame_j->pose.p;

    matrix<4> Pji = Pwj.inverse() * Pwi;

    matrix<4> P = (Pwc.inverse() * PwI * Pji * PwI.inverse() * Pwc);

    R = P.block<3, 3>(0, 0);
    t = P.block(0, 3, 3, 1);
}

bool SlidingWindowTracker::judge_track_status() {

    Frame *curr_frame = map->get_frame(map->frame_num() - 1);
    Frame *keyframe = map->get_frame(map->frame_num() - 2);
    Frame *last_frame = keyframe;
    if (!keyframe->subframes.empty()) {
        last_frame = keyframe->subframes.back().get();
    }

    curr_frame->preintegration.integrate(curr_frame->image->t,
                                         last_frame->motion.bg,
                                         last_frame->motion.ba, true, true);
    curr_frame->preintegration.predict(last_frame, curr_frame);

    m_P2D.clear();
    m_P3D.clear();
    m_lens.clear();
    m_indices_map = std::vector<int>(curr_frame->keypoint_num(), -1);

    for (size_t k = 0; k < curr_frame->keypoint_num(); ++k) {
        if (Track *track = curr_frame->get_track(k)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
                const vector<3> &bearing = curr_frame->get_keypoint(k);
                const vector<3> &landmark = track->get_landmark_point();
                m_P2D.push_back(bearing.hnormalized());
                m_P3D.push_back(landmark);
                m_lens.push_back(std::max(track->m_life, size_t(0)));
                m_indices_map[k] = m_P3D.size() - 1;
            }
        }
    }

    if (m_P2D.size() < 20)
        return false;

    const PoseState &pose = curr_frame->get_pose(curr_frame->camera);

    std::vector<char> mask;
    matrix<3> Rcw = pose.q.inverse().toRotationMatrix();
    vector<3> tcw = pose.q.inverse() * pose.p * (-1.0);
    matrix<4> T_IMU =
        find_pnp_matrix_parsac_imu(m_P3D, m_P2D, m_lens, Rcw, tcw, 0.20, 1.0,
                                   mask, 1.0 / curr_frame->K(0, 0));

    matrix<3> R;
    vector<3> t;
    predict_RT(keyframe, curr_frame, R, t);

    // check rpe
    {
        std::vector<vector<2>> P2D_inliers, P2D_outliers;
        std::vector<vector<3>> P3D_inliers, P3D_outliers;

        for (int i = 0; i < m_P2D.size(); ++i) {
            if (mask[i]) {
                P2D_inliers.push_back(m_P2D[i]);
                P3D_inliers.push_back(m_P3D[i]);
            } else {
                P2D_outliers.push_back(m_P2D[i]);
                P3D_outliers.push_back(m_P3D[i]);
            }
        }

        std::vector<double> inlier_errs, outlier_errs;
        double inlier_errs_sum = 0, outlier_errs_sum = 0;
        for (int i = 0; i < P2D_inliers.size(); i++) {
            vector<3> p = pose.q.conjugate() * (P3D_inliers[i] - pose.p);
            double proj_err =
                (apply_k(p, curr_frame->K) -
                 apply_k(P2D_inliers[i].homogeneous(), curr_frame->K))
                    .norm();
            inlier_errs.push_back(proj_err);
            inlier_errs_sum += proj_err;
        }

        for (int i = 0; i < P2D_outliers.size(); i++) {
            vector<3> p = pose.q.conjugate() * (P3D_outliers[i] - pose.p);
            double proj_err =
                (apply_k(p, curr_frame->K) -
                 apply_k(P2D_outliers[i].homogeneous(), curr_frame->K))
                    .norm();
            outlier_errs.push_back(proj_err);
            outlier_errs_sum += proj_err;
        }
    }

    matrix<3> E = compute_essential_matrix(R, t);
    matrix<3> F =
        keyframe->K.transpose().inverse() * E * curr_frame->K.inverse();

    std::vector<vector<2>> inlier_set1, inlier_set2;
    std::vector<vector<2>> outlier_set1, outlier_set2;
    for (size_t i = 0; i < curr_frame->keypoint_num(); ++i) {
        if (m_indices_map[i] != -1) {
            if (size_t j =
                    curr_frame->get_track(i)->get_keypoint_index(keyframe);
                j != nil()) {
                if (mask[m_indices_map[i]]) {
                    inlier_set1.push_back(
                        apply_k(keyframe->get_keypoint(j), keyframe->K));
                    inlier_set2.push_back(
                        apply_k(curr_frame->get_keypoint(i), curr_frame->K));
                } else {
                    outlier_set1.push_back(
                        apply_k(keyframe->get_keypoint(j), keyframe->K));
                    outlier_set2.push_back(
                        apply_k(curr_frame->get_keypoint(i), curr_frame->K));
                }
            }
        }
    }

    std::vector<double> inliers_dist, outliers_dist;

    for (int i = 0; i < inlier_set1.size(); i++) {
        vector<2> &p1 = inlier_set1[i];
        vector<2> &p2 = inlier_set2[i];
        double err = compute_epipolar_dist(F, p1, p2) +
                     compute_epipolar_dist(F.transpose(), p2, p1);
        inliers_dist.push_back(err);
    }

    for (int i = 0; i < outlier_set1.size(); i++) {
        vector<2> &p1 = outlier_set1[i];
        vector<2> &p2 = outlier_set2[i];
        double err = compute_epipolar_dist(F, p1, p2) +
                     compute_epipolar_dist(F.transpose(), p2, p1);
        outliers_dist.push_back(err);
    }

    size_t min_num = 20;
    if (inliers_dist.size() < min_num || outliers_dist.size() < min_num)
        return false;

    std::sort(inliers_dist.begin(), inliers_dist.end());
    std::sort(outliers_dist.begin(), outliers_dist.end());

    double th1 = inliers_dist[size_t(inliers_dist.size() * 0.5)];
    double th2 = outliers_dist[size_t(outliers_dist.size() * 0.5)];

    if (th2 < th1 * 2) // mean there is ambiguity
        return false;

    m_th = (th1 + th2) / 2;

    for (size_t k = 0; k < curr_frame->keypoint_num(); ++k) {
        if (Track *track = curr_frame->get_track(k)) {
            // track->tag(TT_STATIC) = true;
            if (m_indices_map[k] != -1) {
                if (mask[m_indices_map[k]]) {
                    curr_frame->get_track(k)->tag(TT_OUTLIER) = false;
                    curr_frame->get_track(k)->tag(TT_STATIC) = true;
                } else {
                    curr_frame->get_track(k)->tag(TT_OUTLIER) = true;
                    curr_frame->get_track(k)->tag(TT_STATIC) = false;
                }
            }
        }
    }

    return true;
}

void SlidingWindowTracker::update_track_status() {

    Frame *curr_frame = map->get_frame(map->frame_num() - 1);
    size_t frame_id = feature_tracking_map->frame_index_by_id(curr_frame->id());

    if (frame_id == nil())
        return;

    Frame *old_frame = feature_tracking_map->get_frame(frame_id);

    std::vector<size_t> outlier_cnts(curr_frame->keypoint_num(), 0);
    std::vector<size_t> matches_cnts(curr_frame->keypoint_num(), 0);
    size_t start_idx = std::min(
        map->frame_num() - 1,
        std::max(map->frame_num() - 1 - config->parsac_keyframe_check_size(),
                 size_t(0)));
    for (size_t i = start_idx; i < map->frame_num() - 1; i++) {
        std::vector<char> mask;
        std::vector<size_t> pts_to_index;
        if (filter_parsac_2d2d(map->get_frame(i), curr_frame, mask,
                               pts_to_index)) {
            for (size_t j = 0; j < mask.size(); j++) {
                if (!mask[j]) {
                    outlier_cnts[pts_to_index[j]] += 1;
                }
                matches_cnts[pts_to_index[j]] += 1;
            }
        }
    }

    for (size_t i = 0; i < curr_frame->keypoint_num(); i++) {
        if (Track *curr_track = curr_frame->get_track(i)) {
            if (size_t j = curr_track->get_keypoint_index(old_frame)) {
                if (j != nil()) {
                    Track *old_track = old_frame->get_track(j);
                    size_t outlier_th = map->frame_num() / 2;
                    if (outlier_cnts[i] > outlier_th / 2 &&
                        outlier_cnts[i] > 0.8 * matches_cnts[i]) {
                        curr_track->tag(TT_STATIC) = false;
                    }
                    if (!old_track->tag(TT_STATIC) ||
                        !curr_track->tag(TT_STATIC)) {
                        curr_track->tag(TT_STATIC) = false;
                        old_track->tag(TT_STATIC) = false;
                    }
                }
            }
        }
    }
}

} // namespace xrslam
