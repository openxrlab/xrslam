#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/initializer.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/essential.h>
#include <xrslam/geometry/homography.h>
#include <xrslam/geometry/lie_algebra.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/xrslam.h>
#include <xrslam/optimizer/solver.h>

namespace xrslam {

Initializer::Initializer(std::shared_ptr<Config> config) : config(config) {}

Initializer::~Initializer() = default;

void Initializer::mirror_keyframe_map(Map *feature_tracking_map,
                                      size_t init_frame_id) {
    size_t init_frame_index_last =
        feature_tracking_map->frame_index_by_id(init_frame_id);
    size_t init_frame_index_gap = config->initializer_keyframe_gap();
    size_t init_frame_index_distance =
        init_frame_index_gap * (config->initializer_keyframe_num() - 1);

    init_frame_id = nil();

    if (init_frame_index_last < init_frame_index_distance) {
        map.reset();
        return;
    }

    size_t init_frame_index_first =
        init_frame_index_last - init_frame_index_distance;

    std::vector<size_t> init_keyframe_indices;
    for (size_t i = 0; i < config->initializer_keyframe_num(); ++i) {
        init_keyframe_indices.push_back(init_frame_index_first +
                                        i * init_frame_index_gap);
    }

    map = std::make_unique<Map>();
    for (size_t index : init_keyframe_indices) {
        map->attach_frame(feature_tracking_map->get_frame(index)->clone());
    }

    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *old_frame_i =
            feature_tracking_map->get_frame(init_keyframe_indices[j - 1]);
        Frame *old_frame_j =
            feature_tracking_map->get_frame(init_keyframe_indices[j]);
        Frame *new_frame_i = map->get_frame(j - 1);
        Frame *new_frame_j = map->get_frame(j);
        for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_frame_j);
                    kj != nil()) {
                    new_frame_i->get_track(ki, nullptr)
                        ->add_keypoint(new_frame_j, kj);
                }
            }
        }
        new_frame_j->preintegration.data.clear();
        for (size_t f = init_keyframe_indices[j - 1];
             f < init_keyframe_indices[j]; ++f) {
            Frame *old_frame = feature_tracking_map->get_frame(f + 1);
            std::vector<ImuData> &old_data = old_frame->preintegration.data;
            std::vector<ImuData> &new_data = new_frame_j->preintegration.data;
            new_data.insert(new_data.end(), old_data.begin(), old_data.end());
        }
    }
}

std::unique_ptr<SlidingWindowTracker> Initializer::initialize() {
    if (!map)
        return nullptr;
    if (!init_sfm())
        return nullptr;
    if (!init_imu())
        return nullptr;

    map->get_frame(0)->tag(FT_FIX_POSE) = true;

    auto solver = Solver::create();
    auto tinysolver = TinySolver::create();
    for (size_t i = 0; i < map->frame_num(); ++i) {
        solver->add_frame_states(map->get_frame(i));
        tinysolver->add_frame_states(map->get_frame(i));
    }
    std::unordered_set<Track *> visited_tracks;
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->tag(TT_VALID))
                continue;
            if (visited_tracks.count(track) > 0)
                continue;
            visited_tracks.insert(track);
            solver->add_track_states(track);
        }
    }
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->all_tagged(TT_VALID, TT_TRIANGULATED))
                continue;
            if (frame == track->first_frame())
                continue;
            solver->add_factor(frame->reprojection_error_factors[j].get());
            tinysolver->add_factor(frame->tiny_reprojection_error_factors[j].get());
        }
    }
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        if (frame_j->preintegration.integrate(frame_j->image->t,
                                              frame_i->motion.bg,
                                              frame_i->motion.ba, true, true)) {
            solver->put_factor(Solver::create_preintegration_error_factor(
                frame_i, frame_j, frame_j->preintegration));
            tinysolver->put_factor(TinySolver::create_preintegration_error_factor(
                frame_i, frame_j, frame_j->preintegration));
        }
    }
    solver->solve(true);

    tinysolver->solve(true);

    for (size_t i = 0; i < map->frame_num(); ++i) {
        map->get_frame(i)->tag(FT_KEYFRAME) = true;
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

    std::unique_ptr<SlidingWindowTracker> tracker =
        std::make_unique<SlidingWindowTracker>(std::move(map), config);
    return tracker;
}

bool Initializer::init_sfm() {
    // [1] try initializing using raw_map
    Frame *init_frame_i = map->get_frame(0);
    Frame *init_frame_j = map->get_frame(map->frame_num() - 1);

    double total_parallax = 0;
    int common_track_num = 0;
    std::vector<vector<3>> init_points;
    std::vector<std::pair<size_t, size_t>> init_matches;
    std::vector<char> init_point_status;
    std::vector<vector<2>> frame_i_keypoints;
    std::vector<vector<2>> frame_j_keypoints;
    matrix<3> init_R;
    vector<3> init_T;

    for (size_t ki = 0; ki < init_frame_i->keypoint_num(); ++ki) {
        Track *track = init_frame_i->get_track(ki);
        if (!track)
            continue;
        size_t kj = track->get_keypoint_index(init_frame_j);
        if (kj == nil())
            continue;
        frame_i_keypoints.push_back(
            init_frame_i->get_keypoint(ki).hnormalized());
        frame_j_keypoints.push_back(
            init_frame_j->get_keypoint(kj).hnormalized());
        init_matches.emplace_back(ki, kj);
        total_parallax +=
            (apply_k(init_frame_i->get_keypoint(ki), init_frame_i->K) -
             apply_k(init_frame_j->get_keypoint(kj), init_frame_j->K))
                .norm();
        common_track_num++;
    }

    if (common_track_num < (int)config->initializer_min_matches())
        return false;
    total_parallax /= std::max(common_track_num, 1);
    if (total_parallax < config->initializer_min_parallax())
        return false;

    std::vector<matrix<3>> Rs;
    std::vector<vector<3>> Ts;

    matrix<3> RH1, RH2;
    vector<3> TH1, TH2, nH1, nH2;
    matrix<3> H = find_homography_matrix(frame_i_keypoints, frame_j_keypoints,
                                         0.7 / init_frame_i->K(0, 0), 0.999,
                                         1000, config->random());
    if (!decompose_homography(H, RH1, RH2, TH1, TH2, nH1, nH2)) {
        log_debug("SfM init fail: pure rotation.");
        return false; // is pure rotation
    }
    TH1 = TH1.normalized();
    TH2 = TH2.normalized();
    Rs.insert(Rs.end(), {RH1, RH1, RH2, RH2});
    Ts.insert(Ts.end(), {TH1, -TH1, TH2, -TH2});

    matrix<3> RE1, RE2;
    vector<3> TE;
    matrix<3> E = find_essential_matrix(frame_i_keypoints, frame_j_keypoints,
                                        0.7 / init_frame_i->K(0, 0), 0.999,
                                        1000, config->random());
    decompose_essential(E, RE1, RE2, TE);
    TE = TE.normalized();
    Rs.insert(Rs.end(), {RE1, RE1, RE2, RE2});
    Ts.insert(Ts.end(), {TE, -TE, TE, -TE});

    // [1.1] triangulation
    std::vector<std::vector<vector<3>>> triangulation_points(Rs.size());
    std::vector<std::vector<char>> triangulation_status(Rs.size());
    std::vector<size_t> triangulation_counts(Rs.size());
    std::vector<double> triangulation_scores(Rs.size());

    size_t best_rt_index = 0;
    for (size_t i = 0; i < Rs.size(); ++i) {
        auto &points = triangulation_points[i];
        auto &status = triangulation_status[i];
        auto &count = triangulation_counts[i];
        auto &score = triangulation_scores[i];
        points.resize(frame_i_keypoints.size());
        status.resize(frame_i_keypoints.size());
        count = 0;
        score = 0;
        matrix<3, 4> P1, P2;
        P1.setIdentity();
        P2 << Rs[i], Ts[i];
        for (size_t j = 0; j < frame_i_keypoints.size(); ++j) {
            status[j] = 0;

            vector<4> q =
                triangulate_point(P1, P2, frame_i_keypoints[j].homogeneous(),
                                  frame_j_keypoints[j].homogeneous());
            vector<3> q1 = P1 * q;
            vector<3> q2 = P2 * q;
            if (q1[2] * q[3] > 0 && q2[2] * q[3] > 0) {
                if (q1[2] / q[3] < 100 && q2[2] / q[3] < 100) {
                    points[j] = q.hnormalized();
                    status[j] = 1;
                    count++;
                    score += 0.5 * ((q1.hnormalized() - frame_i_keypoints[j])
                                        .squaredNorm() +
                                    (q2.hnormalized() - frame_j_keypoints[j])
                                        .squaredNorm());
                }
            }
        }

        if (triangulation_counts[i] > config->initializer_min_triangulation() &&
            triangulation_scores[i] < triangulation_scores[best_rt_index]) {
            best_rt_index = i;
        } else if (triangulation_counts[i] >
                   triangulation_counts[best_rt_index]) {
            best_rt_index = i;
        }
    }
    init_R = Rs[best_rt_index];
    init_T = Ts[best_rt_index];
    init_points.swap(triangulation_points[best_rt_index]);
    init_point_status.swap(triangulation_status[best_rt_index]);
    size_t triangulated_num = triangulation_counts[best_rt_index];

    if (triangulated_num < config->initializer_min_triangulation()) {
        log_debug("SfM init fail: triangulation (%zd).", triangulated_num);
        return false;
    }

    // [2] create sfm map

    // [2.1] set init states
    PoseState pose;
    pose.q.setIdentity();
    pose.p.setZero();
    init_frame_i->set_pose(init_frame_i->camera, pose);
    pose.q = init_R.transpose();
    pose.p = -(init_R.transpose() * init_T);
    init_frame_j->set_pose(init_frame_j->camera, pose);

    for (size_t k = 0; k < init_points.size(); ++k) {
        if (init_point_status[k] == 0)
            continue;
        Track *track = init_frame_i->get_track(init_matches[k].first);
        track->set_landmark_point(init_points[k]);
        track->tag(TT_VALID) = true;
        track->tag(TT_TRIANGULATED) = true;
    }

    // [2.2] solve other frames via pnp
    for (size_t j = 1; j + 1 < map->frame_num(); ++j) {
        Frame *frame_i = map->get_frame(j - 1);
        Frame *frame_j = map->get_frame(j);
        frame_j->set_pose(frame_j->camera, frame_i->get_pose(frame_i->camera));
        auto solver = Solver::create();
        solver->add_frame_states(frame_j);
        for (size_t k = 0; k < frame_j->keypoint_num(); ++k) {
            Track *track = frame_j->get_track(k);
            if (!track)
                continue;
            if (!track->has_keypoint(map->get_frame(0)))
                continue;
            if (track->tag(TT_VALID) && track->tag(TT_TRIANGULATED)) {
                solver->put_factor(
                    Solver::create_reprojection_prior_factor(frame_j, track));
            }
        }
        solver->solve();
    }

    // [2.3] triangulate more points
    for (size_t i = 0; i < map->track_num(); ++i) {
        Track *track = map->get_track(i);
        if (track->tag(TT_VALID))
            continue;
        if (auto p = track->triangulate()) {
            track->set_landmark_point(p.value());
            track->tag(TT_VALID) = true;
            track->tag(TT_TRIANGULATED) = true;
        }
    }

    // [3] sfm

    // [3.1] bundle adjustment
    map->get_frame(0)->tag(FT_FIX_POSE) = true;
    auto solver = Solver::create();
    auto tinysolver = TinySolver::create();
    for (size_t i = 0; i < map->frame_num(); ++i) {
        solver->add_frame_states(map->get_frame(i), false);
        tinysolver->add_frame_states(map->get_frame(i), false);
    }
    std::unordered_set<Track *> visited_tracks;
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->tag(TT_VALID))
                continue;
            if (visited_tracks.count(track) > 0)
                continue;
            visited_tracks.insert(track);
            solver->add_track_states(track);
        }
    }
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        for (size_t j = 0; j < frame->keypoint_num(); ++j) {
            Track *track = frame->get_track(j);
            if (!track)
                continue;
            if (!track->all_tagged(TT_VALID, TT_TRIANGULATED))
                continue;
            if (frame == track->first_frame())
                continue;
            solver->add_factor(frame->reprojection_error_factors[j].get());
            tinysolver->add_factor(frame->tiny_reprojection_error_factors[j].get());
        }
    }
    if (!solver->solve(false)) {
        return false;
    }

    // tinysolver->solve(true);

    // [3.2] cleanup invalid points
    map->prune_tracks([](const Track *track) {return !track->tag(TT_VALID) || track->landmark.reprojection_error > 3.0; // TODO: make configurable
    });

    return true;
}

bool Initializer::init_imu() {
    reset_states();
    solve_gyro_bias();
    solve_gravity_scale_velocity();
    if (scale < 0.001 || scale > 1.0)
        return false;
    if (!config->initializer_refine_imu()) {
        return apply_init();
    }
    refine_scale_velocity_via_gravity();
    if (scale < 0.001 || scale > 1.0)
        return false;
    return apply_init();
}

void Initializer::solve_gyro_bias() {
    preintegrate();
    matrix<3> A = matrix<3>::Zero();
    vector<3> b = vector<3>::Zero();

    for (size_t j = 1; j < map->frame_num(); ++j) {
        const size_t i = j - 1;

        const Frame *frame_i = map->get_frame(i);
        const Frame *frame_j = map->get_frame(j);

        const PoseState pose_i = frame_i->get_pose(frame_i->imu);
        const PoseState pose_j = frame_j->get_pose(frame_j->imu);

        const quaternion &dq = frame_j->preintegration.delta.q;
        const matrix<3> &dq_dbg = frame_j->preintegration.jacobian.dq_dbg;
        A += dq_dbg.transpose() * dq_dbg;
        b += dq_dbg.transpose() * logmap((pose_i.q * dq).conjugate() * pose_j.q);
    }

    Eigen::JacobiSVD<matrix<3>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
    bg = svd.solve(b);
}

void Initializer::solve_gravity_scale_velocity() {
    preintegrate();
    int N = (int)map->frame_num();
    matrix<> A;
    vector<> b;
    A.resize((N - 1) * 6, 3 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    A.setZero();
    b.setZero();

    for (size_t j = 1; j < map->frame_num(); ++j) {
        const size_t i = j - 1;

        const Frame *frame_i = map->get_frame(i);
        const Frame *frame_j = map->get_frame(j);
        const PreIntegrator::Delta &delta = frame_j->preintegration.delta;
        const PoseState camera_pose_i = frame_i->get_pose(frame_i->camera);
        const PoseState camera_pose_j = frame_j->get_pose(frame_j->camera);

        A.block<3, 3>(i * 6, 0) = -0.5 * delta.t * delta.t * matrix<3>::Identity();
        A.block<3, 1>(i * 6, 3) = camera_pose_j.p - camera_pose_i.p;
        A.block<3, 3>(i * 6, 4 + i * 3) = -delta.t * matrix<3>::Identity();
        b.segment<3>(i * 6) = frame_i->pose.q * delta.p +
                              (frame_j->pose.q * frame_j->camera.p_cs -
                               frame_i->pose.q * frame_i->camera.p_cs);

        A.block<3, 3>(i * 6 + 3, 0) = -delta.t * matrix<3>::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + i * 3) = -matrix<3>::Identity();
        A.block<3, 3>(i * 6 + 3, 4 + j * 3) = matrix<3>::Identity();
        b.segment<3>(i * 6 + 3) = frame_i->pose.q * delta.v;
    }

    vector<> x = A.fullPivHouseholderQr().solve(b);
    gravity = x.segment<3>(0).normalized() * XRSLAM_GRAVITY_NOMINAL;
    scale = x(3);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        velocities[i] = x.segment<3>(4 + i * 3);
    }
}

void Initializer::refine_scale_velocity_via_gravity() {
    static const double damp = 0.1;
    preintegrate();
    int N = (int)map->frame_num();
    matrix<> A;
    vector<> b;
    vector<> x;
    A.resize((N - 1) * 6, 2 + 1 + 3 * N);
    b.resize((N - 1) * 6);
    x.resize(2 + 1 + 3 * N);

    for (size_t iter = 0; iter < 1; ++iter) {
        A.setZero();
        b.setZero();
        matrix<3, 2> Tg = s2_tangential_basis(gravity);

        for (size_t j = 1; j < map->frame_num(); ++j) {
            const size_t i = j - 1;

            const Frame *frame_i = map->get_frame(i);
            const Frame *frame_j = map->get_frame(j);
            const PreIntegrator::Delta &delta = frame_j->preintegration.delta;
            const PoseState camera_pose_i = frame_i->get_pose(frame_i->camera);
            const PoseState camera_pose_j = frame_j->get_pose(frame_j->camera);

            A.block<3, 2>(i * 6, 0) = -0.5 * delta.t * delta.t * Tg;
            A.block<3, 1>(i * 6, 2) = camera_pose_j.p - camera_pose_i.p;
            A.block<3, 3>(i * 6, 3 + i * 3) = -delta.t * matrix<3>::Identity();
            b.segment<3>(i * 6) = 0.5 * delta.t * delta.t * gravity +
                                  frame_i->pose.q * delta.p +
                                  (frame_j->pose.q * frame_j->camera.p_cs -
                                   frame_i->pose.q * frame_i->camera.p_cs);

            A.block<3, 2>(i * 6 + 3, 0) = -delta.t * Tg;
            A.block<3, 3>(i * 6 + 3, 3 + i * 3) = -matrix<3>::Identity();
            A.block<3, 3>(i * 6 + 3, 3 + j * 3) = matrix<3>::Identity();
            b.segment<3>(i * 6 + 3) =
                delta.t * gravity + frame_i->pose.q * delta.v;
        }

        x = A.fullPivHouseholderQr().solve(b);
        vector<2> dg = x.segment<2>(0);
        gravity =
            (gravity + damp * Tg * dg).normalized() * XRSLAM_GRAVITY_NOMINAL;
    }

    scale = x(2);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        velocities[i] = x.segment<3>(3 + i * 3);
    }
}

void Initializer::reset_states() {
    bg.setZero();
    ba.setZero();
    gravity.setZero();
    scale = 1;
    velocities.resize(map->frame_num(), vector<3>::Zero());
}

void Initializer::preintegrate() {
    for (size_t j = 1; j < map->frame_num(); ++j) {
        Frame *frame_j = map->get_frame(j);
        frame_j->preintegration.integrate(frame_j->image->t, bg, ba, true,
                                          false);
    }
}

bool Initializer::apply_init(bool apply_ba, bool apply_velocity) {
    static const vector<3> gravity_nominal{0, 0, -XRSLAM_GRAVITY_NOMINAL};

    quaternion q = quaternion::FromTwoVectors(gravity, gravity_nominal);
    for (size_t i = 0; i < map->frame_num(); ++i) {
        Frame *frame = map->get_frame(i);
        PoseState imu_pose = frame->get_pose(frame->imu);
        imu_pose.q = q * imu_pose.q;
        imu_pose.p = scale * (q * imu_pose.p);
        frame->set_pose(frame->imu, imu_pose);
        if (apply_velocity) {
            frame->motion.v = q * velocities[i];
        } else {
            frame->motion.v.setZero();
        }
        frame->motion.bg = bg;
        if (apply_ba) {
            frame->motion.ba = ba;
        } else {
            frame->motion.ba.setZero();
        }
    }
    size_t final_point_num = 0;
    for (size_t i = 0; i < map->track_num(); ++i) {
        Track *track = map->get_track(i);
        if (auto p = track->triangulate()) {
            track->set_landmark_point(p.value());
            track->tag(TT_VALID) = true;
            track->tag(TT_TRIANGULATED) = true;
            final_point_num++;
        } else {
            track->tag(TT_VALID) = false;
        }
    }

    return final_point_num >= config->initializer_min_landmarks();
}

} // namespace xrslam
