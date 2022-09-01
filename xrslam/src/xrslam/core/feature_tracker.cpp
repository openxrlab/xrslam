#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/estimation/solver.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/inspection.h>
#include <xrslam/localizer/localizer.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>
#include <xrslam/utility/unique_timer.h>
namespace xrslam {

FeatureTracker::FeatureTracker(XRSLAM::Detail *detail,
                               std::shared_ptr<Config> config)
    : detail(detail), config(config) {
    map = std::make_unique<Map>();
    keymap = std::make_unique<Map>();
}

FeatureTracker::~FeatureTracker() = default;

void FeatureTracker::work(std::unique_lock<std::mutex> &l) {
    auto ft_timer = make_timer([](double t) {
        inspect(feature_tracker_time, time) {
            static double avg_time = 0;
            static double avg_count = 0;
            avg_time = (avg_time * avg_count + t) / (avg_count + 1);
            avg_count += 1.0;
            time = avg_time;
        }
    });

    std::unique_ptr<Frame> frame = std::move(frames.front());
    frames.pop_front();
    l.unlock();

    frame->image->preprocess();

    auto [latest_optimized_time, latest_optimized_frame_id,
          latest_optimized_pose, latest_optimized_motion] =
        detail->frontend->get_latest_state();
    bool is_initialized = latest_optimized_frame_id != nil();
    bool slidind_window_frame_tag =
        !is_initialized ||
        frame->id() % config->sliding_window_tracker_frequent() == 0;
    synchronized(map) {
        if (map->frame_num() > 0) {
            if (is_initialized) {
                size_t latest_optimized_frame_index =
                    map->frame_index_by_id(latest_optimized_frame_id);
                if (latest_optimized_frame_index != nil()) {
                    Frame *latest_optimized_frame =
                        map->get_frame(latest_optimized_frame_index);
                    latest_optimized_frame->pose = latest_optimized_pose;
                    latest_optimized_frame->motion = latest_optimized_motion;
                    for (size_t j = latest_optimized_frame_index + 1;
                         j < map->frame_num(); ++j) {
                        Frame *frame_i = map->get_frame(j - 1);
                        Frame *frame_j = map->get_frame(j);
                        frame_j->preintegration.integrate(
                            frame_j->image->t, frame_i->motion.bg,
                            frame_i->motion.ba, false, false);
                        frame_j->preintegration.predict(frame_i, frame_j);
                    }
                } else {
                    // TODO: unfortunately the frame has slided out, which means
                    // we are lost...
                    log_warning("SWT cannot catch up.");
                    std::unique_lock lk(latest_pose_mutex);
                    latest_state.reset();
                }
            }
            Frame *last_frame = map->get_frame(map->frame_num() - 1);
            if (!last_frame->preintegration.data.empty()) {
                if (frame->preintegration.data.empty() ||
                    (frame->preintegration.data.front().t -
                         last_frame->image->t >
                     1.0e-5)) {
                    ImuData imu = last_frame->preintegration.data.back();
                    imu.t = last_frame->image->t;
                    frame->preintegration.data.insert(
                        frame->preintegration.data.begin(), imu);
                }
            }
            frame->preintegration.integrate(
                frame->image->t, last_frame->motion.bg, last_frame->motion.ba,
                false, false);
            last_frame->track_keypoints(frame.get(), config.get());
            if (is_initialized) {
                frame->preintegration.predict(last_frame, frame.get());
#if defined(XRSLAM_IOS)
                synchronized(keymap) {
                    attach_latest_frame(frame.get());
                    solve_pnp();
                    Frame *latest_frame =
                        keymap->get_frame(keymap->frame_num() - 1);
                    std::unique_lock lk(latest_pose_mutex);
                    latest_state = {latest_frame->image->t, latest_frame->pose,
                                    latest_frame->motion};
                    lk.unlock();
                    keymap->erase_frame(keymap->frame_num() - 1);
                    if (config->visual_localization_enable() &&
                        detail->frontend->global_localization_state()) {
                        detail->frontend->localizer->query_localization(
                            latest_frame->image, latest_frame->pose);
                        // detail->frontend->localizer->send_pose_message(frame->image->t);
                    }
                }
#else
                std::unique_lock lk(latest_pose_mutex);
                latest_state = {frame->image->t, frame->pose, frame->motion};
                if (config->visual_localization_enable() &&
                    detail->frontend->global_localization_state()) {
                    detail->frontend->localizer->query_localization(
                        frame->image, frame->pose);
                    // detail->frontend->localizer->send_pose_message(frame->image->t);
                }
                lk.unlock();
#endif
            }
            last_frame->image->release_image_buffer();
        }

        if (slidind_window_frame_tag)
            frame->detect_keypoints(config.get());
        map->attach_frame(std::move(frame));

        while (map->frame_num() >
                   (is_initialized
                        ? config->feature_tracker_max_frames()
                        : config->feature_tracker_max_init_frames()) &&
               map->get_frame(0)->id() < latest_optimized_frame_id) {
            map->erase_frame(0);
        }

        inspect_debug(feature_tracker_painter, p) {
            if (p.has_value()) {
                auto painter = std::any_cast<InspectPainter *>(p);
                auto frame = map->get_frame(map->frame_num() - 1);
                painter->set_image(frame->image.get());
                for (size_t i = 0; i < frame->keypoint_num(); ++i) {
                    if (Track *track = frame->get_track(i)) {
                        color3b c = {0, 255, 0};
                        painter->point(apply_k(frame->get_keypoint(i), frame->K)
                                           .cast<int>(),
                                       c, 5);
                    } else {
                        color3b c = {255, 0, 255};
                        painter->point(apply_k(frame->get_keypoint(i), frame->K)
                                           .cast<int>(),
                                       c, 5, 1);
                    }
                }
            }
        }
    }
    if (slidind_window_frame_tag)
        detail->frontend->issue_frame(map->get_frame(map->frame_num() - 1));
}

void FeatureTracker::track_frame(std::unique_ptr<Frame> frame) {
    auto l = lock();
    frames.emplace_back(std::move(frame));
    resume(l);
}

std::optional<std::tuple<double, PoseState, MotionState>>
FeatureTracker::get_latest_state() const {
    std::unique_lock lk(latest_pose_mutex);
    return latest_state;
}

void FeatureTracker::synchronize_keymap(Map *sliding_window_tracker_map) {

    // clean keymap
    while (keymap->frame_num()) {
        keymap->erase_frame(0);
    }

    // mirror the latest SWT map to keymap
    mirror_map(sliding_window_tracker_map);

    // add last frame (include subframe) in swt map to keymap for track
    // associating
    mirror_lastframe(sliding_window_tracker_map);
}

void FeatureTracker::mirror_map(Map *sliding_window_tracker_map) {

    for (size_t index = 0; index < sliding_window_tracker_map->frame_num();
         ++index) {
        keymap->attach_frame(
            sliding_window_tracker_map->get_frame(index)->clone());
    }

    for (size_t j = 1; j < keymap->frame_num(); ++j) {
        Frame *old_frame_i = sliding_window_tracker_map->get_frame(j - 1);
        Frame *old_frame_j = sliding_window_tracker_map->get_frame(j);
        Frame *new_frame_i = keymap->get_frame(j - 1);
        Frame *new_frame_j = keymap->get_frame(j);
        for (size_t ki = 0; ki < old_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_frame_j);
                    kj != nil()) {
                    Track *new_track = new_frame_i->get_track(ki, keymap.get());
                    new_track->add_keypoint(new_frame_j, kj);
                    new_track->landmark = track->landmark;
                    new_track->tag(TT_VALID) = track->tag(TT_VALID);
                    new_track->tag(TT_TRIANGULATED) =
                        track->tag(TT_TRIANGULATED);
                    new_track->tag(TT_FIX_INVD) = true;
                }
            }
        }
    }

    for (size_t index = 0; index < keymap->frame_num(); ++index) {
        Frame *keyframe = keymap->get_frame(index);
        keyframe->tag(FT_KEYFRAME) = true;
        keyframe->tag(FT_FIX_POSE) = true;
        keyframe->tag(FT_FIX_MOTION) = true;
    }
}

void FeatureTracker::mirror_lastframe(Map *sliding_window_tracker_map) {

    Frame *last_keyframe_i = keymap->get_frame(keymap->frame_num() - 1);
    Frame *last_keyframe_j = sliding_window_tracker_map->get_frame(
        sliding_window_tracker_map->frame_num() - 1);

    if (last_keyframe_j->subframes.empty()) // no subframes means this keyframe
                                            // has been existed in FT map
        return;

    Frame *last_subframe = last_keyframe_j->subframes.back().get();

    keymap->attach_frame(last_subframe->clone());

    Frame *new_keyframe = keymap->get_frame(keymap->frame_num() - 1);

    for (size_t ki = 0; ki < last_keyframe_j->keypoint_num(); ++ki) {
        if (Track *track = last_keyframe_j->get_track(ki)) {
            if (size_t kj = track->get_keypoint_index(last_subframe);
                kj != nil()) {
                last_keyframe_i->get_track(ki, keymap.get())
                    ->add_keypoint(new_keyframe, kj);
            }
        }
    }

    new_keyframe->tag(FT_KEYFRAME) = false;
    new_keyframe->tag(FT_FIX_POSE) = false;
    new_keyframe->tag(FT_FIX_MOTION) = false;
}

void FeatureTracker::attach_latest_frame(Frame *frame) {

    Frame *new_last_frame_i = keymap->get_frame(keymap->frame_num() - 1);
    size_t last_frame_index = map->frame_index_by_id(new_last_frame_i->id());

    size_t frame_index_i = map->frame_index_by_id(
        keymap->get_frame(keymap->frame_num() - 1)->id());
    size_t frame_index_j = map->frame_num() - 1;

    keymap->attach_frame(frame->clone());
    Frame *new_last_frame_j = keymap->get_frame(keymap->frame_num() - 1);

    if (last_frame_index != nil()) {
        Frame *old_last_frame_i = map->get_frame(last_frame_index);
        Frame *old_last_frame_j = frame;
        for (size_t ki = 0; ki < old_last_frame_i->keypoint_num(); ++ki) {
            if (Track *track = old_last_frame_i->get_track(ki)) {
                if (size_t kj = track->get_keypoint_index(old_last_frame_j);
                    kj != nil()) {
                    Track *track =
                        new_last_frame_i->get_track(ki, keymap.get());
                    track->add_keypoint(new_last_frame_j, kj);
                }
            }
        }
        new_last_frame_j->tag(FT_KEYFRAME) = false;
        new_last_frame_j->tag(FT_FIX_POSE) = false;
    } else {
        std::cout << "error: cannot find last frame id in FT map" << std::endl;
    }
}

void FeatureTracker::solve_pnp() {

    Frame *latest_frame = keymap->get_frame(keymap->frame_num() - 1);

    auto solver = Solver::create();

    solver->add_frame_states(latest_frame);

    for (size_t j = 0; j < latest_frame->keypoint_num(); ++j) {
        if (Track *track = latest_frame->get_track(j)) {
            if (track->all_tagged(TT_VALID, TT_TRIANGULATED)) {
                solver->put_factor(Solver::create_reprojection_prior_factor(
                    latest_frame, track));
            }
        }
    }
}

} // namespace xrslam
