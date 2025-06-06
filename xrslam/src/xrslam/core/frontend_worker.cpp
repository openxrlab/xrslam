#include <iostream>
#include <xrslam/common.h>
#include <xrslam/core/detail.h>
#include <xrslam/core/feature_tracker.h>
#include <xrslam/core/frontend_worker.h>
#include <xrslam/core/initializer.h>
#include <xrslam/core/sliding_window_tracker.h>
#include <xrslam/geometry/stereo.h>
#include <xrslam/localizer/localizer.h>
#include <xrslam/map/frame.h>
#include <xrslam/map/map.h>
#include <xrslam/map/track.h>

namespace xrslam {

FrontendWorker::FrontendWorker(XRSLAM::Detail *detail,
                               std::shared_ptr<Config> config)
    : detail(detail), config(config) {
    initializer = std::make_unique<Initializer>(config);

    latest_state = {{}, nil(), {}, {}};
}

FrontendWorker::~FrontendWorker() = default;

bool FrontendWorker::empty() const { return pending_frame_ids.empty(); }

void FrontendWorker::work(std::unique_lock<std::mutex> &l) {
    if (initializer) {
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.clear();
        l.unlock();
        synchronized(detail->feature_tracker->map) {
            initializer->mirror_keyframe_map(detail->feature_tracker->map.get(),
                                             pending_frame_id);
        }
        if ((sliding_window_tracker = initializer->initialize())) {
#if defined(XRSLAM_IOS)
            synchronized(detail->feature_tracker->keymap) {
                detail->feature_tracker->synchronize_keymap(
                    sliding_window_tracker->map.get());
            }
#endif
            if (config->visual_localization_enable() &&
                global_localization_state()) {
                localizer = std::make_unique<Localizer>(config);
                sliding_window_tracker->map->create_virtual_object_manager(localizer.get());
            } else {
                sliding_window_tracker->map->create_virtual_object_manager();
            }
            sliding_window_tracker->feature_tracking_map = detail->feature_tracker->map;
            sliding_window_tracker->set_detail(detail);
            std::unique_lock lk(latest_state_mutex);
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {t, pending_frame_id, pose, motion};
            lk.unlock();
            initializer.reset();
        }
    } else if (sliding_window_tracker) {
        size_t pending_frame_id = pending_frame_ids.front();
        pending_frame_ids.pop_front();
        l.unlock();
        synchronized(detail->feature_tracker->map) {
            sliding_window_tracker->mirror_frame(
                detail->feature_tracker->map.get(), pending_frame_id);
        }
        if (sliding_window_tracker->track()) {
#if defined(XRSLAM_IOS)
            synchronized(detail->feature_tracker->keymap) {
                detail->feature_tracker->synchronize_keymap(
                    sliding_window_tracker->map.get());
            }
#endif
            std::unique_lock lk(latest_state_mutex);
            auto [t, pose, motion] = sliding_window_tracker->get_latest_state();
            latest_state = {t, pending_frame_id, pose, motion};
            lk.unlock();
        } else {
            std::unique_lock lk(latest_state_mutex);
            latest_state = {{}, nil(), {}, {}};
            lk.unlock();
            initializer = std::make_unique<Initializer>(config);
            sliding_window_tracker.reset();
        }
    }
}

void FrontendWorker::issue_frame(Frame *frame) {
    auto l = lock();
    pending_frame_ids.push_back(frame->id());
    resume(l);
}

std::tuple<double, size_t, PoseState, MotionState>
FrontendWorker::get_latest_state() const {
    std::unique_lock lk(latest_state_mutex);
    return latest_state;
}

size_t FrontendWorker::create_virtual_object() {
    auto l = lock();
    if (sliding_window_tracker) {
        return sliding_window_tracker->map->create_virtual_object();
    } else {
        return nil();
    }
    l.unlock();
}

OutputObject FrontendWorker::get_virtual_object_pose_by_id(size_t id) {
    auto l = lock();
    if (sliding_window_tracker) {
        return sliding_window_tracker->map->get_virtual_object_pose_by_id(id);
    } else {
        return {{0.0, 0.0, 0.0, 1.0}, {1000.0, 1000.0, 1000.0}, 1};
    }
    l.unlock();
}

SysState FrontendWorker::get_system_state() const {
    if (initializer) {
        return SysState::SYS_INITIALIZING;
    } else if (sliding_window_tracker) {
        return SysState::SYS_TRACKING;
    }
    return SysState::SYS_UNKNOWN;
}

void FrontendWorker::query_frame() {
    if (localizer)
        localizer->query_frame();
}

bool FrontendWorker::global_localization_state() const {
    return global_localization_flag;
}

void FrontendWorker::set_global_localization_state(bool state) {
    global_localization_flag = state;
}

} // namespace xrslam
