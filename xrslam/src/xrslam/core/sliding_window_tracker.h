#ifndef XRSLAM_SLIDING_WINDOW_TRACKER_H
#define XRSLAM_SLIDING_WINDOW_TRACKER_H

#include <xrslam/common.h>
#include <xrslam/estimation/state.h>

namespace xrslam {

class Config;
class Frame;
class Map;

class SlidingWindowTracker {
  public:
    SlidingWindowTracker(std::unique_ptr<Map> keyframe_map,
                         std::shared_ptr<Config> config);
    ~SlidingWindowTracker();

    void mirror_frame(Map *feature_tracking_map, size_t frame_id);

    void localize_newframe();
    void track_landmark();
    void refine_window();
    void slide_window();
    bool manage_keyframe();

    void refine_subwindow();

    bool track();

    std::tuple<double, PoseState, MotionState> get_latest_state() const;

    std::unique_ptr<Map> map;

  private:
    std::shared_ptr<Config> config;
};

} // namespace xrslam

#endif // XRSLAM_SLIDING_WINDOW_TRACKER_H
