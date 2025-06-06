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

    bool judge_track_status();
    void fuse_imu_track();
    bool check_frames_rpe(Track *track, const vector<3> &p);
    void predict_RT(Frame *frame_i, Frame *frame_j, matrix<3> &R, vector<3> &t);
    bool filter_parsac_2d2d(Frame *frame_i, Frame *frame_j,
                            std::vector<char> &mask,
                            std::vector<size_t> &pts_to_index);
    void update_track_status();

    std::vector<vector<3>> m_P3D;
    std::vector<vector<2>> m_P2D;
    std::vector<size_t> m_lens;
    std::vector<int> m_indices_map;

    bool track();

    std::tuple<double, PoseState, MotionState> get_latest_state() const;

    double m_th;
    std::unique_ptr<Map> map;
    std::shared_ptr<Map> feature_tracking_map;

    void set_detail(XRSLAM::Detail *detail) { this->detail = detail; }
  private:
    std::shared_ptr<Config> config;

    XRSLAM::Detail *detail;
};

} // namespace xrslam

#endif // XRSLAM_SLIDING_WINDOW_TRACKER_H
