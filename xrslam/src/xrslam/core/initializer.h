#ifndef XRSLAM_INITIALIZER_H
#define XRSLAM_INITIALIZER_H

#include <xrslam/common.h>

namespace xrslam {

class Config;
class Map;
class SlidingWindowTracker;

class Initializer {
  public:
    Initializer(std::shared_ptr<Config> config);
    ~Initializer();

    void mirror_keyframe_map(Map *feature_tracking_map, size_t init_frame_id);
    std::unique_ptr<SlidingWindowTracker> initialize();

    std::unique_ptr<Map> map;

  private:
    bool init_sfm();
    bool init_imu();

    void solve_gyro_bias();
    void solve_gravity_scale_velocity();
    void refine_scale_velocity_via_gravity();

    void reset_states();
    void preintegrate();
    bool apply_init(bool apply_ba = false, bool apply_velocity = true);

    vector<3> bg;
    vector<3> ba;
    vector<3> gravity;
    double scale;
    std::vector<vector<3>> velocities;

    std::shared_ptr<Config> config;
};

} // namespace xrslam

#endif // XRSLAM_INITIALIZER_H
