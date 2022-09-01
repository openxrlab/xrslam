#ifndef XRSLAM_DETAIL_H
#define XRSLAM_DETAIL_H

#include <xrslam/common.h>
#include <xrslam/xrslam.h>

namespace xrslam {

class Config;
class FeatureTracker;
class Frame;
class FrontendWorker;
class Image;
class Map;
class Synchronizer;

struct XRSLAM::Detail {
    struct GyroscopeData {
        double t;
        vector<3> w;
    };
    struct AccelerometerData {
        double t;
        vector<3> a;
    };

  public:
    Detail(std::shared_ptr<Config> config);
    virtual ~Detail();

    const Config *configurations() const;

    Pose track_gyroscope(const double &t, const double &x, const double &y,
                         const double &z);
    Pose track_accelerometer(const double &t, const double &x, const double &y,
                             const double &z);
    Pose track_camera(std::shared_ptr<Image> image);

    std::tuple<double, Pose> get_latest_state() const;

    std::unique_ptr<FeatureTracker> feature_tracker;
    std::unique_ptr<FrontendWorker> frontend;

    SysState get_system_state() const;

    size_t create_virtual_object();
    OutputObject get_virtual_object_pose_by_id(size_t id);

    void enable_global_localization();
    void disable_global_localization();
    void query_frame();
    bool global_localization_initialized();

  private:
    void track_imu(const ImuData &imu);
    Pose predict_pose(const double &t);

    std::deque<GyroscopeData> gyroscopes;
    std::deque<AccelerometerData> accelerometers;

    std::deque<ImuData> imus;
    std::deque<std::unique_ptr<Frame>> frames;
    std::deque<ImuData> frontal_imus;

    std::shared_ptr<Config> config;
};

} // namespace xrslam

#endif // XRSLAM_DETAIL_H
