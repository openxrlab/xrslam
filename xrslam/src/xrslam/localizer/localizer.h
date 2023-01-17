#include "base64_convert.h"
#include "httplib.h"
#include "json.h"
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <xrslam/bridge.h>

namespace xrslam {

enum ScreenState { Portrait, Down, Left, Right };

class Localizer {
  public:
    Localizer(std::shared_ptr<Config> config);

    ~Localizer();

    void test_connection();
    void query_loc(const cv::Mat cvimg, const Pose &T_slam_body,
                   ScreenState screenState);
    void add_pose_message(const double timestamp, const Pose &pose);
    void send_pose();
    Pose transform(const Pose &pose);
    bool is_initialized() const;

    void query_frame();
    void query_localization(std::shared_ptr<xrslam::Image> img, Pose pose);
    void send_pose_message(double time);

    ScreenState get_screenstate(const Eigen::Matrix3d R);
    cv::Mat get_image_by_screenstate(const ScreenState &screenState,
                                     const cv::Mat &img);
    std::vector<float> rotate_intrinsic(const ScreenState screenState, size_t w,
                                        size_t h);

  private:
    std::string url;
    size_t port;

    std::shared_ptr<Config> config;
    Pose slam_to_sfm;

    size_t inliers_th = 50;

    mutable std::mutex transform_mutex;
    mutable std::mutex send_pose_mutex;
    std::vector<float> poses;
    std::vector<float> intrinsic;
    std::vector<float> distortion;

    double send_image_interval = 5.0;
    double send_poses_interval = 1.0;
    double image_last_time = 0.0;
    double poses_last_time = 0.0;

    bool init_flag;
    bool query_frame_flag = false;
    bool verbose = false;
};

} // namespace xrslam
