#ifndef _VISUALIZER_H_
#define _VISUALIZER_H_

#ifndef XRSLAM_PC_HEADLESS_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <condition_variable>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "glbinding-aux/types_to_string.h"
#include "lightvis.h"
#include "nuklear.h"

#include "opencv_painter.h"
#include "xrslam/inspection.h"

struct VizElements {
    std::vector<Eigen::Vector3f> landmarks;
    std::vector<Eigen::Vector4f> landmarks_color;
    cv::Mat show_frame; // bgr image
    Eigen::Quaterniond camera_q;
    Eigen::Vector3d camera_p;
    Eigen::Vector3d acc_bias;
    Eigen::Vector3d gyro_bias;
};

class Visualizer : public lightvis::LightVis {
  public:
    Visualizer(bool play, const std::string &config_file);
    ~Visualizer();
    void main();
    int is_running();
    void load() override;
    void unload() override;
    void gui(void *ctx, int w, int h) override;
    void draw(int w, int h) override;
    void update_frame(const std::shared_ptr<VizElements> curframe);

  private:
    void read_device_params(const std::string &config_file);

    int is_playing = 0;
    int running = 0;
    // std::mutex frame_mutex;
    std::condition_variable con_var;

    std::vector<Eigen::Vector3f> trajectory;
    std::vector<Eigen::Vector3f> landmarks;
    Eigen::Vector4f trajectory_color;
    Eigen::Vector4f camera_color;
    std::vector<Eigen::Vector4f> landmarks_color;
    std::deque<Eigen::Vector3d> bg_list, ba_list;

    cv::Mat feature_tracker_cvimage;
    std::unique_ptr<xrslam::InspectPainter> feature_tracker_painter;
    std::unique_ptr<lightvis::Image> feature_tracker_image;

    std::vector<Eigen::Vector3f> positions;
    Eigen::Quaterniond latest_camera_q;
    Eigen::Vector3d latest_camera_p;

    std::vector<double> bg_x, bg_y, bg_z, ba_x, ba_y, ba_z;

    // device config
    Eigen::Matrix3d camera_intrinsic;
    Eigen::Quaterniond camera_to_body_rotation;
    Eigen::Vector3d camera_to_body_translation;
};

#endif

#endif