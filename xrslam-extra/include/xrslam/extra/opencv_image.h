#ifndef XRSLAM_EXTRA_OPENCV_IMAGE_H
#define XRSLAM_EXTRA_OPENCV_IMAGE_H

#include <ceres/cubic_interpolation.h>
#include <opencv2/opencv.hpp>
#include <xrslam/xrslam.h>

namespace xrslam::extra {

class OpenCvImage : public Image {
  public:
    OpenCvImage();

    uchar *get_rawdata() const override { return raw.data; }

    size_t width() const override { return image.cols; }

    size_t height() const override { return image.rows; }

    size_t level_num() const override { return 3; }

    double evaluate(const vector<2> &u, int level = 0) const override;
    double evaluate(const vector<2> &u, vector<2> &ddu,
                    int level = 0) const override;

    void detect_keypoints(std::vector<vector<2>> &keypoints,
                          size_t max_points = 1000,
                          double keypoint_distance = 10) const override;
    void track_keypoints(const Image *next_image,
                         const std::vector<vector<2>> &curr_keypoints,
                         std::vector<vector<2>> &next_keypoints,
                         std::vector<char> &result_status) const override;

    void preprocess(double clipLimit, int width, int height) override;
    void correct_distortion(const matrix<3> &intrinsics,
                            const vector<4> &coeffs);
    void release_image_buffer() override;

    cv::Mat image;
    cv::Mat raw;

  private:
    std::vector<cv::Mat> image_pyramid;
    std::vector<cv::Mat> image_levels;
    std::vector<vector<2>> scale_levels;

    typedef ceres::Grid2D<unsigned char, 1> Grid;
    std::vector<Grid> grid_levels;

    typedef ceres::BiCubicInterpolator<Grid> Interpolator;
    std::vector<Interpolator> interpolator_levels;

    static cv::CLAHE *clahe(double clipLimit, int width, int height);
    static cv::GFTTDetector *gftt(size_t max_points);
    static cv::FastFeatureDetector *fast();
    static cv::ORB *orb();
};

} // namespace xrslam::extra

#endif /* XRSLAM_EXTRA_OPENCV_IMAGE_H */
