#ifndef XRSLAM_PC_IMAGE_UNDISTORTER_H
#define XRSLAM_PC_IMAGE_UNDISTORTER_H

#include <opencv2/opencv.hpp>
#include <string>

namespace xrslam::extra {

class ImageUndistorter {
  public:
    ImageUndistorter(size_t width, size_t height, const matrix<3> &K,
                     const std::vector<double> &distort_coeffs,
                     const std::string &model)
        : width(width), height(height), K(K), distort_coeffs(distort_coeffs),
          model(model) {
        const cv::Size image_size = {(int)width, (int)height};
        cv::Mat map_x_float(image_size, CV_32FC1);
        cv::Mat map_y_float(image_size, CV_32FC1);
        // Compute the remap maps
        for (size_t v = 0; v < height; ++v) {
            for (size_t u = 0; u < width; ++u) {
                vector<2> distorted_location = distort_pixel({u, v});
                map_x_float.at<float>(v, u) =
                    static_cast<float>(distorted_location.x());
                map_y_float.at<float>(v, u) =
                    static_cast<float>(distorted_location.y());
            }
        }
        cv::convertMaps(map_x_float, map_y_float, map_x, map_y, CV_16SC2);
    }

    void undistort_image(const cv::Mat &image, cv::Mat &image_undistorted) {
        cv::remap(image, image_undistorted, map_x, map_y, cv::INTER_LINEAR,
                  cv::BORDER_CONSTANT);
    }

    vector<2> distort_pixel(const vector<2> &undistort_location) {
        // transform image coordinates to be size and focus independent
        Eigen::Vector3d norm_pixel_location =
            K.inverse() *
            vector<3>(undistort_location.x(), undistort_location.y(), 1.0);

        const double &x = norm_pixel_location.x();
        const double &y = norm_pixel_location.y();

        Eigen::Vector3d norm_distorted_pixel_location(0.0, 0.0,
                                                      norm_pixel_location.z());
        double &xd = norm_distorted_pixel_location.x();
        double &yd = norm_distorted_pixel_location.y();

        const std::vector<double> &D = distort_coeffs;

        if (model == "radtan") {
            const double &k1 = D[0];
            const double &k2 = D[1];
            const double &k3 = D[4];
            const double &p1 = D[2];
            const double &p2 = D[3];
            const double r2 = x * x + y * y;
            const double r4 = r2 * r2;
            const double r6 = r4 * r2;
            const double kr = (1.0 + k1 * r2 + k2 * r4 + k3 * r6);
            xd = x * kr + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
            yd = y * kr + 2.0 * p2 * x * y + p1 * (r2 + 2.0 * y * y);
        } else if (model == "equidistant") {
            const double &k1 = D[0];
            const double &k2 = D[1];
            const double &k3 = D[2];
            const double &k4 = D[3];
            const double r = std::sqrt(x * x + y * y);
            if (r < 1e-10) {
                return undistort_location;
            }
            const double theta = atan(r);
            const double theta2 = theta * theta;
            const double theta4 = theta2 * theta2;
            const double theta6 = theta2 * theta4;
            const double theta8 = theta4 * theta4;
            const double thetad = theta * (1 + k1 * theta2 + k2 * theta4 +
                                           k3 * theta6 + k4 * theta8);
            const double scaling = (r > 1e-8) ? thetad / r : 1.0;
            xd = x * scaling;
            yd = y * scaling;
        } else {
            // TODO(ybbbbt): follow
            // https://github.com/ethz-asl/image_undistort/blob/master/src/undistorter.cpp
            // to add more supports
            throw std::runtime_error("unknown model: " + model);
        }
        return (K * norm_distorted_pixel_location).hnormalized();
    }

  private:
    size_t width;
    size_t height;
    const matrix<3> K;
    const std::string model;
    const std::vector<double> distort_coeffs;
    cv::Mat map_x;
    cv::Mat map_y;
};

} // namespace xrslam::extra

#endif
