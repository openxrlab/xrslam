#ifndef XRSLAM_PC_OPENCV_PAINTER_H
#define XRSLAM_PC_OPENCV_PAINTER_H

#include <xrslam/extra/opencv_image.h>
#include <xrslam/inspection.h>
#include <xrslam/xrslam.h>

class OpenCvPainter : public xrslam::InspectPainter {
    cv::Mat &canvas;

  public:
    OpenCvPainter(cv::Mat &canvas) : canvas(canvas) {}

    void set_image(const xrslam::Image *image) override {
        const xrslam::extra::OpenCvImage *cvimage =
            dynamic_cast<const xrslam::extra::OpenCvImage *>(image);
        if (cvimage->raw.channels() == 3)
            canvas = cvimage->raw.clone();
        else
            cv::cvtColor(cvimage->raw, canvas, cv::COLOR_GRAY2BGR);
    }

    void point(const xrslam::point2i &p, const xrslam::color3b &c, int size = 1,
               int style = 0) override {
        cv::Point center(p.x(), p.y());
        cv::Scalar color = CV_RGB(c.x(), c.y(), c.z());
        switch (style) {
        case 0: // .
        {
            cv::circle(canvas, center, size, color, -1);
            break;
        }
        case 1: // o
        {
            cv::circle(canvas, center, size, color, 3);
            break;
        }
        case 2: // +
        {
            cv::Point dx(size, 0);
            cv::Point dy(0, size);
            cv::line(canvas, center + dx, center - dx, color);
            cv::line(canvas, center + dy, center - dy, color);
            break;
        }
        case 3: // x
        {
            cv::Point du(size, size);
            cv::Point dv(size, -size);
            cv::line(canvas, center + du, center - du, color);
            cv::line(canvas, center + dv, center - dv, color);
            break;
        }
        }
    }

    void line(const xrslam::point2i &p1, const xrslam::point2i &p2,
              const xrslam::color3b &c, int thickness = 1) override {
        cv::Point point1(p1.x(), p1.y());
        cv::Point point2(p2.x(), p2.y());
        cv::Scalar color = CV_RGB(c.x(), c.y(), c.z());
        cv::line(canvas, point1, point2, color, thickness);
    }
};

#endif // XRSLAM_PC_OPENCV_PAINTER_H
