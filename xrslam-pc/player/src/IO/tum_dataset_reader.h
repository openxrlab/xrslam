#ifndef XRSLAM_PC_TUM_DATASET_READER_H
#define XRSLAM_PC_TUM_DATASET_READER_H

#include <dataset_reader.h>
#include <deque>
#include <string>
#include <xrslam/extra/image_undistorter.h>

class TUMDatasetReader : public DatasetReader {
  public:
    TUMDatasetReader(const std::string &filename, void *yaml_config);
    NextDataType next() override;
    void get_image_resolution(int &width, int &height) override;
    std::pair<double, cv::Mat> read_image() override;
    std::pair<double, XRSLAMGyroscope> read_gyroscope() override;
    std::pair<double, XRSLAMAcceleration> read_accelerometer() override;
    size_t num_images;

  private:
    std::shared_ptr<xrslam::extra::YamlConfig> config;
    std::deque<std::pair<double, NextDataType>> all_data;
    std::deque<std::pair<double, XRSLAMGyroscope>> gyroscope_data;
    std::deque<std::pair<double, XRSLAMAcceleration>> accelerometer_data;
    std::deque<std::pair<double, std::string>> image_data;
    std::unique_ptr<xrslam::extra::ImageUndistorter> image_undistorter;
    int image_width;
    int image_height;
};

struct TUMCameraCsv {
    struct CameraData {
        double t;
        std::string filename;
    };

    std::vector<CameraData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
            int ret = fscanf(csv, "%2047[^\n]", header_line);
            char filename_buffer[2048];
            CameraData item;
            while (!feof(csv)) {
                memset(filename_buffer, 0, 2048);
                if (fscanf(csv, "%lf,%2047[^\n]", &item.t, filename_buffer) !=
                    2) {
                    break;
                }
                item.t *= 1e-9;
                item.filename = std::string(filename_buffer);
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[ns:double],filename[string]\n", csv);
            for (auto &item : items) {
                fprintf(csv, "%.14e,%s\n", item.t, item.filename.c_str());
            }
            fclose(csv);
        }
    }
};

struct TUMImuCsv {
    struct ImuData {
        double t;
        struct {
            double x;
            double y;
            double z;
        } w;
        struct {
            double x;
            double y;
            double z;
        } a;
    };

    std::vector<ImuData> items;

    void load(const std::string &filename) {
        items.clear();
        if (FILE *csv = fopen(filename.c_str(), "r")) {
            char header_line[2048];
            int ret = fscanf(csv, "%2047[^\n]", header_line);
            ImuData item;
            while (!feof(csv) &&
                   fscanf(csv, "%lf,%lf,%lf,%lf,%lf,%lf,%lf\r\n", &item.t,
                          &item.w.x, &item.w.y, &item.w.z, &item.a.x, &item.a.y,
                          &item.a.z) == 7) {
                item.t *= 1e-9;
                items.emplace_back(std::move(item));
            }
            fclose(csv);
        }
    }

    void save(const std::string &filename) const {
        if (FILE *csv = fopen(filename.c_str(), "w")) {
            fputs("#t[s:double],w.x[rad/s:double],w.y[rad/s:double],w.z[rad/"
                  "s:double],a.x[m/s^2:double],a.y[m/s^2:double],a.z[m/"
                  "s^2:double]\n",
                  csv);
            for (auto item : items) {
                fprintf(csv, "%.14e,%.9e,%.9e,%.9e,%.9e,%.9e,%.9e\n", item.t,
                        item.w.x, item.w.y, item.w.z, item.a.x, item.a.y,
                        item.a.z);
            }
            fclose(csv);
        }
    }
};

#endif
