#ifndef XRSLAM_PC_ASYNC_DATASET_READER_H
#define XRSLAM_PC_ASYNC_DATASET_READER_H

#include <atomic>
#include <condition_variable>
#include <dataset_reader.h>
#include <mutex>
#include <queue>
#include <thread>

class AsyncDatasetReader : public DatasetReader {
  public:
    AsyncDatasetReader(std::unique_ptr<DatasetReader> reader);
    ~AsyncDatasetReader();

    NextDataType next() override;
    std::shared_ptr<xrslam::Image> read_image() override;
    std::pair<double, xrslam::vector<3>> read_gyroscope() override;
    std::pair<double, xrslam::vector<3>> read_accelerometer() override;

  private:
    bool data_available() const;
    void reader_loop();

    std::unique_ptr<DatasetReader> reader;
    std::atomic<bool> reader_running;
    std::thread reader_thread;
    std::mutex reader_mutex;
    std::condition_variable reader_cv;

    std::queue<std::pair<double, xrslam::vector<3>>> pending_gyroscopes;
    std::queue<std::pair<double, xrslam::vector<3>>> pending_accelerometers;
    std::queue<std::shared_ptr<xrslam::Image>> pending_images;
    bool EOD = false;
};

#endif // XRSLAM_PC_ASYNC_DATASET_READER_H
