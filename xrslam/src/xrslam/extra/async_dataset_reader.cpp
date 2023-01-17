#include <xrslam/extra/async_dataset_reader.h>
AsyncDatasetReader::AsyncDatasetReader(std::unique_ptr<DatasetReader> reader)
    : reader(std::move(reader)) {
    reader_running = true;
    reader_thread = std::thread(&AsyncDatasetReader::reader_loop, this);
}

AsyncDatasetReader::~AsyncDatasetReader() {
    if (reader_running) {
        reader_running = false;
        reader_cv.notify_all();
    }
    reader_thread.join();
}

DatasetReader::NextDataType AsyncDatasetReader::next() {
    std::lock_guard lock(reader_mutex);
    double image_time = std::numeric_limits<double>::max();
    double gyroscope_time = std::numeric_limits<double>::max();
    double accelerometer_time = std::numeric_limits<double>::max();
    bool has_data = false;
    if (pending_images.size() > 0) {
        image_time = pending_images.front()->t;
        has_data = true;
    }
    if (pending_gyroscopes.size() > 0) {
        gyroscope_time = pending_gyroscopes.front().first;
        has_data = true;
    }
    if (pending_accelerometers.size() > 0) {
        accelerometer_time = pending_accelerometers.front().first;
        has_data = true;
    }
    if (!has_data) {
        if (!EOD) {
            return AGAIN;
        } else {
            return END;
        }
    }
    if (gyroscope_time <= image_time && gyroscope_time <= accelerometer_time) {
        return GYROSCOPE;
    } else if (accelerometer_time < gyroscope_time &&
               accelerometer_time <= image_time) {
        return ACCELEROMETER;
    } else {
        return CAMERA;
    }
}

std::shared_ptr<xrslam::Image> AsyncDatasetReader::read_image() {
    std::lock_guard lock(reader_mutex);
    auto r = pending_images.front();
    pending_images.pop();
    reader_cv.notify_all();
    return r;
}

std::pair<double, xrslam::vector<3>> AsyncDatasetReader::read_gyroscope() {
    std::lock_guard lock(reader_mutex);
    auto r = pending_gyroscopes.front();
    pending_gyroscopes.pop();
    return r;
}

std::pair<double, xrslam::vector<3>> AsyncDatasetReader::read_accelerometer() {
    std::lock_guard lock(reader_mutex);
    auto r = pending_accelerometers.front();
    pending_accelerometers.pop();
    return r;
}

bool AsyncDatasetReader::data_available() const {
    return pending_images.size() > 10;
}

void AsyncDatasetReader::reader_loop() {
    while (reader_running) {
        auto reader_lock = std::unique_lock(reader_mutex);
        if (reader_running && data_available()) {
            reader_cv.wait(reader_lock, [this] {
                return !reader_running || !data_available();
            });
        }
        if (!reader_running)
            break;
        DatasetReader::NextDataType next_type;
        while ((next_type = reader->next()) == DatasetReader::AGAIN) {
        }
        switch (next_type) {
        case DatasetReader::CAMERA: {
            pending_images.emplace(reader->read_image());
        } break;
        case DatasetReader::GYROSCOPE: {
            pending_gyroscopes.emplace(reader->read_gyroscope());
        } break;
        case DatasetReader::ACCELEROMETER: {
            pending_accelerometers.emplace(reader->read_accelerometer());
        } break;
        case DatasetReader::AGAIN:
        case DatasetReader::END:
            EOD = true;
            reader_running = false;
        }
    }
}
