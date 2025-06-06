#ifndef SENSORS_LIBSENSORS_H
#define SENSORS_LIBSENSORS_H

#include <memory>

namespace libsensors {

class Sensors {
    class SensorsImpl;
    friend class SensorsImpl;
    std::unique_ptr<SensorsImpl> pimpl;

  public:
    Sensors();
    virtual ~Sensors();

    void parse_data(const void* bytes, size_t size);

  protected:
    virtual void on_image(double t, int width, int height, const unsigned char* bytes) {
    }

    virtual void on_gyroscope(double t, double x, double y, double z) {
    }

    virtual void on_accelerometer(double t, double x, double y, double z) {
    }

    virtual void on_magnetometer(double t, double x, double y, double z) {
    }

    virtual void on_altimeter(double t, double pressure, double elevation) {
    }

    virtual void on_gps(double t, double longitude, double latitude, double altitude, double horizontal_accuracy, double vertical_accuracy) {
    }

    virtual void on_gravity(double t, double x, double y, double z) {
    }

    virtual void on_error(const char* msg) {
    }
};

} // namespace libsensors

#endif // SENSORS_LIBSENSORS_H
