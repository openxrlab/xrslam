/**
 * @file XRSLAM.h
 * @brief XRSlam API
 */

#ifndef _XRSLAM_H_
#define _XRSLAM_H_
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************************
 *                                 XRSLAM sensor data
 ******************************************************************************************/
/**
 * @brief input sensor data type
 */
typedef enum XRSLAMSensorType {
    XRSLAM_SENSOR_CAMERA = 0,      /*!< gray image. */
    XRSLAM_SENSOR_DEPTH_CAMERA,    /*!< depth image. */
    XRSLAM_SENSOR_ACCELERATION,    /*!< acceleration. */
    XRSLAM_SENSOR_GYROSCOPE,       /*!< gyroscope. */
    XRSLAM_SENSOR_GRAVITY,         /*!< gravity. */
    XRSLAM_SENSOR_ROTATION_VECTOR, /*!< rotation vector. */
    XRSLAM_SENSOR_UNKNOWN
} XRSLAMSensorType;

/**
 * @brief Image extension info
 */
typedef struct XRSLAMImageExtension {
    double exposure_time;          /*!< image exposure time. */
    double default_focus_distance; /*!< default focus info. */
    double focal_length;           /*!< current focal length. */
    double focus_distance;         /*!< current focus distance. */
} XRSLAMImageExtension;

/**
 * @brief input gray image data
 */
typedef struct XRSLAMImage {
    unsigned char
        *data;        /*!< contains the intensity value for each image pixel. */
    double timeStamp; /*!< timestamp in second. */
    int stride;       /*!< image stride, number of bytes per row. */
    int camera_id;    /*!< camera id. */
    XRSLAMImageExtension *ext; /*!< ext info of image. */
} XRSLAMImage;

/**
 * @brief input depth image data
 */
typedef struct XRSLAMDepthImage {
    uint16_t *data;       /*!< the warped depth data. */
    uint16_t *confidence; /*!< the warped confidence. */
    double timeStamp;     /*!<  timestamp in second. */
} XRSLAMDepthImage;

/**
 * @brief input acceleration data
 */
typedef struct XRSLAMAcceleration {
    double data[3];   /*!< acceleration raw data. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMAcceleration;

/**
 * @brief input gyroscope data
 */
typedef struct XRSLAMGyroscope {
    double data[3];   /*!< gyroscope raw data. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMGyroscope;

/**
 * @brief input gravity direction data
 */
typedef struct XRSLAMGravity {
    double data[3];   /*!< gravity direction. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMGravity;

/**
 * @brief attitude of the device
 */
typedef struct XRSLAMRotationVector {
    double data[4];   /*!< attitude of the device. */
    double timestamp; /*!< timestamp in second. */
} XRSLAMRotationVector;

/******************************************************************************************
 *                                   XRSLAM tracking result
 ******************************************************************************************/
/**
 * @brief slam result data type
 */
typedef enum XRSLAMResultType {
    XRSLAM_RESULT_BODY_POSE = 0, /*!< body pose. */
    XRSLAM_RESULT_CAMERA_POSE,   /*!< camera pose. */
    XRSLAM_RESULT_STATE,         /*!< system state. */
    XRSLAM_RESULT_LANDMARKS,     /*!< 3D landmarks. */
    XRSLAM_RESULT_FEATURES,      /*!< 2D features. */
    XRSLAM_RESULT_BIAS,          /*!< imu bias. */
    XRSLAM_RESULT_DEBUG_LOGS,    /*!< debug logs. */
    XRSLAM_RESULT_VERSION,       /*!< version. */
    XRSLAM_RESULT_UNKNOWN
} XRSLAMResultType;

/**
 * @brief    slam pose.
 * @details  For a 3D point in world coordinate \f$ X_w \f$, its 3D
 *           coordinate in the camera frame \f$ X_c = R * X_w + T \f$
 */
typedef struct XRSLAMPose {
    double quaternion[4];  /*!< quaternion of rotation: format [x, y, z, w]. */
    double translation[3]; /*!< translation vector T. */
    double timestamp;      /*!< timestamp. */
} XRSLAMPose;

/**
 * @brief    slam state.
 * @details  SLAM state to show the system status.
 */
typedef enum XRSLAMState {
    XRSLAM_STATE_INITIALIZING,     /*!< SLAM is in initialize state. */
    XRSLAM_STATE_TRACKING_SUCCESS, /*!< SLAM is in tracking state and track
                                      success. */
    XRSLAM_STATE_TRACKING_FAIL /*!< SLAM is in tracking state and track fail. */
} XRSLAMState;

/**
 * @brief  landmark 3d position.
 * @details x, y, z is world position of the point.
 */
typedef struct XRSLAMLandmark {
    double x, y, z;
} XRSLAMLandmark;
typedef struct XRSLAMLandmarks {
    XRSLAMLandmark *landmarks;
    int num_landmarks;
} XRSLAMLandmarks;

/**
 * @brief  2d corner in image coordinate.
 */
typedef struct XRSLAMFeature {
    double x, y;
} XRSLAMFeature;
typedef struct XRSLAMFeatures {
    XRSLAMFeature *features;
    int num_features;
} XRSLAMFeatures;

/**
 * @brief IMU bias.
 */
typedef struct XRSLAMBias {
    double data[3];
} XRSLAMBias;
typedef struct XRSLAMIMUBias {
    XRSLAMBias acc_bias;
    XRSLAMBias gyr_bias;
} XRSLAMIMUBias;
/**
 * @brief  debug logs.
 */
typedef struct XRSLAMStringOutput {
    int str_length;
    char *data; /*!< slam information. */
} XRSLAMStringOutput;

/******************************************************************************************
 *                                   XRSLAM function interface
 ******************************************************************************************/

/**
 * @brief create SLAM system with configuration files.
 * @param[in] slam_config_path slam configuration file path.
 * @param[in] device_config_path  device configuration file path, include camera
 * intrinsics, extrinsics and so on.
 * @param[in] license_path  license path
 * @param[in] product_name  product name which user can define
 * @return 1 success, otherwise 0
 */
int XRSLAMCreate(const char *slam_config_path, const char *device_config_path,
                 const char *license_path, const char *product_name);

/**
 * @brief push sensor data to SLAM system
 * @param[in] sensor_type sensor type.
 * @param[in] sensor_data sensor data.
 */
void XRSLAMPushSensorData(XRSLAMSensorType sensor_type, void *sensor_data);

/**
 * @brief end one frame input and run slam
 */
void XRSLAMRunOneFrame();

/**
 * @brief get SLAM results
 * @param[in]  result_type slam result type.
 * @param[out] result_data result data.
 */
void XRSLAMGetResult(XRSLAMResultType result_type, void *result_data);

/**
 * @brief destroy SLAM system
 */
void XRSLAMDestroy();

#ifdef __cplusplus
}
#endif

#endif // _XRSLAM_H_
