#ifndef XRSLAM_XRSLAM_H
#define XRSLAM_XRSLAM_H

#include <stdint.h>

/******************************************************************************************/
/************************************XRSLAM sensor data************************************/
/******************************************************************************************/
namespace xrslam {
typedef enum XRSLAMSensorType {
    XRSLAM_SENSOR_CAMERA = 0,
    XRSLAM_SENSOR_DEPTH_CAMERA,
    XRSLAM_SENSOR_ACCELERATION,
    XRSLAM_SENSOR_GYROSCOPE,
    XRSLAM_SENSOR_GRAVITY,
    XRSLAM_SENSOR_ROTATION_VECTOR,
    XRSLAM_SENSOR_UNKNOWN
} XRSLAMSensorType;

typedef struct XRSLAMImageExtension {
    double exposure_time;               // image exposure time
    double default_focus_distance;      // default focus info
    double focal_length;                // current focal length
    double focus_distance;              // current focus distance
} XRSLAMImageExtension;

typedef struct XRSLAMImage {
    unsigned char* data;	            // contains the intensity value for each image pixel
    double timeStamp;		            // timestamp in second
    int stride;				            // image stride, number of bytes per row
    int camera_id;                      // camera id
    XRSLAMImageExtension *ext;          // ext info of image
} XRSLAMImage;

typedef struct XRSLAMDepthImage {
    uint16_t* data;        // the warped depth data
    uint16_t* confidence;  // the warped confidence
    double timeStamp;      //  timestamp in second
} XRSLAMDepthImage;

typedef struct XRSLAMAcceleration {
    double data[3];  		// acceleration raw data
    double timestamp;		// timestamp in second
} XRSLAMAcceleration;

typedef struct XRSLAMGyroscope {
    double data[3];  		// gyroscope raw data
    double timestamp;		// timestamp in second
} XRSLAMGyroscope;

typedef struct XRSLAMGravity {
    double data[3];         // gravity direction
    double timestamp;       // timestamp in second
} XRSLAMGravity;

typedef struct XRSLAMRotationVector {
    double data[4];         // attitude of the device
    double timestamp;       // timestamp in second
} XRSLAMRotationVector;


/******************************************************************************************/
/************************************XRSLAM tracking result********************************/
/******************************************************************************************/

typedef enum XRSLAMResultType {
    XRSLAM_RESULT_BODY_POSE = 0,
    XRSLAM_RESULT_CAMERA_POSE,
    XRSLAM_RESULT_STATE,
    XRSLAM_RESULT_LANDMARKS,
    XRSLAM_RESULT_FEATURES,
    XRSLAM_RESULT_BIAS,
    XRSLAM_RESULT_PLANES,
    XRSLAM_RESULT_DEBUG_LOGS,
    XRSLAM_RESULT_VERSION,
    XRSLAM_RESULT_UNKNOWN
} XRSLAMResultType;

/* for a 3D point in world coordinate X_w, its 3D
coordinate in the camera frame X_c = R * X_w + T */
typedef struct XRSLAMPose {
    double quaternion[4];   // quaternion of rotation: format [x, y, z, w]
    double translation[3];  // translation vector T
    double timestamp;       // timestamp
} XRSLAMPose;

/* slam result */
typedef enum XRSLAMState {
    XRSLAM_STATE_INITIALIZING,            // SLAM is in initialize state
    XRSLAM_STATE_TRACKING_SUCCESS,        // SLAM is in tracking state and track success
    XRSLAM_STATE_TRACKING_FAIL,           // SLAM is in tracking state and track fail
    XRSLAM_STATE_TRACKING_RESET,          // SLAM system need to reset
    XRSLAM_STATE_TRACKING_RELOCALIZING,   // SLAM is in reloc state
    XRSLAM_STATE_TRACKING_SUCCESS_NO_MAP  // SLAM Tracking succ, but map not expand OK
} XRSLAMState;

/* landmark 3d position */
typedef struct XRSLAMLandmark {
    double x, y, z;  // x, y, z is world position of the point
} XRSLAMLandmark;
typedef struct XRSLAMLandmarks {
    XRSLAMLandmark *landmarks;
    int num_landmarks;
}XRSLAMLandmarks;

/* corner in image coordinate */
typedef struct XRSLAMFeature { double x, y; } XRSLAMFeature;
typedef struct XRSLAMFeatures {
    XRSLAMFeature *features;
    int num_features;
}XRSLAMFeatures;

typedef struct XRSLAMBias {
    double data[3];
} XRSLAMBias;
typedef struct XRSLAMIMUBias {
    XRSLAMBias acc_bias;
    XRSLAMBias gyr_bias;
} XRSLAMIMUBias;

typedef struct XRSLAMStringOutput
{
    int str_length;
    char *data;       // slam information
}XRSLAMLogs;


/******************************************************************************************/
/************************************XRSLAM function interface*****************************/
/******************************************************************************************/

/* @brief: create SLAM system with configuration files
* @input param: slam parameter file path, device parameter file path
* @return value: 1 success, otherwise 0
* */
int XRSLAMCreate(
    const char* slam_config_path,       // slam configuration file path
    const char* device_config_path      // device configuration file path
    );

/* @brief: push sensor data
* @input param: sensor type, sensor data
* */
void XRSLAMPushSensorData(
    XRSLAMSensorType sensor_type,       // sensor type
    void *sensor_data               // sensor data
    );

/* @brief: end one frame input and run slam
* */
void XRSLAMRunOneFrame();

/* @brief: get SLAM tracking result at time t
* @input param: time t
* @return value: slam tracking result
* */
void XRSLAMGetResult(
    XRSLAMResultType result_type,   // result type
    void *result_data               // result data
    );

/* @brief: destroy SLAM system
* */
void XRSLAMDestroy();
} // namespace xrslam
#endif  // XRSLAM_XRSLAM_H