# Configuration Parse

## Linux/Mac

The following is the parameter description of the PC configuration, include [slam parameters](../../configs/euroc_slam.yaml) and [device parameters](../../configs/euroc_sensor.yaml).

### output

* `q_bo`: the rotation part of extrinsics from output to body in quaternion, (x y z w)
* `p_bo`: the translation part of extrinsics from output to body, (x y z)

### camera

* `noise`: the convariance matrix for keypoint measurement noise, (pixel^2)
* `distortion`: the distortion coefficient of camera, (k1 k2 p1 p2)
* `intrinsic`: the intrinsics of camera (fx fy cx cy)
* `q_bc`: the rotation part of extrinsics from camera to body
* `p_bc`: the translation part of extrinsics from camera to body
* **note**: if you need to modify the `distortion` and `intrinsic`, remember the content at `xrslam-pc/player/src/euroc_dataset_reader.cpp` L55,56 to be modified at the same time.

### imu

* `cov_g`: the convariance matrix for gyroscope white noise, (rad/s/sqrt(hz))^2)
* `cov_a`: the convariance matrix for accelerometer  white noise, ((m/s^2/sqrt(hz))^2)
* `cov_bg`: the convariance matrix for gyroscope bias diffusion, ((rad/s^2/sqrt(hz))^2)
* `cov_ba`: the convariance matrix for accelerometer bias diffusion, ((m/s^3/sqrt(hz))^2)
* `q_bi`: the rotation part of extrinsics from IMU to body
* `p_bi`: the translation part of extrinsics from IMU to body

### sliding window

* `size`: the size of keyframes in the sliding window
* `force_keyframe_landmark`: decide keyframe if the number of landmarks observed at this frame is below a certain threshold

### feature_tracker

* `min_keypoint_distance`: the minimum distance between two detected keypoints
* `max_keypoint_detection`: the maxmium number of keypoints detected in each frame
* `max_init_frames`: the maxmium number of frames in the map of feature tracker before initialization
* `max_frames`: the maxmium number of frames in the map of feature tracker after initialization
* `predict_keypoints`: weather to predict key keypoints according to transformation

### initializer

* `keyframe_num`: the number of keyframes required for initialization
* `keyframe_gap`: the number of frames between two adjacent keyframes
* `min_matches`: the minimum matches between the first and last frame for sfm initialization
* `min_parallax`: the minimum average parallax between the first and last frame
* `min_triangulation`: the minimum triangulation counts between the first and last frame
* `min_landmarks`: the minimum number of landmarks in the map for initialization
* `refine_imu`: weather to refine scale and velocity via gravity

### solver

* `iteration_limit`: the maxmium number of iteration for each optimization
* `time_limit`: the maxmium time which optimizer cost for each optimization

## iOS

iOS configuration also includes [slam parameters](../../xrslam-ios/visualizer/configs/slam_params.yaml) and [iPhone Parameters](../../xrslam-ios/visualizer/configs/iPhone&#32;X.yaml).

### output

* `q_bo`: the rotation part of extrinsics from output to body in quaternion, (x y z w)
* `p_bo`: the translation part of extrinsics from output to body, (x y z)

### camera

* `noise`: the convariance matrix for keypoint measurement noise, (pixel^2)
* `distortion`: the distortion coefficient of camera, (k1 k2 p1 p2)
* `intrinsic`: the intrinsics of camera (fx fy cx cy)
* `q_bc`: the rotation part of extrinsics from camera to body
* `p_bc`: the translation part of extrinsics from camera to body
* **note**: if you need to modify the `distortion` and `intrinsic`, remember the content at `xrslam-pc/player/src/euroc_dataset_reader.cpp` L55,56 to be modified at the same time.

### imu

* `cov_g`: the convariance matrix for gyroscope white noise, (rad/s/sqrt(hz))^2)
* `cov_a`: the convariance matrix for accelerometer  white noise, ((m/s^2/sqrt(hz))^2)
* `cov_bg`: the convariance matrix for gyroscope bias diffusion, ((rad/s^2/sqrt(hz))^2)
* `cov_ba`: the convariance matrix for accelerometer bias diffusion, ((m/s^3/sqrt(hz))^2)
* `q_bi`: the rotation part of extrinsics from IMU to body
* `p_bi`: the translation part of extrinsics from IMU to body

### sliding window

* `size`: the size of keyframes in the sliding window
* `tracker_frequent`: use one frame for every n frames

### feature_tracker

* `min_keypoint_distance`: the minimum distance between two detected keypoints
* `max_keypoint_detection`: the maxmium number of keypoints detected in each frame
* `max_frames`: the maxmium number of frames in the map of feature tracker after initialization

### solver

* `iteration_limit`: the maxmium number of iteration for each optimization
* `time_limit`: the maxmium time which optimizer cost for each optimization

### visual_localization

* `enable`:  enable sfm-based localization or not
* `ip`:  server IP

* `port`: server port
