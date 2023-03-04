#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <queue>
#include <thread>

#include "XRSLAM.h"

std::string IMAGE_TOPIC = "/cam0/image_raw";
std::string IMU_TOPIC = "/imu0";

std::queue<sensor_msgs::ImageConstPtr> image_buf;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
FILE *file;
double last_imu_t = 0;

ros::Publisher pub_traj;
ros::Publisher pub_pose;
nav_msgs::Path traj_msg;


void img_callback(const sensor_msgs::ImageConstPtr &image_msg) {
    image_buf.push(image_msg);
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
    if (imu_msg->header.stamp.toSec() <= last_imu_t) {
        ROS_WARN("imu message in disorder!");
        return;
    }
    last_imu_t = imu_msg->header.stamp.toSec();
    imu_buf.push(imu_msg);
}

void process() {
    while (ros::ok()) {
        if (imu_buf.empty() || image_buf.empty())
            continue;
        auto image_msg = image_buf.front();
        auto imu_msg = imu_buf.front();
        if (image_msg->header.stamp.toSec() > imu_msg->header.stamp.toSec()) {
            XRSLAMGyroscope gyr = {{imu_msg->angular_velocity.x,
                                    imu_msg->angular_velocity.y,
                                    imu_msg->angular_velocity.z},
                                   imu_msg->header.stamp.toSec()};
            XRSLAMAcceleration acc = {{imu_msg->linear_acceleration.x,
                                       imu_msg->linear_acceleration.y,
                                       imu_msg->linear_acceleration.z},
                                      imu_msg->header.stamp.toSec()};
            XRSLAMPushSensorData(XRSLAM_SENSOR_GYROSCOPE, &gyr);
            XRSLAMPushSensorData(XRSLAM_SENSOR_ACCELERATION, &acc);
            imu_buf.pop();
        } else {
            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1") {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img,
                                          sensor_msgs::image_encodings::MONO8);
            } else
                ptr = cv_bridge::toCvCopy(image_msg,
                                          sensor_msgs::image_encodings::MONO8);

            cv::Mat img_distorted = ptr->image;

            cv::Mat img;
            cv::Mat dist_coeffs = (cv::Mat_<float>(4, 1) << -0.28340811,
                                   0.07395907, 0.00019359, 1.76187114e-05);
            cv::Mat K = (cv::Mat_<float>(3, 3) << 458.654, 0, 367.215, 0,
                         457.296, 248.375, 0, 0, 1);
            cv::undistort(img_distorted, img, K, dist_coeffs);

            XRSLAMImage image;
            image.camera_id = 0;
            image.timeStamp = image_msg->header.stamp.toSec();
            image.ext = nullptr;
            image.data = img.data;
            image.stride = img.step[0];

            XRSLAMPushSensorData(XRSLAM_SENSOR_CAMERA, &image);
            XRSLAMRunOneFrame();
            XRSLAMState state;
            XRSLAMGetResult(XRSLAM_RESULT_STATE, &state);
            if (state == XRSLAM_STATE_TRACKING_SUCCESS) {
                XRSLAMPose pose_b;
                XRSLAMGetResult(XRSLAM_RESULT_BODY_POSE, &pose_b);
                if (pose_b.timestamp > 0) {
                    fprintf(file, "%.18e %.9e %.9e %.9e %.7e %.7e %.7e %.7e\n",
                            pose_b.timestamp, pose_b.translation[0],
                            pose_b.translation[1], pose_b.translation[2],
                            pose_b.quaternion[0], pose_b.quaternion[1],
                            pose_b.quaternion[2], pose_b.quaternion[3]);
                    fflush(file);

                    geometry_msgs::PoseStamped p;
                    p.header.frame_id = "map";
                    p.header.stamp = image_msg->header.stamp;
                    p.pose.position.x = pose_b.translation[0];
                    p.pose.position.y = pose_b.translation[1];
                    p.pose.position.z = pose_b.translation[2];
                    p.pose.orientation.x = pose_b.quaternion[0];
                    p.pose.orientation.y = pose_b.quaternion[1];
                    p.pose.orientation.z = pose_b.quaternion[2];
                    p.pose.orientation.w = pose_b.quaternion[3];

                    traj_msg.poses.push_back(p);
                    pub_traj.publish(traj_msg);
                    pub_pose.publish(p);
                }
            }
            image_buf.pop();
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xrslam_ros");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                   ros::console::levels::Info);

    std::string slam_config_path, device_config_path, tum_output_path;
    n.getParam("slam_config_path", slam_config_path);
    n.getParam("device_config_path", device_config_path);
    n.getParam("trajectory_file", tum_output_path);

    int create_succ = XRSLAMCreate(
        slam_config_path.c_str(), device_config_path.c_str(), "", "XRSLAM ROS");
    std::cout << "create SLAM success: " << create_succ << std::endl;

    if (!(file = fopen(tum_output_path.c_str(), "w"))) {
        throw "Cannot open file";
    }

    traj_msg.header.frame_id = "map";
    
    ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 100, img_callback);
    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback,
                                          ros::TransportHints().tcpNoDelay());

    pub_traj = n.advertise<nav_msgs::Path>("traj", 1000);
    pub_pose = n.advertise<geometry_msgs::PoseStamped>("pose", 1000);

    std::thread measurement_process{process};
    ros::spin();
    return 0;
}
