/**
 *  duo_avoidance.hpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  Collision Avoidance for bebop with duo3d Cam
 */

#ifndef __DUO_AVOIDANCE_HPP__
#define __DUO_AVOIDANCE_HPP__

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <ait_ros_messages/VioSensorMsg.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>



#include <ros/ros.h>

#include <stdio.h>
#include <string.h>

namespace duo_avoidance {

class DuoAvoidance {
public:
  DuoAvoidance();

private:

  ros::NodeHandle nh_;
  ros::Subscriber disparity_message_sub_;
  ros::Publisher test_pub1_;
  ros::Publisher test_pub2_;
  cv::Mat disparity_img_;
  cv::Mat depth_img_;
  cv::Size size_cropped_;
  cv::Mat binned_img_;
float focal_length_;
float baseline_;
float command_x_;
float command_y_;

//  ros::Publisher vio_pub_;
//  std::vector<sensor_msgs::Imu> imu_;
//  ait_ros_messages::VioSensorMsg duo_msg_;
  sensor_msgs::Image disparity_msg_;
  sensor_msgs::Image test_output1_;
  sensor_msgs::Image test_output2_;

  void disparityMessageCb(const stereo_msgs::DisparityImage::ConstPtr &msg);

};

} // namespace duo_avoidance

#endif
