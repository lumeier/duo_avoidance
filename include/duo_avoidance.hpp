/**
 *  duo_avoidance.hpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  Collision Avoidance for bebop with duo3d Cam
 */

#ifndef __DUO_AVOIDANCE_HPP__
#define __DUO_AVOIDANCE_HPP__

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ait_ros_messages/VioSensorMsg.h>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <ros/ros.h>

#include <stdio.h>

namespace duo_avoidance {

class DuoAvoidance {
public:
  DuoAvoidance();

private:

  ros::NodeHandle nh_;
  ros::Subscriber duo_message_sub_;
  ros::Publisher test_pub_;
  cv::Mat img_l_;
  cv::Mat img_r_;

//  ros::Publisher vio_pub_;
//  std::vector<sensor_msgs::Imu> imu_;
  ait_ros_messages::VioSensorMsg duo_msg_;
  sensor_msgs::Image test_output_;

  void duoMessageCb(const ait_ros_messages::VioSensorMsg::ConstPtr &msg);

};

} // namespace vio_bridge

#endif
