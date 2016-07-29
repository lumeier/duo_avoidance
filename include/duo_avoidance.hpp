/**
 *  duo_avoidance.hpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  Collision Avoidance for bebop with duo3d Cam
 */

#ifndef __DUO_AVOIDANCE_HPP__
#define __DUO_AVOIDANCE_HPP__

#include <sensor_msgs/Image.h>
//#include <sensor_msgs/Imu.h>
#include <sensor_msgs/CameraInfo.h>

#include <ait_ros_messages/VioSensorMsg.h>

#include <ros/ros.h>
#include <vector>
#include <stdio.h>

namespace duo_avoidance {

class DuoAvoidance {
public:
  DuoAvoidance();

private:

  ros::NodeHandle nh_;
  ros::Subscriber duo_message_sub_;

//  ros::Publisher vio_pub_;
//  std::vector<sensor_msgs::Imu> imu_;
  ait_ros_messages::VioSensorMsg duo_msg_;

  void duoMessageCb(const ait_ros_messages::VioSensorMsg::ConstPtr &msg);

};

} // namespace vio_bridge

#endif
