/**
 *  vio_bridge.cpp
 *  author: Simone Guscetti <simonegu@student.ethz.ch>
 *  simple bridge class for DUO VIO
 */

#include "duo_avoidance.hpp"

namespace duo_avoidance {

DuoAvoidance::DuoAvoidance() : nh_("~") {
  duo_message_sub_ = nh_.subscribe<ait_ros_messages::VioSensorMsg>(
      "/vio_sensor", 2, &DuoAvoidance::duoMessageCb, this);

  //vio_pub_ = nh_.advertise<ait_ros_messages::VioSensorMsg>("/vio_sensor", 1);
}

void DuoAvoidance::duoMessageCb(const ait_ros_messages::VioSensorMsg::ConstPtr &msg) {

ait_ros_messages::VioSensorMsg duo_msg_  = *msg;
printf("Received message from Duo3d-Cam. Timestamp: %d\n",duo_msg_.header.stamp.sec);


  // sensor_msgs::Image left = *msg;
  // vio_msg_.header = left.header;
  // vio_msg_.left_image = left;
  // vio_pub_.publish(vio_msg_);
}

} // namespace vio_bridge
