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

  test_pub_ = nh_.advertise<sensor_msgs::Image>("/test_left_image", 1);
}

void DuoAvoidance::duoMessageCb(const ait_ros_messages::VioSensorMsg::ConstPtr &msg) {

ait_ros_messages::VioSensorMsg duo_msg_  = *msg;
printf("Received message from Duo3d-Cam. Timestamp: %d.%d\n",duo_msg_.header.stamp.sec,duo_msg_.header.stamp.nsec);

//Use cv_bridge to transform images to cv::Mat
cv::Mat img_l_=cv_bridge::toCvCopy(duo_msg_.left_image,"mono8")->image;
cv::Mat img_r_=cv_bridge::toCvCopy(duo_msg_.right_image,"mono8")->image;

cv_bridge::CvImage cv_out;
cv_out.image=img_l_;
cv_out.encoding="mono8";

cv_out.toImageMsg(test_output_);
test_pub_.publish(test_output_);

//Republish left image


  // sensor_msgs::Image left = *msg;
  // vio_msg_.header = left.header;
  // vio_msg_.left_image = left;
  // vio_pub_.publish(vio_msg_);
}

} // namespace vio_bridge
