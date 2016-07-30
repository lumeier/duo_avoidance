/**
 *  duo_avoidance.cpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  Collison Avoidance for Parrot Bebop with Duo3d Caml
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

// //Debug: Republish left image
// cv_bridge::CvImage cv_out;
// cv_out.image=img_r_;
// cv_out.encoding="mono8";
// cv_out.toImageMsg(test_output_);
// test_pub_.publish(test_output_);

//Calculate Disparity image in Opencv (camera Calibration has to be done, to get good results!!!!!!!)

int max_disp=500;
int wsize=21;
cv::Mat left_disp;
cv::Mat right_disp;
cv::Mat left;
cv::Mat right;
left  = img_l_.clone();
right = img_r_.clone();

printf("Opencv Version: %s\n",CV_VERSION);
cv::StereoBM *left_matcher=cv::StereoBM::create(max_disp,wsize);
left_matcher->setTextureThreshold(10);
left_matcher->setUniquenessRatio(3);


printf("Object created!!!\n");
left_matcher-> compute(left, right,left_disp);
printf("Disparity Calculated!!\n");





// cuda::Ptr<StereoBM> left_matcher = cuda::StereoBM::create(max_disp,wsize);
// cv::Ptr<DisparityWLSFilter> wls_filter;
// wls_filter = cv::createDisparityWLSFilter(left_matcher);
// cv::Ptr<StereoMatcher> right_matcher = cv::createRightMatcher(left_matcher);
//
// //matching_time = (double)getTickCount();
// left_matcher-> compute(img_l_, img_r_, left_disp);
// right_matcher->compute(img_r_, img_l_, right_disp);
//matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();



}

} // namespace duo_avoidance
