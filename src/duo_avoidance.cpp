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

  test_pub1_ = nh_.advertise<sensor_msgs::Image>("/test_left_image", 1);
  test_pub2_ = nh_.advertise<sensor_msgs::Image>("/test_right_image", 1);
}

void DuoAvoidance::duoMessageCb(const ait_ros_messages::VioSensorMsg::ConstPtr &msg) {

ait_ros_messages::VioSensorMsg duo_msg_  = *msg;
printf("Received message from Duo3d-Cam. Timestamp: %d.%d\n",duo_msg_.header.stamp.sec,duo_msg_.header.stamp.nsec);

//Use cv_bridge to transform images to cv::Mat
cv::Mat img_l_=cv_bridge::toCvCopy(duo_msg_.left_image,"mono8")->image;
cv::Mat img_r_=cv_bridge::toCvCopy(duo_msg_.right_image,"mono8")->image;


cv::Mat left_disp;
cv::Mat right_disp;
cv::Mat left;
cv::Mat right;
left  = img_l_.clone();
right = img_r_.clone();
cv::Size img_size =img_l_.size();

//Camera Calibration Parameters of left Camera
cv::Mat distortion_cam1(1,5,CV_64FC1,cvScalar(0.));
distortion_cam1.at<double>(0,0)=-0.4186;
distortion_cam1.at<double>(0,1)=0.2771;
distortion_cam1.at<double>(0,4)=-0.1375;

cv::Mat intrinsics_cam1(3,3,CV_64FC1,cvScalar(0.));
intrinsics_cam1.at<double>(0,0)=269.181725;	//fx Focal Length
intrinsics_cam1.at<double>(1,1)=269.9321;	//fy Focal Length
intrinsics_cam1.at<double>(0,2)=156.3816;	//x0 Principal Point
intrinsics_cam1.at<double>(1,2)=113.9369;	//y0 Principal Point
intrinsics_cam1.at<double>(2,2)=1.;

//Camera calibration parameters of right Camera
cv::Mat distortion_cam2(1,5,CV_64FC1,cvScalar(0.));
distortion_cam2.at<double>(0,0)=-0.4233;
distortion_cam2.at<double>(0,1)=0.2967;
distortion_cam2.at<double>(0,4)=-0.1663;

cv::Mat intrinsics_cam2(3,3,CV_64FC1,cvScalar(0.));
intrinsics_cam2.at<double>(0,0)=270.1185;	//fx Focal Length
intrinsics_cam2.at<double>(1,1)=270.7983;	//fy Focal Length
intrinsics_cam2.at<double>(0,2)=163.4106;	//x0 Principal Point
intrinsics_cam2.at<double>(1,2)=120.1803;	//y0 Principal Point
intrinsics_cam2.at<double>(2,2)=1.;

//Rotation Matrix R_lr
cv::Mat R_lr(3,3,CV_64FC1,cvScalar(0.));
R_lr.at<double>(0,0)=1.;	R_lr.at<double>(0,1)=0.0018258;	R_lr.at<double>(0,2)=0.0028920;
R_lr.at<double>(1,0)=-0.0018415;R_lr.at<double>(1,1)=1.;	R_lr.at<double>(1,2)=0.005452;
R_lr.at<double>(2,0)=0.0028820;	R_lr.at<double>(2,1)=-0.0054579;R_lr.at<double>(2,2)=1.;

//Translation r_lr
cv::Mat r_lr(3,1,CV_64FC1,cvScalar(0.));
r_lr.at<double>(0,0)=30.5341;
r_lr.at<double>(1,0)=-0.2507;
r_lr.at<double>(2,0)=-1.6489;



//Define StereoRectify Outputs
cv::Mat R1(3,3,CV_64FC1,cvScalar(0.));
cv::Mat R2(3,3,CV_64FC1,cvScalar(0.));
cv::Mat P1(3,4,CV_64FC1,cvScalar(0.));
cv::Mat P2(3,4,CV_64FC1,cvScalar(0.));
cv::Mat Q(4,4,CV_64FC1,cvScalar(0.));

cv::Rect validRoi[2];

//Calculate the Stereo Rectification
cv::stereoRectify(	intrinsics_cam1,distortion_cam1,
			intrinsics_cam2,distortion_cam2,
			img_size, R_lr.inv(), r_lr, R1, R2, P1, P2, Q,
			cv::CALIB_ZERO_DISPARITY, 1, img_size, &validRoi[0], &validRoi[1]);

//Prepare remapping & remap (undistort)
cv::Mat rmap[2][2];
cv::initUndistortRectifyMap(intrinsics_cam1, distortion_cam1, R1, P1, img_size, CV_16SC2, rmap[0][0], rmap[0][1]);
cv::initUndistortRectifyMap(intrinsics_cam2, distortion_cam2, R2, P2, img_size, CV_16SC2, rmap[1][0], rmap[1][1]);

remap(img_l_,left,rmap[0][0],rmap[0][1],cv::INTER_LINEAR);
remap(img_r_,right,rmap[1][0],rmap[1][1],cv::INTER_LINEAR);

//crop_img to be rectangular again and resize to initial size
cv::Mat cropped_left=left(validRoi[0]);
cv::Mat cropped_right=right(validRoi[1]);
cv::Mat resized_left;
cv::Mat resized_right;
cv::resize(cropped_left,resized_left,img_size,0,0,cv::INTER_AREA);
cv::resize(cropped_right,resized_right,img_size,0,0,cv::INTER_AREA);


//append Image on left side to not get cropped result
cv::Mat img_l_app(img_size.height,img_size.width+90,0,cvScalar(0.));
cv::Mat img_r_app(img_size.height,img_size.width+90,0,cvScalar(0.));
img_l_.copyTo(img_l_app(cv::Rect(90,0,img_l_.cols,img_l_.rows)));
img_r_.copyTo(img_r_app(cv::Rect(90,0,img_r_.cols,img_r_.rows)));

//Calculate disparity
int max_disp=64;
int wsize=25;
cv::StereoBM sbm(cv::StereoBM::BASIC_PRESET,max_disp,wsize);
sbm(resized_left,resized_right,left_disp,CV_16S);
sbm(img_l_app,img_r_app,left_disp,CV_16S);
left_disp=left_disp(cv::Rect(90,0,img_r_.cols,img_r_.rows));

//Alternatively calculation with stereSGBM (Semi Global Block Matching) but 10x more expensive
//cv::StereoSGBM sbm;
//sbm.SADWindowSize = 15;
//sbm.numberOfDisparities = 64;
//sbm.preFilterCap = 63;
//sbm.minDisparity = -2;
//sbm.uniquenessRatio= 30;
//sbm.speckleWindowSize = 100;
//sbm.speckleRange = 32;
//sbm.disp12MaxDiff =1;
//sbm.fullDP=false;
//sbm.P1=100;
//sbm.P2=200;
//sbm(resized_left,resized_right,left_disp);


//Convert to 8bit
left_disp.convertTo(left_disp,0);


// //Debug: Republish left image
 cv_bridge::CvImage cv_out1;
 cv_out1.image=img_l_app;
 cv_out1.encoding="mono8";
 cv_out1.toImageMsg(test_output1_);
 test_pub1_.publish(test_output1_);

 cv_bridge::CvImage cv_out2;
 cv_out2.image=left_disp;
 cv_out2.encoding="mono8";
 cv_out2.toImageMsg(test_output2_);
 test_pub2_.publish(test_output2_);

}

} // namespace duo_avoidance
