/**
 *  duo_avoidance.cpp
 *  author: Lukas Meier <lukasdanmeier@gmail.com>
 *  Collison Avoidance for Parrot Bebop with Duo3d Caml
 */

#include "duo_avoidance.hpp"

namespace duo_avoidance {

DuoAvoidance::DuoAvoidance() : nh_("~") {
  disparity_message_sub_ = nh_.subscribe<stereo_msgs::DisparityImage>(
      "/duo3d_camera/disparity", 2, &DuoAvoidance::disparityMessageCb, this);

  test_pub1_ = nh_.advertise<sensor_msgs::Image>("/test_left_image", 1);
  test_pub2_ = nh_.advertise<sensor_msgs::Image>("/test_right_image", 1);
  printf("Started sending!!\n");
}

void DuoAvoidance::disparityMessageCb(const stereo_msgs::DisparityImage::ConstPtr &msg) {

stereo_msgs::DisparityImage disparity_msg_  = *msg;
//printf("Received message from Duo3d-Cam. Timestamp: %d.%d\n",disparity_msg_.header.stamp.sec,duo_msg_.header.stamp.nsec);

//Use cv_bridge to transform images to cv::Mat
cv::Mat disparity_img_=cv_bridge::toCvCopy(disparity_msg_.image,"32FC1")->image;
focal_length_=disparity_msg_.f;
baseline_=disparity_msg_.T;

//Crop image to square to remove black boarder from block matching
cv::Rect ROI(43,20,234,201);
disparity_img_=disparity_img_(ROI);
cv::Size size_cropped_ =disparity_img_.size();

//cv::Scalar mean_value = cv::mean(disparity_img_);
//cv::Mat mean_img(mean_value);
//printf("mean value: %f\n",mean_value[0]);

//calculate depth

//binning
int n_bins_x=6;
int n_bins_y=3;

int bin_start_x=0;
int bin_start_y=0;
int avg_count=0;
float avg_cumsum=0;
int bin_size_x=size_cropped_.width/n_bins_x;
int bin_size_y=size_cropped_.height/n_bins_y;
float pix_val=0;
float mean_val=0;
//printf("binx %d biny: %d\n",bin_size_x,bin_size_y);
cv::Mat bin_img(n_bins_y,n_bins_x,CV_32FC1);


for (int bin_x=0;bin_x<n_bins_x;bin_x++)
{	 
 for(int bin_y=0;bin_y<n_bins_y;bin_y++)
   {

	avg_count=0;
	avg_cumsum=0;
	for(int i_x=0;i_x<bin_size_x;i_x++)
	{
	for(int i_y=0;i_y<bin_size_y;i_y++)
	{	//printf("i_x: %d i_y: %d\n",i_x,i_y);
		pix_val=disparity_img_.at<float>(bin_y*bin_size_y+i_y,bin_x*bin_size_x+i_x);
		//pix_val=disparity_img_.at<float>(50,50);
		//pix_val=5;		
		if(pix_val>-0.5)
		{
		//printf("i_x: %d i_y: %d\n",i_x,i_y);
		avg_count++;
		avg_cumsum=avg_cumsum+pix_val;
		}
	}
	}

	if(avg_count>0){	
	mean_val=avg_cumsum/(avg_count*1.0);
	bin_img.at<float>(bin_y,bin_x)=focal_length_*baseline_/mean_val;
	}
    }
}

float x_command_sum=0;

for (int i=1;i<n_bins_x-1;i++)
{
x_command_sum=x_command_sum+bin_img.at<float>(1,i);
}
command_x_=2-(1./(n_bins_x-2)*x_command_sum);

if (command_x_>1.1)
	command_x_=1.1;
else if (command_x_<0)
	command_x_=0;

command_y_=2-(bin_img.at<float>(1,0)+bin_img.at<float>(1,1)-bin_img.at<float>(1,4)-bin_img.at<float>(1,5))/5.;



printf("command x: %f command y: %f depth: %f m\n",command_x_,command_y_,bin_img.at<float>(1,3));
//cv::Size img_size =img_l_.size();


//Convert to 8bit
//left_disp.convertTo(left_disp,0);


// //Debug: Republish left image
 cv_bridge::CvImage cv_out1;
 cv_out1.image=disparity_img_;
 cv_out1.encoding="32FC1";
 cv_out1.toImageMsg(test_output1_);
 test_pub1_.publish(test_output1_);

 cv_bridge::CvImage cv_out2;
 cv_out2.image=bin_img;
 cv_out2.encoding="32FC1";
 cv_out2.toImageMsg(test_output2_);
test_pub2_.publish(test_output2_);

}

} // namespace duo_avoidance
