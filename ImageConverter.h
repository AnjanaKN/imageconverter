#ifndef __DETECTOR_NODE__
#define __DETECTOR_NODE__
#include <signal.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <std_msgs/String.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>
#include <ocam_functions.h>

#include <geodesy/utm.h>

#include <geodesy/wgs84.h>



#include <sstream>
#include "imageconverter/leaf_ratio.h"



using namespace std;
using namespace cv;


static const std::string OPENCV_WINDOW = "Image window";
cv::Mat fisheye_undistort(cv::Mat image,std::string calib_file);


class ImageConverter {
public:
  //---------------------------------------------------------------------------
  ImageConverter(ros::NodeHandle node);
  int finish();
  int init();
  void upwardCameraInit();

 

  void cam3Callback(const sensor_msgs::ImageConstPtr& msg_im, const sensor_msgs::CameraInfoConstPtr& msg_info, const      sensor_msgs::NavSatFixConstPtr& gps_msg);
  cv::Mat convertMsgToImage(const sensor_msgs::Image::ConstPtr& msg );
  

private:

  string name;

  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub;
  image_transport::Publisher image_pub;
  ros::Publisher leaf_ratio_pub;
  message_filters::Subscriber<sensor_msgs::NavSatFix> gps_msg;
  message_filters::Subscriber< sensor_msgs::Image > msg_im;
  message_filters::Subscriber< sensor_msgs::CameraInfo > msg_info;


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::NavSatFix> MySyncPolicy;

  message_filters::Synchronizer< MySyncPolicy > sync;
};


int main(int argc, char **argv);

#endif
