#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

image_transport::Publisher pub_cam_image_sync;
ros::Publisher pub_cam_info_sync;
ros::Publisher nav_fix_sync;


void cam_callback(const sensor_msgs::ImageConstPtr& cam_image, const sensor_msgs::CameraInfoConstPtr& cam_info)
{
   sensor_msgs::CameraInfo msg_cam_info_sync;
   sensor_msgs::Image msg_cam_image_sync;
   msg_cam_info_sync = *cam_info;
   msg_cam_info_sync.header.stamp = ros::Time::now();
   msg_cam_image_sync = *cam_image;
   msg_cam_image_sync.header.stamp = ros::Time::now();
   pub_cam_info_sync.publish(msg_cam_info_sync);
   pub_cam_image_sync.publish(msg_cam_image_sync);
}

void fix_callback(const sensor_msgs::NavSatFixConstPtr& gps)
{
   sensor_msgs::NavSatFix msg_gps_sync;
   msg_gps_sync = *gps;
   msg_gps_sync.header.stamp = ros::Time::now();
   nav_fix_sync.publish(msg_gps_sync);
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "logger_sync");
   ros::NodeHandle node;
   image_transport::ImageTransport it(node);
   pub_cam_image_sync = it.advertise("/cam3/image_raw_sync",1);
   pub_cam_info_sync = node.advertise<sensor_msgs::CameraInfo>("/cam3/camera_info_sync",1);
   nav_fix_sync = node.advertise<sensor_msgs::NavSatFix>("/fix_sync",1);

   //message_filters::Subscriber<sensor_msgs::Image> image_sub(node, "/cam3/image_raw", 1);
   //message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(node, "/cam3/camera_info", 1);
   message_filters::Subscriber<sensor_msgs::Image> image_sub(node, "/flea3_cam0/image_raw", 1);
   message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(node, "/flea3_cam0/camera_info", 1);
   typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;
   Synchronizer<MySyncPolicy> sync(MySyncPolicy(2), image_sub, info_sub);
   sync.registerCallback(boost::bind(&cam_callback, _1, _2));

   ros::Subscriber gps_sub = node.subscribe("/fix",1,fix_callback);
   
   ros::spin();

   return 0;
}

