
#include <ImageConverter.h>
#include <algorithm>
#include <vector>
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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/NavSatFix.h>
#include "imageconverter/leaf_ratio.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sstream>
#include <boost/thread.hpp>
#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <geodesy/utm.h>


using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ofstream csv_file;



//------------------------------------------------------------------------------------------------
void detector_callback( const sensor_msgs::CameraInfoConstPtr& cam_info, const sensor_msgs::NavSatFixConstPtr& gps_msg, const imageconverter::leaf_ratioConstPtr& msg2)
{
 

	double latitude =  gps_msg->latitude;
    	double longitude =  gps_msg->longitude;
    	double altitude = gps_msg->altitude;


	geographic_msgs::GeoPoint geopoint = geodesy::toMsg (latitude, longitude, altitude); 	
	geodesy::UTMPoint utmpoint = geodesy::UTMPoint(geopoint);

	csv_file << std::fixed <<cam_info->header.stamp.sec << ",";
    	cout << std::fixed <<cam_info->header.stamp.sec << ",";
	csv_file << cam_info->header.stamp.nsec << ",";
	cout << cam_info->header.stamp.nsec << ",";
        csv_file << std::fixed << std::setprecision(9)<< gps_msg->latitude << ",";
	cout << std::fixed << std::setprecision(9)<< gps_msg->latitude << ",";
	csv_file << std::fixed << std::setprecision(9)<< gps_msg->longitude << ",";
	cout << std::fixed << std::setprecision(9)<<gps_msg->longitude << ",";
	csv_file << std::fixed << std::setprecision(6)<< gps_msg->altitude << ",";
	cout << std::fixed << gps_msg->altitude << ",";
	csv_file <<std::fixed << utmpoint.easting << ",";
	cout <<std::fixed << utmpoint.easting << ",";
	csv_file << std::fixed << utmpoint.northing << ",";
	cout << std::fixed << utmpoint.northing << ",";
        
	csv_file << std::fixed << std::setprecision(4) << msg2->leaf_ratio_1 << ",";
	cout << std::fixed << std::setprecision(4) << msg2->leaf_ratio_1 << ",";

	csv_file << std::fixed << std::setprecision(4) << msg2->leaf_ratio_2 << ",";
	cout << std::fixed << std::setprecision(4) << msg2->leaf_ratio_2 << ",";
	
	csv_file << std::fixed << std::setprecision(4) << msg2->leaf_ratio_3 << endl;
	cout << std::fixed << std::setprecision(4) << msg2->leaf_ratio_3 << endl;






}

//------------------------------------------------------------------------------------------------
int main(int argc, char **argv)
{
  ros::init(argc, argv, "detector_logger");
  ros::NodeHandle nh;
  ros::NodeHandle nhp = ros::NodeHandle("~");
 
  
  string config_path = "/tmp/";
  nhp.getParam("config_path", config_path);
  fprintf(stderr, "The config path is : %s\n", config_path.c_str());

  message_filters::Subscriber<CameraInfo> cam_info(nh, "/cam3/camera_info_sync", 1);
  message_filters::Subscriber<NavSatFix> gps_msg(nh, "/fix_sync", 1);
  message_filters::Subscriber<imageconverter::leaf_ratio> msg2(nh, "/imageconverter/leaf_ratio", 1);

  typedef sync_policies::ApproximateTime<CameraInfo,NavSatFix,imageconverter::leaf_ratio> MySyncPolicy;
  // takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), cam_info, gps_msg,msg2);
  sync.registerCallback(boost::bind(&detector_callback, _1, _2, _3));

  string csv_filename = config_path+"/frc_vision_result_1.csv";
  csv_file.open (csv_filename.c_str());
  csv_file << "time_sec, time_nsec, gps_lat, gps_lon, gps_altitude, gps_northings, gps_eastings,leaf_ratio_1,leaf_ratio_2,leaf_ratio_3\n";
  cout << "time_sec, time_nsec,  gps_lat, gps_lon, gps_altitude,  gps_northings, gps_eastings, leaf_ratio_1,leaf_ratio_2,leaf_ratio_3\n";

  ros::spin();

  csv_file.close();
  return 0;
}
