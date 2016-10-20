
#include "ImageConverter.h"

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

ofstream csv_file;

cv::Mat image_bgr;
cv::Mat image;
cv::Mat image_hsv;
cv::Mat image_gray;
int MAX_MAP_SIZE = 256*256*256+256*256+256;
Mat sky_map= cv::Mat::zeros(MAX_MAP_SIZE,1,CV_8U); 
int err=0;
cv::Mat image1,image2,image3,image4,image5,image6,image7,image_l,image_ll,image_lll;
//vector<Mat> channels;
int c=0;
// Do our own spin loop
int SIGNAL_caught = 0;
void my_handler(int s){
 printf("Caught signal %d\n",s);
 SIGNAL_caught = 1;
 sleep(1);
}



ImageConverter::ImageConverter(ros::NodeHandle node):
       nh_(node),
	it_(nh_),
    	gps_msg(nh_, "/fix_sync",2),
       	msg_info(nh_, "cam3/camera_info_sync", 2),
        msg_im( nh_, "cam3/image_raw_sync", 2),
        sync( MySyncPolicy(2), msg_im,msg_info,gps_msg ) 
      {
        this->name = "ImageConverter";
        init();
      }

int ImageConverter::init() {

  ros::NodeHandle nh_param("~");
  this->name = "ImageConverter";

  this->sync.registerCallback( boost::bind( &ImageConverter::cam3Callback, this, _1, _2,_3) );
  image_transport::ImageTransport it(nh_);
  image_pub = it.advertise("/image_conversion/image", 1);
  leaf_ratio_pub=nh_.advertise<imageconverter::leaf_ratio>("/imageconverter/leaf_ratio",1);
  
  return 0;
}

//--------------------------------------------------------------------------------------------------------

void ImageConverter::cam3Callback(const sensor_msgs::ImageConstPtr& msg_im, const sensor_msgs::CameraInfoConstPtr& msg_info, const sensor_msgs::NavSatFixConstPtr& gps_msg) 
  {
	//fprintf(stdout, "Callback \n");
	imageconverter::leaf_ratio msg2;	
	image=convertMsgToImage(msg_im);
        std::string calib_file;
	nh_.getParam("/calib_file",calib_file);
	
	image=fisheye_undistort(image, calib_file);
        
	//fprintf(stdout, "Undistorted \n");
	  
	
	int i=0;
	int j=0;
        
	int f;
	
        
         //fprintf(stdout, "%d\n",MAX_MAP_SIZE);
	
	//fprintf(stdout, "%d %d\n",image.cols,image.rows);
	if (c==0)   //For first image only
 	{       

            int min_i = 0;
            int max_i =2448;
            int min_j = 0; 
            int max_j = 2048;
       
             for (i=min_i;i<max_i;i++)
    	        for (j=min_j;j<max_j;j++) {
                    int map_index =(int) (256*256*1.417*image.at<Vec3b>(j,i)[0]) + 256*image.at<Vec3b>(j,i)[1] + image.at<Vec3b>(j,i)[2];
                    sky_map.at<unsigned char>(map_index,0) = true;
			 }
             
              
       // fprintf(stdout, "Shade count %f\n",cv::sum(sky_map)[0]);
        	  

	}
	
	//Comparing it with sky map
	if(c==1)
            {
             for (i=0;i<image.cols;i++)
    	        for (j=0;j<image.rows;j++)
	        {	
                 int map_index =(int)(256*256*1.417* image.at<Vec3b>(j,i)[0]) + 256*image.at<Vec3b>(j,i)[1] + image.at<Vec3b>(j,i)[2];
	       int map_index_1 =(int)(256*256*1.417* (image.at<Vec3b>(j,i)[0]+1)) + 256*(image.at<Vec3b>(j,i)[1]+1) + image.at<Vec3b>(j,i)[2]+1;
                int map_index_2 =(int)(256*256*1.416* image.at<Vec3b>(j,i)[0]-1) + 256*(image.at<Vec3b>(j,i)[1]-1) + image.at<Vec3b>(j,i)[2]-1;
	int map_index_3 =(int)(256*256*1.417* (image.at<Vec3b>(j,i)[0]+2)) + 256*(image.at<Vec3b>(j,i)[1]+2) + image.at<Vec3b>(j,i)[2]+2;
                    int map_index_4=(int)(256*256*1.416* image.at<Vec3b>(j,i)[0]-2) + 256*(image.at<Vec3b>(j,i)[1]-2) + image.at<Vec3b>(j,i)[2]-2;
                      if(sky_map.at<unsigned char>(map_index,0)||sky_map.at<unsigned char>(map_index_1,0)||sky_map.at<unsigned char>(map_index_2,0)||sky_map.at<unsigned char>(map_index_3,0) ||sky_map.at<unsigned char>(map_index_4,0)) 
				{//Pixel is sky			
		                image.at<Vec3b>(j,i)[0]=180;
				image.at<Vec3b>(j,i)[1]=255;
				image.at<Vec3b>(j,i)[2]=255;
				
		
                       }
		       else {   image.at<Vec3b>(j,i)[0]=0;
				image.at<Vec3b>(j,i)[1]=0;
				image.at<Vec3b>(j,i)[2]=0;
		
				}
		  }
			 }
	
	   c=1;
	
        
//--------------------------------------------------------------------------------------------------------------------
	
	cv::cvtColor(image, image, CV_HSV2BGR);
        //fprintf(stdout, "HSV_BGR\n");
	 
	 cvtColor(image,image_gray,CV_BGR2GRAY);
	
	 
	 cv::circle(image_gray,cv::Point(image.cols/2, image.rows/2), 250, CV_RGB(255,0,0));
         cv::circle(image_gray,cv::Point(image.cols/2, image.rows/2), 500, CV_RGB(255,0,0));
         cv::circle(image_gray,cv::Point(image.cols/2, image.rows/2), 750, CV_RGB(255,0,0));
	
        sensor_msgs::ImagePtr im_msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_gray).toImageMsg();
	float leaf_ratio[3]={0,0,0};
	
	 

	cv::Rect r_a(774,518,500,500);
	cv::Mat roi_a=image_gray(r_a);
	cv:Mat mask_a(roi_a.size(), roi_a.type(),Scalar::all(0));
	circle(mask_a, Point(mask_a.cols/2,mask_a.rows/2),250, Scalar::all(255),-1);
	cv::Mat cropped_a = roi_a & mask_a;
	
	cv::Rect r_b(524,268,1000,1000);
	cv::Mat roi_b=image_gray(r_b);
	cv::Mat mask_b(roi_b.size(), roi_b.type(),Scalar::all(0));
	circle(mask_b, Point(mask_b.cols/2,mask_b.rows/2),500, Scalar::all(255),-1);
	cv::Mat cropped_b = roi_b & mask_b;
	

        cv::Rect r_c(274,18,1500,1500);
	cv::Mat roi_c=image_gray(r_c);
	cv::Mat mask_c(roi_c.size(), roi_c.type(),Scalar::all(0));
	circle(mask_c, Point(mask_c.cols/2,mask_c.rows/2),750, Scalar::all(255),-1);
	cv::Mat cropped_c = roi_c & mask_c;

        float Totalpixels_a=0.78529*cropped_a.rows*cropped_a.cols;
	float ZeroPixels_a=Totalpixels_a-countNonZero(cropped_a);
	 leaf_ratio[1]= 100*ZeroPixels_a/Totalpixels_a;
        float Totalpixels_b=0.78529*cropped_b.rows*cropped_b.cols;
	float ZeroPixels_b=Totalpixels_b-countNonZero(cropped_b);
	 leaf_ratio[2]= 100*ZeroPixels_b/Totalpixels_b;
        float Totalpixels_c=0.78529*cropped_c.rows*cropped_c.cols;
	float ZeroPixels_c=Totalpixels_c-countNonZero(cropped_c);
          leaf_ratio[3]= 100*ZeroPixels_c/Totalpixels_c; 

			
	double latitude =  gps_msg->latitude;
    	double longitude =  gps_msg->longitude;
    	double altitude = gps_msg->altitude;

 	if(isnan(altitude)) {
	     altitude = 0;
	    }

	geographic_msgs::GeoPoint geopoint = geodesy::toMsg (latitude, longitude, altitude); 	
	geodesy::UTMPoint utmpoint = geodesy::UTMPoint(geopoint);

	msg2.leaf_ratio_1=leaf_ratio[1];
	msg2.leaf_ratio_2=leaf_ratio[2];
	msg2.leaf_ratio_3=leaf_ratio[3];
	msg2.header.stamp = msg_info->header.stamp;
	msg2.header.frame_id = msg_info->header.frame_id;
	leaf_ratio_pub.publish(msg2);
	image_pub.publish(im_msg);
      
  }
//----------------------------------------------------------------------------------------------      
    cv::Mat ImageConverter::convertMsgToImage(const sensor_msgs::Image::ConstPtr& msg)
  {
      cv::Mat image_a;
  
    try 
      {image_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::cvtColor(image_bgr,image_a, CV_BGR2HSV);
	}
    catch(cv_bridge::Exception) 
          { ROS_ERROR("%s. Unable to convert %s image to gray", this->name.c_str(), msg->encoding.c_str());}
  
    if (image_bgr.empty()) 
          {ROS_ERROR("%s. No image data!", this->name.c_str());}
	
	return image_a;     

  }


//---------------------------------------------------------------------------------------------------------------

      int main(int argc, char **argv) 
     {
	ros::init(argc, argv, "ImageConverter");
	ros::NodeHandle node; 

	//catch SIGINT
	 struct sigaction sigIntHandler;
	 sigIntHandler.sa_handler = my_handler;
	 sigemptyset(&sigIntHandler.sa_mask);
	 sigIntHandler.sa_flags = 0;
	 sigaction(SIGINT, &sigIntHandler, NULL);
	
	 ImageConverter *imageConverter;
         imageConverter = new ImageConverter(node);

	  while (ros::ok() && !SIGNAL_caught)
	 {
	   ros::spinOnce();
	 
	   usleep(100000);
	 }

	 //fprintf(stderr, "Shutting down\n");
	 ros::shutdown();
	 sleep(1.0);

	 return 0;	
     }





