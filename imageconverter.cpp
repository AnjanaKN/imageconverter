
#include "ImageConverter.h"

#define get_ticks cv::getTickCount
#define get_freq  cv::getTickFrequency
double dt(int64 t) { return double(t*1000/get_freq())/1000.0; }

struct Profile
{
    string name;
    int64 t; // accumulated time
    int64 c; // function calls

    Profile(const string & name) 
        : name(name)
        , t(0) 
        , c(0)
    {}   

    ~Profile() 
    {
        cerr << format("%-24s %8u ",name.c_str(),c);
        cerr << format("%13.6f ",dt(t/c)); 
        cerr << format("%13.6f ",dt(t));
        cerr << format("%14u",t);
        cerr << endl;
    }

    struct Scope
    {
        Profile & p;
        int64 t;

        Scope(Profile & p) 
            : p(p) 
            , t(get_ticks()) 
        {}

        ~Scope() 
        { 
            int64 t1 = get_ticks();
            if ( t1 > t )
            {
                p.t += t1 - t;
                p.c ++;
            }
         }
    }; 
};

#define PROFILEX(s) static Profile _a_rose(s); Profile::Scope _is_a_rose_is(_a_rose);
#define PROFILE PROFILEX(__FUNCTION__)

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

ofstream csv_file;

cv::Mat image_bgr,image, image_hsv,image_gray,image_a;

int MAX_MAP_SIZE = 256*256*256+256*256+256;
int co=0;
int x_min, x_max, y_min,y_max; 

Mat sky_map= cv::Mat::zeros(MAX_MAP_SIZE,1,CV_8U); 


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
    	gps_msg(nh_, "/fix_sync",1),
       	msg_info(nh_, "cam3/camera_info_sync", 2),
        msg_im( nh_, "cam3/image_raw_sync", 2),
        sync( MySyncPolicy(10), msg_im,msg_info,gps_msg ) 
      {
        this->name = "ImageConverter";
        init();
      }

int ImageConverter::init() {

  ros::NodeHandle nh_param("~");
  this->name = "ImageConverter";

  this->sync.registerCallback( boost::bind( &ImageConverter::cam3Callback, this, _1, _2,_3) );
  image_transport::ImageTransport it(nh_);
  image_pub = it.advertise("/image_conversion/image", 2);
  image_undistorted = it.advertise("/undistorted/image", 2);
  leaf_ratio_pub=nh_.advertise<imageconverter::leaf_ratio>("/imageconverter/leaf_ratio",2);
  
  return 0;
}


//-------------------------------------------------------------------------------------------------------
 
 void ImageConverter::on_trackbar(int, void*)
  {
        
        rectangle(image,Point(x_min,y_min), Point(x_max,y_max),Scalar(0,0,0),1,8, 0 );
	fprintf(stdout,"on_trackbar");
    
        imshow("Choose_Sky", image);
	fprintf(stdout,"-------%d>x>%d     %d>y>%d------------\n",x_max,x_min,y_max,y_min);
  }


 void ImageConverter::createTrackbars()
   {
  
   	namedWindow("Choose_Sky",0);       
   	createTrackbar("X_MIN",OPENCV_WINDOW,&x_min,200,on_trackbar); 
   	createTrackbar("X_MAX",OPENCV_WINDOW,&x_max,2400,on_trackbar);
   	createTrackbar("Y_MIN",OPENCV_WINDOW,&y_min,2000,on_trackbar);
   	createTrackbar("Y_MAX",OPENCV_WINDOW,&y_max,2048,on_trackbar); 
  

   }



//--------------------------------------------------------------------------------------------------------

 void ImageConverter::cam3Callback(const sensor_msgs::ImageConstPtr& msg_im, const sensor_msgs::CameraInfoConstPtr& msg_info, const sensor_msgs::NavSatFixConstPtr& gps_msg) 
  { PROFILE;

	imageconverter::leaf_ratio msg2;	
	image=convertMsgToImage(msg_im);
        std::string calib_file;
	nh_.getParam("/calib_file",calib_file);
	
	image=fisheye_undistort(image, calib_file);
        sensor_msgs::ImagePtr im_msg_2 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg(); 
	
	  

	
	int i=0;
	int j=0;
       
	if (co==0)   						//Creation of Map:For first image only
 	{      	
		x_min =10;
		x_max = 2400;
		y_min = 20;
 		y_max = 2048;
		createTrackbars();
		on_trackbar(0, 0);
	       
                waitKey();
		PROFILEX("Creating map");
            for (i=x_min;i<x_max;i++)
    	        for (j=y_min;j<y_max;j++) 
                   {
			 int map_index =(int) (256*256*1.417*image.at<Vec3b>(j,i)[0]) + 256*image.at<Vec3b>(j,i)[1] + image.at<Vec3b>(j,i)[2];
	                 sky_map.at<unsigned char>(map_index,0) = true;
		    }
     
       	      fprintf(stdout, "--------------------------Shade count------------ %f\n",cv::sum(sky_map)[0]);
        
	}
	
	
	
	
	if(co)									//Comparing it with sky map
            {    PROFILEX("Comparing with map");
		//chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
		
		for (i=0;i<image.cols;i++)
    	        for (j=0;j<image.rows;j++)
	        {	
                int map_index_1 =(int)(256*256*1.417* image.at<Vec3b>(j,i)[0]) + 256*image.at<Vec3b>(j,i)[1] + image.at<Vec3b>(j,i)[2]; 
			     



         /*   image.at<Vec3b>(j,i)[0]=image.at<Vec3b>(j,i)[0]-1;
	        //  image.at<Vec3b>(j,i)[1]=image.at<Vec3b>(j,i)[1]-1;
		//image.at<Vec3b>(j,i)[2]=image.at<Vec3b>(j,i)[2]-1;
        	//int map_index_2 =(int)(256*256*1.416* image.at<Vec3b>(j,i)[0]) + 256*(image.at<Vec3b>(j,i)[1]) + image.at<Vec3b>(j,i)[2];


	   	/*absdiff(image.at<Vec3b>(j,i), 1, image.at<Vec3b>(j,i)); 
        	int map_index_3 =(int)(256*256*1.416* image.at<Vec3b>(j,i)[0]) + 256*(image.at<Vec3b>(j,i)[1]) + image.at<Vec3b>(j,i)[2];
	   	absdiff(image.at<Vec3b>(j,i), 1, image.at<Vec3b>(j,i)); 
        	int map_index_4 =(int)(256*256*1.416* image.at<Vec3b>(j,i)[0]) + 256*(image.at<Vec3b>(j,i)[1]) + image.at<Vec3b>(j,i)[2];
	   	//fprintf(stdout,"--------");
       		add(image, 4, image);
                int map_index_5 =(int)(256*256*1.417* image.at<Vec3b>(j,i)[0]) + 256*(image.at<Vec3b>(j,i)[1]) + image.at<Vec3b>(j,i)[2];

       		add(image, 1, image);
                int map_index_6 =(int)(256*256*1.417* image.at<Vec3b>(j,i)[0]) + 256*(image.at<Vec3b>(j,i)[1]) + image.at<Vec3b>(j,i)[2];

		add(image, 1, image);
     	        int map_index_7 =(int)(256*256*1.417* image.at<Vec3b>(j,i)[0]) + 256*(image.at<Vec3b>(j,i)[1]) + image.at<Vec3b>(j,i)[2];			
		//||sky_map.at<unsigned char>(map_index_2,0))
		//||sky_map.at<unsigned char>(map_index_3,0)||sky_map.at<unsigned char>(map_index_4,0))
		//||sky_map.at<unsigned char>(map_index_5,0)||sky_map.at<unsigned char>(map_index_6,0)||sky_map.at<unsigned char>(map_index_7,0))
		*/





	if(sky_map.at<unsigned char>(map_index_1,0))

			{		
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
	
	   co=1;
	//chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
	//std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() <<std::endl;
             
//--------------------------------------------------------------------------------------------------------------------
	
        cvtColor(image, image, CV_HSV2BGR);	 
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
      image_undistorted.publish(im_msg_2);
  }
//----------------------------------------------------------------------------------------------      
  cv::Mat ImageConverter::convertMsgToImage(const sensor_msgs::Image::ConstPtr& msg)
  {      PROFILE;
    try 
      {
	image_bgr = cv_bridge::toCvShare(msg, "bgr8")->image;
	cv::cvtColor(image_bgr,image_a, CV_BGR2HSV);
       }

    catch(cv_bridge::Exception) 
      { 
	ROS_ERROR("%s. Unable to convert %s image to gray", this->name.c_str(), msg->encoding.c_str());
      }
  
    if (image_bgr.empty()) 
      {
	ROS_ERROR("%s. No image data!", this->name.c_str());
       }
	
	return image_a;     

  }


//---------------------------------------------------------------------------------------------------------------

  int main(int argc, char **argv) 
     {    PROFILE;
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
	 {    PROFILEX("while in main");
	   ros::spinOnce();
	 
	   usleep(100000);
	 }

	 //fprintf(stderr, "Shutting down\n");
	 ros::shutdown();
	 sleep(1.0);

	 return 0;	
     }





