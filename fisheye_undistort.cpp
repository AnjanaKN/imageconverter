

#include "ocam_functions.h"
using namespace cv;
cv::Mat fisheye_undistort(cv::Mat ab,std::string calib_file)
{   
   //char * calib=calib_file;
  const char * calib = calib_file.c_str(); 
  struct ocam_model o, o_cata; 
  get_ocam_model(&o,"/home/anjana/vision/cam0.txt");
  IplImage src1 = ab;


  static IplImage* dst_persp;
  static CvMat* mapy_persp;
  static CvMat* mapx_persp;
  static bool first_run = true;
  if(first_run){
    dst_persp=cvCreateImage( cvSize(ab.cols,ab.rows), 8, 3 ); 
    mapx_persp = cvCreateMat(src1.height, src1.width, CV_32FC1);
    mapy_persp = cvCreateMat(src1.height, src1.width, CV_32FC1);
    float sf = 4;
    create_perspecive_undistortion_LUT( mapx_persp, mapy_persp, &o, sf );
    first_run = false;
  }
  cvRemap( &src1, dst_persp, mapx_persp, mapy_persp, CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS, cvScalarAll(0) );
  
  cv::Mat image=cv::cvarrToMat(dst_persp,true);
	//cv::namedWindow("OPENCV_WINDOW",CV_WINDOW_AUTOSIZE); 
        //cv::imshow("OPENCV_WINDOW", image);
 	 //fprintf(stdout, "fisheye_undistort finished\n");
	

	
  return image;

}


