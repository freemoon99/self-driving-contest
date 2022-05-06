//opencv_line_detect.cpp
//-I/usr/local/include/opencv4/opencv -I/usr/local/include/opencv4

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/Int16.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include <opencv2/opencv.hpp>

#include <iostream>
#include <stdio.h> 
#include <string.h> 
#include <stdio.h>
#include <unistd.h>   
#include <stdint.h>   
#include <stdlib.h>  
#include <errno.h>

using namespace cv;
using namespace std;

#define IMG_Width     1280
#define IMG_Height    720

#define USE_DEBUG  0   // 1 Debug  사용
#define USE_CAMERA 1   // 1 CAMERA 사용  0 CAMERA 미사용

#define ROI_CENTER_Y  250
#define ROI_WIDTH     50

#define NO_LINE 20
#define DEG2RAD(x) (M_PI/180.0)*x
#define RAD2DEG(x) (180.0/M_PI)*x

std::string gstreamer_pipeline (int capture_width, int capture_height, int display_width, int display_height, int framerate, int flip_method) {
    return "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=(int)" + std::to_string(capture_width) + ", height=(int)" +
           std::to_string(capture_height) + ", format=(string)NV12, framerate=(fraction)" + std::to_string(framerate) +
           "/1 ! nvvidconv flip-method=" + std::to_string(flip_method) + " ! video/x-raw, width=(int)" + std::to_string(display_width) + ", height=(int)" +
           std::to_string(display_height) + ", format=(string)BGRx ! videoconvert ! video/x-raw, format=(string)BGR ! appsink";
}


Mat mat_image_org_color;  // Image 저장하기 위한 변수
Mat mat_image_org_gray;
Mat mat_image_roi;
Mat mat_image_canny_edge;
Mat mat_image_canny_edge_roi;

Scalar GREEN(0,255,0);
Scalar RED(0,0,255);
Scalar BLUE(255,0,0);
Scalar YELLOW(0,255,255);


int img_width = 640;
int img_height = 480;

Mat Region_of_Interest(Mat image, Point *points)
{
  Mat img_mask =Mat::zeros(image.rows,image.cols,CV_8UC1);	 
  
  Scalar mask_color = Scalar(255,255,255);
  const Point* pt[1]={ points };	    
  int npt[] = { 4 };
  cv::fillPoly(img_mask,pt,npt,1,Scalar(255,255,255),LINE_8);
  Mat masked_img;	
  bitwise_and(image,img_mask,masked_img);
  
  return masked_img;
}

Mat Region_of_Interest_crop(Mat image, Point *points)
{
   Mat img_roi_crop;	

   Rect bounds(0,0,image.cols,image.rows);	 
   Rect r(points[0].x,points[0].y,image.cols, points[2].y-points[0].y);  
   //printf("%d %d %d %d\n",points[0].x,points[0].y,points[2].x, points[2].y-points[0].y);
   //printf("%d  %d\n", image.cols, points[2].y-points[0].y);

   img_roi_crop = image(r & bounds);
   
   return img_roi_crop;
}

Mat Canny_Edge_Detection(Mat img)
{
   Mat mat_blur_img, mat_canny_img;
   blur(img, mat_blur_img, Size(3,3));	
   Canny(mat_blur_img,mat_canny_img, 70,150,3);
	
   return mat_canny_img;	
}

int main(int argc, char **argv)
{
	
   int i;
   int steer_angle_new, steer_angle_old;
   
   steer_angle_new = steer_angle_old =0; 
 
   float gradient[NO_LINE]  = {0,};
   float intersect[NO_LINE] = {0,};
   float intersect_base[NO_LINE]  = {0,};
   float c_x_sum=0;	
   
   int capture_width = 1280 ;
   int capture_height = 720 ;
   int display_width = 640 ;
   int display_height = 360 ;
   int framerate = 60 ;
   int flip_method = 2 ;
    
   int img_width  = 640;
   int img_height = 360;
   if(USE_CAMERA == 0) img_height = 480;	 
   
   std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  // return 1;
    
   std::string pipeline = gstreamer_pipeline(capture_width,
	capture_height,
	display_width,
	display_height,
	framerate,
	flip_method);
	
    std::cout << "Using pipeline: \n\t" << pipeline << "\n";
 
   VideoCapture cap(pipeline, cv::CAP_GSTREAMER);
 
   Point points[4];  // ROI(Region of Interest) 
 
   if (!cap.isOpened()) 
   {
	cerr << "에러 - 카메라를 열 수 없습니다.\n";		
	return -1;	
   }
   
   cap.read(mat_image_org_color);
   img_width = mat_image_org_color.size().width ;
   img_height = mat_image_org_color.size().height;
	
   if(USE_CAMERA == 0) printf("Image size[%3d,%3d]\n", capture_width,capture_height);	
  
  
   ros::init(argc, argv, "m_race_vision");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
   ros::NodeHandle nh;
   
   ros::Publisher car_control_pub_cmd = nh.advertise<std_msgs::Int16>("Car_Control_cmd/V_SteerAngle_Int16", 10);
   
   std_msgs::Int16 cmd_steering_msg;      
   cmd_steering_msg.data  = 0;
   
   std::cout << "OpenCV version : " << CV_VERSION << std::endl;
  
   ros::Rate loop_rate(10);
   int count = 0;
   int line_count = 0;
   float  c[NO_LINE] = {0.0, };
   float  d[NO_LINE] = {0.0, };
   float  line_center_x = 0.0; 
   float  steering_conversion = 0.07;
   int inter_sect_x[NO_LINE] = {0, };
   int inter_sect_group[NO_LINE] = {0, };  
  ////////////////  image transport ///////////////////////////
   image_transport::ImageTransport it(nh);
   image_transport::Publisher pub = it.advertise("camera/image", 1);
   /* 
   namedWindow("Camera Image", WINDOW_NORMAL);
   resizeWindow("Camera Image", IMG_Width/2,IMG_Height/2);
   moveWindow("Camera Image", 10, 10); 
                             
   namedWindow("Gray Image window", WINDOW_NORMAL);   
   resizeWindow("Gray Image window", img_width,img_height);   
   moveWindow("Gray Image window", 700, 10);
   
   namedWindow("ROI Image window", WINDOW_NORMAL);   
   resizeWindow("ROI Image window", img_width,img_height);   
   moveWindow("ROI Image window", 10, 500);
    
   namedWindow("Edge Image window", WINDOW_NORMAL);   
   resizeWindow("Edge Image window", img_width,img_height);   
   moveWindow("Edge Image window", 700, 500);
    */
   points[0] = Point(0,ROI_CENTER_Y-ROI_WIDTH);
   points[1] =  Point(0,ROI_CENTER_Y+ROI_WIDTH);
   points[2] =  Point(img_width,ROI_CENTER_Y+ROI_WIDTH);
   points[3] =  Point(img_width,ROI_CENTER_Y-ROI_WIDTH);
   
   sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_org_color).toImageMsg();
       
   while (ros::ok())
   { 
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
     if(USE_CAMERA == 1)  cap.read(mat_image_org_color);
     else                 mat_image_org_color = imread("./img/line_1.jpg", IMREAD_COLOR);
     cvtColor(mat_image_org_color, mat_image_org_gray, cv::COLOR_RGB2GRAY);        // color to gray conversion  
     mat_image_roi = Region_of_Interest_crop(mat_image_org_gray,points);    // ROI 영역을 추출함      
     mat_image_canny_edge = Canny_Edge_Detection(mat_image_roi);
      
     vector<Vec4i> linesP;
     HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI/180,30,30,40);
     //printf("Line Number : %3d\n", (int)linesP.size());
     
     line_count = line_center_x = 0.0;
	  
	  for(int i=0; i<linesP.size();i++)
	  {
		float intersect = 0.0;
		
		if(i>=NO_LINE) break;
		Vec4i L= linesP[i];
		/*
		int cx1 = linesP[i][0];
		int cy1 = linesP[i][1];
		int cx2 = linesP[i][2];
		int cy2 = linesP[i][3];
		*/
		c[i] =  (L[2]-L[0])/(L[3]-L[1]);
		d[i] = L[0]-c[i] *L[1] ;
		if(fabs(c[i])< DEG2RAD(65))
		{
		
		   intersect = c[i]*(float)ROI_WIDTH +d[i];
		   line_center_x += intersect;
		   inter_sect_x[line_count]=intersect;
		   line_count++;
		   line(mat_image_org_color,Point(L[0],L[1]+ROI_CENTER_Y-ROI_WIDTH),Point(L[2],L[3]+ROI_CENTER_Y-ROI_WIDTH), Scalar(0,0,255), 2, LINE_AA);		   
	    }
		
		if(USE_DEBUG==1)
		 {
		   printf("L[%d] :[%3d, %3d] , [%3d , %3d] \n",i,  L[0],L[1], L[2],L[3]); 
		 //printf("x[%d] = [%6.3f] *y + [%6.3f] \n",i, c[i],d[i]); 
		   printf("x[%d] = [%4.2f] *y + [%4.2f] \n", i,c[i],d[i]); 
		   printf("intersect =[%f] [%f]\n\n", intersect, line_center_x);
		  //printf("H :[%3d , %3d] , [%3d , %3d] \n", cx1,cy1, cx2,cy2);
	     }
	   } 
	   
	  
      
     if(line_count >0)
     {
        
        line_center_x = line_center_x / (float)linesP.size() - img_width/2;
	
        steer_angle_new = (int)( line_center_x*steering_conversion);  //스티어링 조정값 계수 수정 필요 
        //printf("c_x_sum = %f  %d\n",line_center_x , steer_angle_new);
	    //printf("\n\n\n");
        cmd_steering_msg.data  = steer_angle_new;      //
        
        if(steer_angle_old !=  steer_angle_new) car_control_pub_cmd.publish(cmd_steering_msg);
	
	    line(mat_image_org_color,Point(0,ROI_CENTER_Y),Point(img_width,ROI_CENTER_Y), Scalar(0,255,0), 1, LINE_AA);	
	    line(mat_image_org_color,Point((int)line_center_x+img_width/2,ROI_CENTER_Y-ROI_WIDTH),Point((int)line_center_x+img_width/2,ROI_CENTER_Y+ROI_WIDTH), Scalar(255,255,0), 1, LINE_AA);	
	   
     }
      
     else
     {
         
     }
       steer_angle_old =  steer_angle_new ;     
     
      img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", mat_image_org_color).toImageMsg();  
      pub.publish(img_msg);
     // imshow("Camera Image", mat_image_org_color);
    /*imshow("Gray Image window", mat_image_org_gray); 
      imshow("ROI Image window",mat_image_roi);
      imshow("Edge Image window",mat_image_canny_edge_roi);  
    */
    if (waitKey(25) >= 0)
      break;
     
     ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  //destroyWindow("Camera Image");
  //destroyWindow("Gray Image window");
  //destroyWindow("ROI Image window");
  //destroyWindow("Edge Image window");
   return 0;
}

 
