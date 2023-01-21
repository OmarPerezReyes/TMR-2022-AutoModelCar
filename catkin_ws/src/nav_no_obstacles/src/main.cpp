/*
# Released under MIT License
Copyright (c) 2022 Héctor Hugo Avilés Arriaga.
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "ros/ros.h"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <unistd.h>

#include <iostream>
#include <chrono>
#include <atomic>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <rosgraph_msgs/Clock.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt16.h"

//**************************************************

ros::Publisher pubSpeed; 
ros::Publisher pubSteering;

cv::Mat orig_image(cv::Size(640, 480), CV_8UC3);

double FT = 0;
int l = 50; //80 
int x_ref = 120;
int x1_h = 120;
double u = 0.0;
double v = 55.0; //55.0

// Define the images that will be used first
cv::Mat orig;
cv::Mat imagenF(300, 200, CV_8UC3); //Destination for warped image   
cv::Mat imagenFHSV1(300, 200, CV_8UC3); 
cv::Mat imagenFHSV2(300, 200, CV_8UC3); 
cv::Mat imagenF1(300, 200, CV_8UC1);  
cv::Mat imagenF2(300, 200, CV_8UC1);  

cv::Mat imagenFNew(300, 200, CV_8UC1); 
cv::Mat imagenFThresh(300, 200, CV_8UC1); 
cv::Mat imagenFSobel(300, 200, CV_8UC1); 

cv::Mat perspectiveMatrix = (cv::Mat_<double>(3,3) << -5.88392618e-02,-4.02514041e-01,1.19565927e+02, 1.24049432e-18,-1.34260497,3.67342070e+02, 4.47714719e-21,-4.01176785e-03,1.0);

// Function declaration  
void imageCallback(const sensor_msgs::ImageConstPtr& msg);
void line_detector(cv::Mat imagen0, int l, int side, cv::Point *pt1, cv::Point *pt2);
int roi_zone(int x);
void vec_create(int x, int stride, int side, int *xi, int *xd);
unsigned char get_pixel_grey(const cv::Mat &frame, int x, int y);


int main(int argc, char **argv){
    // ROS initialization
   ros::init(argc, argv, "brake");

    if (!ros::isInitialized()){         
       std::cout << "ROS not initialized" << std::endl;                    
       exit(EXIT_FAILURE);
    }

    ros::NodeHandle n("~");
      
    ros::Rate rate(2);

    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/raw", 1, imageCallback);

    pubSpeed = n.advertise<std_msgs::Float64>("/speed", 15);
    rate.sleep(); 

    pubSteering = n.advertise<std_msgs::Float64>("/steering", 15);
    rate.sleep(); 

    while(ros::ok()) {
      ros::spinOnce();
   }

   std::cout << "Main loop is about to die " << std::endl;
   std::cout.flush();      

    // HHAA These lines MUST be here
    ros::shutdown();
       
    exit(EXIT_SUCCESS);
}

//***********************************
int roi_zone(int x){

 int y = 0;

   if (!((x >= 0) && (x <= 199))){
     //std::cout << "x: " << x << " is out of limits " << std::endl;
     //std::cout.flush();  
   }

   if ((x > 120) && (x <= 199)){
      y = int(round(-1.7722*x + 511.6582));
   } 
   if ((x >= 80) && (x <= 120)){
      y = 299;
   } 
   if ((x >= 0) && (x < 80)){
      y = int(round(1.6875*x + 164.0));
   }

  return y;
}


//***********************************
void vec_create(int x, int stride, int side, int *xi, int *xd){

  if (side == 1){
    *xi = x + stride;
    *xd = x - stride;
  } else {
    *xi = x - stride;
    *xd = x + stride;
  }

  if(*xi < 0){ *xi = 0; }
  if(*xi > 199){ *xi = 299; }
  if(*xd < 0){ *xi = 0; }
  if(*xd > 199){ *xi = 299; }

}

//**************************************************
void line_detector(cv::Mat imagen0, int l, int side, cv::Point *pt1, cv::Point *pt2){

  bool K = true;
  int stridex = 3;
  int stridey = 5;
  pt1->y = roi_zone(pt1->x);

  int xi = 0;
  int xd = 0;
  vec_create(pt1->x, stridex, side, &xi, &xd);
   
  while (K){
     int m = pt1->y + stridey;
     if (m >= 299){ m = 299;}
     for (int j = m; j > pt1->y - stridey; j--){
        for (int i = xi; i > xd; i--){ 
	   if (get_pixel_grey(imagen0, i, j) == 255){
	      pt1->x = i;
	      pt1->y = j; 
	      K = false;
	      break;
           } 
        }
        xi = 0;
        xd = 0;
	vec_create(pt1->x, stridex, side, &xi, &xd);
	if (K == false){ break; }
     }
     if (K == true){ 
	pt1->x = pt1->x - 1 * side;
	pt1->y = roi_zone(pt1->x);
     }

  } // while

  xi = 0;
  xd = 0;
  pt2->x = pt1->x;
  vec_create(pt2->x, stridex, side, &xi, &xd);
  for (int j = pt1->y - 1; j > pt1->y - l; j--){
      for (int i = xi; i > xd; i--){ 
	  if (get_pixel_grey(imagen0, i, j) == 255){
	     pt2->x = i;
	     pt2->y = j;
	     K = false;
	     break;
          } 
      }
      vec_create(pt2->x, stridex, side, &xi, &xd);	
  }  
	
}

//**************************************************
void imageCallback(const sensor_msgs::ImageConstPtr& msg){
  try
  {
    //orig_image, original
    orig_image = cv_bridge::toCvShare(msg, "bgr8")->image.clone(); 
    orig = orig_image.clone(); 
    
    //imagenF, perspectiva, tip function in python
    cv::warpPerspective(orig, imagenF, perspectiveMatrix, imagenF.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT);  
    
    //imagenF1,blanco bordes, in range(cvtcolor->colores bordes rojos)
    cv::cvtColor(imagenF, imagenFHSV1, cv::COLOR_BGR2HSV);
    cv::inRange(imagenFHSV1, cv::Scalar(10, 0, 100), cv::Scalar(30, 100, 150), imagenF1);
    
    //imagenF2, lineas dividen carril blanco,  in range(cvtcolor)
    cv::cvtColor(imagenF, imagenFHSV2, cv::COLOR_BGR2HSV);
    cv::inRange(imagenFHSV2, cv::Scalar(0, 0, 100), cv::Scalar(179, 50, 255), imagenF2);    

    //imagenFNew, desenfoque gaussiano no idéntico al original (cambio blend por suma)
    cv::GaussianBlur(imagenF1+imagenF2, imagenFNew, cv::Size(9, 9), 0); 
    
    //imagenFThresh, blanco, umbralización, agrupa pixeles en regiones mayores
    cv::threshold(imagenFNew, imagenFThresh, 25, 255,cv::THRESH_BINARY);
    
    //imagenFSobel, img final, bordes
    cv::Sobel(imagenFThresh, imagenFSobel, CV_8UC1, 1, 0, 7, 1, 0, cv::BORDER_DEFAULT); 
    
    //Desplegar ventana de segmentación de carriles
    cv::imshow("imagenFSobel", imagenFSobel);
    cv::waitKey(1);

    cv::Point pt1( 120, 0);
    cv::Point pt2( 120, 0);
    
    //cambio valor de 0 a 30
    if (FT <= 30){
       pt1.x = 180;
       FT = FT + 1;
    } else {
       pt1.x = x1_h;
    } 

    line_detector(imagenFSobel, l, 1, &pt1, &pt2);
    x1_h = pt1.x;

    cv::rectangle(imagenFSobel, pt1, pt2, cv::Scalar(128, 128, 128), cv::FILLED, 8, 0);

    // CONTROL
    double ky = 0.02143299;
    double kth = 0.25015021;

    double e_y = pt1.x - x_ref;
    // Ángulo??
    double e_th = atan2(pt2.x - pt1.x, l);
    u = atan(ky * e_y + kth * e_th);

    if (u > 0.83){ u = 0.83; }
    if (u < -0.83){ u = -0.83; }
    
    std_msgs::Float64 speedMsg;  
    speedMsg.data = v;
    pubSpeed.publish(speedMsg);

    std_msgs::Float64 steeringMsg;
    steeringMsg.data = u;
    pubSteering.publish(steeringMsg);    
       

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }
}


//**************************************************
unsigned char get_pixel_grey(const cv::Mat &frame, int x, int y){

   int offset = 0;

   if (x < 0) { x = 0; }
   if (x >= frame.cols) { x = frame.cols - 1; }
   if (y < 0) { y = 0; }
   if (y >= frame.rows) { y = frame.rows - 1; }

   offset += y * frame.step;
  
   // Offset
   offset +=  frame.channels() * x;

   return (frame.data[offset]);

}
