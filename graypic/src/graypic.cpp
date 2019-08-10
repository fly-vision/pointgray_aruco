#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "stdafx.h"

#include <iostream>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/opencv.hpp"  
#include <opencv2/core/core.hpp>

#include "FlyCapture2.h"

using namespace FlyCapture2;
using namespace std;

void PrintError(Error error) { error.PrintErrorTrace(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graypic");
  ros::NodeHandle nh_("~");
  if(!nh_.ok())return 0;

  //int width_, height_;
  string camera_frame_id_("head_camera");

  ros::Publisher imgPub = nh_.advertise<sensor_msgs::Image>("image_raw", 10);

  //image_transport::ImageTransport it(nh_);
  //ros::advertis imgPub = it.advertiseCamera("image_raw", 10);

  //nh_.param<int>("image_width", width_, 640);
  //nh_.param<int>("image_height", height_, 480);
  //nh_.param("camera_frame_id", camera_frame_id_, std::string("head_camera"));
  //width_=640;
  //height_=480;

  //check for default camera info

  //connect camera
  Error error;
  BusManager busMgr;
  unsigned int numCameras;
  busMgr.GetNumOfCameras(&numCameras);
  cout << "Number of cameras detected: " << numCameras << endl;
  PGRGuid guid;
  busMgr.GetCameraFromIndex(0, &guid);
  Camera cam;
  //FLIR::mGigEGrab cam(width_, height_);
  cam.Connect(&guid);
  cam.StartCapture();
  cv::Mat cvImage;
  Image rawImage, convertedImage;

  ros::Rate loop_rate(200);
  bool init_(true);
  while (ros::ok())
  {
    
    cam.RetrieveBuffer(&rawImage);
    rawImage.Convert(PIXEL_FORMAT_BGR, &convertedImage);
    unsigned int rowBytes = (double)convertedImage.GetReceivedDataSize() / (double)convertedImage.GetRows();
    cvImage = cv::Mat(convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3, convertedImage.GetData(), rowBytes);
    
    if (init_)
    {
      cout << "INIT: Image Resolution: (" << cvImage.cols << ", " << cvImage.rows << ")" << endl;
      init_ = false;
    }
    cv_bridge::CvImage out_msg;
    out_msg.header.stamp=ros::Time::now();
    out_msg.header.frame_id=camera_frame_id_;
    out_msg.encoding=sensor_msgs::image_encodings::BGR8;
    out_msg.image=cvImage;

    sensor_msgs::Image img_;
    out_msg.toImageMsg(img_);

    imgPub.publish(img_);

    ros::spinOnce();
    loop_rate.sleep();
  }
  cam.StopCapture();
  cam.Disconnect();

  return 0;
}

