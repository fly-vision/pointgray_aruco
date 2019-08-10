#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "stdafx.h"
#include "FlyCapture2.h"
#include <iostream>
#include <sstream>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/objdetect/objdetect.hpp>
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include "opencv2/opencv.hpp"  
#include <opencv2/core/core.hpp>


using namespace FlyCapture2;
using namespace std;

void PrintError(Error error) { error.PrintErrorTrace(); }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "graypic");
  ros::NodeHandle nh_("~");
  if(!nh_.ok())return 0;

  int width_, height_;
  string camera_frame_id_("head_camera");

  ros::Publisher imgPub = nh_.advertise<sensor_msgs::Image>("image_raw", 1000);

  //image_transport::ImageTransport it(nh_);
  //ros::advertis imgPub = it.advertiseCamera("image_raw", 10);

  //nh_.param<int>("image_width", width_, 640);
  //nh_.param<int>("image_height", height_, 480);
  //nh_.param("camera_frame_id", camera_frame_id_, std::string("head_camera"));
  width_=640;
  height_=480;


  // check for default camera info

  //connect camera
  Error error;
  BusManager busMgr;
  unsigned int numCameras;
  error = busMgr.GetNumOfCameras(&numCameras);
  if (error != PGRERROR_OK)
  {
      PrintError(error);
      return -1;
  }
  cout << "Number of cameras detected: " << numCameras << endl;
  PGRGuid guid;
  error = busMgr.GetCameraFromIndex(0, &guid);
  if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

  Camera cam;
  //FLIR::mGigEGrab cam(width_, height_);
  error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

  cv::Mat cvImage;
  Image rawImage, convertedImage;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    
     error = cam.RetrieveBuffer(&rawImage);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

       error = rawImage.Convert(PIXEL_FORMAT_BGR, &convertedImage);
        if (error != PGRERROR_OK)
        {
            PrintError(error);
            return -1;
        }

      unsigned int rowBytes = (double)convertedImage.GetReceivedDataSize()/(double)convertedImage.GetRows();
      cvImage = cv::Mat( convertedImage.GetRows(), convertedImage.GetCols(), CV_8UC3, convertedImage.GetData(), rowBytes );
      //cv::imshow("i am die", cvImage ); 
      //cv::waitKey(30);
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
  error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }
  error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError(error);
        return -1;
    }

  return 0;
}

