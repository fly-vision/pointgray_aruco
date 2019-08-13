
#include "ros/ros.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"

#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/image_encodings.h"

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "aruco.h"
#include "cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>

using namespace std;
using namespace cv;
using namespace aruco;


aruco::CameraParameters CamParam;


/**
 * Callback executed every time a new camera frame is received.
 * This callback is used to process received images and publish messages with camera position data if any.
 */
void onFrame(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{		
		Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image;

        imshow("recv_im", frame);
        waitKey(10);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("Error getting image data");
	}
}





int main(int argc, char** argv)
{
    ros::init(argc, argv, "aruco");

	//ROS node instance
	ros::NodeHandle node("aruco");

    string topic_camera = "/graypic/image_raw";
    string camera_yml = "src/pointgray_aruco/aruco_ros/ros/cm3_u3_13y3c_cs_sn_16466253.yml";

    CamParam.readFromXMLFile(camera_yml);
    
    //Subscribe topics
	image_transport::ImageTransport it(node);
	image_transport::Subscriber sub_camera = it.subscribe(topic_camera, 1, onFrame);

    ros::spin();
}
