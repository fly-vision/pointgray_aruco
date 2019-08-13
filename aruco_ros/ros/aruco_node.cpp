
#include "ros/ros.h"
#include "ros/package.h"

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
#include <aruco/MarkerArray.h>

using namespace std;
using namespace cv;
using namespace aruco;


aruco::CameraParameters CamParam;
MarkerDetector MDetector;
std::map<uint32_t, MarkerPoseTracker> MTracker;
float MarkerSize = 0.2;  // 20 cm

ros::Publisher imgPub;
ros::Publisher markerPub;
aruco::MarkerArray::Ptr marker_msg_;



/**
 * Callback executed every time a new camera frame is received.
 * This callback is used to process received images and publish messages with camera position data if any.
 */
void onFrame(const sensor_msgs::ImageConstPtr& msg)
{
	try
	{		
		Mat InImage = cv_bridge::toCvShare(msg, "bgr8")->image;

        // Ok, let's detect
        vector<Marker> Markers = MDetector.detect(InImage);
        for (auto& marker : Markers)  // for each marker
            MTracker[marker.id].estimatePose(marker, CamParam, MarkerSize);  // call its tracker and estimate the pose

        ros::Time curr_stamp(ros::Time::now());
        marker_msg_->markers.clear();
        marker_msg_->markers.resize(Markers.size());
        marker_msg_->header.stamp = curr_stamp;
        marker_msg_->header.seq++;

        // for each marker, draw info and its boundaries in the image
        for (unsigned int i = 0; i < Markers.size(); i++)
        {
            aruco::MarkerMsg& marker_i = marker_msg_->markers.at(i);
            marker_i.header.stamp = curr_stamp;
            marker_i.id = Markers.at(i).id;
            marker_i.confidence = 1.0;

            marker_i.pose.position.x = Markers[i].Tvec.at<float>(0);
            marker_i.pose.position.y = Markers[i].Tvec.at<float>(1);
            marker_i.pose.position.z = Markers[i].Tvec.at<float>(2);
            marker_i.pose.orientation.x = Markers[i].Rvec.at<float>(0);
            marker_i.pose.orientation.y = Markers[i].Rvec.at<float>(1);
            marker_i.pose.orientation.z = Markers[i].Rvec.at<float>(2);

            // cout << Markers[i] << endl;
            Markers[i].draw(InImage, Scalar(0, 0, 255), 2);
        }
        // draw a 3d cube in each marker if there is 3d info
        if (CamParam.isValid() && MarkerSize != -1)
        {
            for (unsigned int i = 0; i < Markers.size(); i++)
            {
                if (Markers[i].isPoseValid()){
                    CvDrawingUtils::draw3dCube(InImage, Markers[i], CamParam);
                    CvDrawingUtils::draw3dAxis(InImage, Markers[i], CamParam);
                }
            }
        }
        // show input with augmented information
        // cv::namedWindow("in", 1);
        // cv::imshow("in", InImage);
        // waitKey(10);
        if (marker_msg_->markers.size() > 0)
            markerPub.publish(marker_msg_);
    
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = ros::Time::now();
        out_msg.header.frame_id = "head_camera";
        out_msg.encoding=sensor_msgs::image_encodings::BGR8;
        out_msg.image = InImage;

        sensor_msgs::Image img_;
        out_msg.toImageMsg(img_);

        imgPub.publish(img_);
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
    string camera_yml = ros::package::getPath("aruco") + "/ros/cm3_u3_13y3c_cs_sn_16466253.xml";

    cout << "Reading Camera Params: " << camera_yml << endl;
    CamParam.readFromXMLFile(camera_yml.c_str());
    MDetector.setDictionary("ARUCO_MIP_36h12", 0.f);
    marker_msg_ = aruco::MarkerArray::Ptr(new aruco::MarkerArray());
    marker_msg_->header.frame_id = "";
    marker_msg_->header.seq = 0;
    

    imgPub = node.advertise<sensor_msgs::Image>("/aruco/detection_vis", 10);
    markerPub = node.advertise<aruco::MarkerArray>("/aruco/markers", 10);
    //Subscribe topics
	image_transport::ImageTransport it(node);
	image_transport::Subscriber sub_camera = it.subscribe(topic_camera, 1, onFrame);

    ros::spin();
}
