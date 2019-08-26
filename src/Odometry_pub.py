#!/usr/bin/env python
# Python implementation of "ekf_slam"

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
from aruco.msg import MarkerArray
from math import *
import numpy as np
import tf
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

pub1_ = rospy.Publisher("/ekf_estimate/path", Path, queue_size=10)
groundtruth_path = Path()
estimate_pub_ = rospy.Publisher('/ekf_estimate', Odometry, queue_size=10)

class ekf_slam:
    #init functions
    def __init__(self):
        #init stuff
        #get stuff

        self.num_landmarks = 25

        # Estimator stuff
        # x = pn, pe, pd, phi, theta, psi
        self.xhat = np.zeros((9 + 3*self.num_landmarks, 1))
        # self.xhat_odom = Odometry()

        # Covariance matrix
        self.P = np.zeros((9 + 3*self.num_landmarks, 9 + 3*self.num_landmarks))
        self.P[9:,9:] = np.eye(3*self.num_landmarks)*9999999.9 # Inf
        self.Q = np.diag([10.0, 5.0, 5.0]) # meas noise

        # Measurements stuff
        # Truth
        self.xhat[0]=0
        self.xhat[1]=0
        self.xhat[2]=0
        self.xhat[3]=0
        self.xhat[4]=0
        self.xhat[5]=0
        self.xhat[6]=0
        self.xhat[7]=0
        self.xhat[8]=0


        #aruco Stuff
        self.aruco_location = {
        # 1 col
        #0	:[52.534 , 0 , 0],
        1	:[45.058 , 0 , 0],
        2	:[37.628 , 0 , 0],
        3	:[29.937 , 0 , 0],
        4	:[22.637  , 0 , 0],
        5	:[15.121 , 0 , 0],
        6	:[7.560 , 0 , 0],
        7	:[0 , 0 , 0],
        # 2 col
        8	:[52.534  , 6.734761 , 0],
        9	:[45.058 , 6.734761 , 0],
        10	:[37.628 , 6.734761 , 0],
        11	:[29.937 , 6.734761 , 0],
        12	:[22.637 , 6.734761 , 0],
        13	:[15.121 , 6.734761 , 0],
        14	:[7.5603 , 6.734761 , 0],
        15	:[0  , 6.734761 , 0],
        # 3 col
        116	:[52.534  , 13.339 , 0],
        17	:[45.058 , 13.339 , 0],
        18	:[37.628 , 13.339 , 0],
        19	:[29.937 , 13.339 , 0],
        20	:[22.637 , 13.339 , 0],
        21	:[15.121 , 13.339 , 0],
        22	:[7.5603 , 13.339 , 0],
        23	:[0  , 13.339 , 0],
        # 4 col
        24	:[52.534  , 20.074, 0],
        25	:[45.058 , 20.074 , 0],
        26	:[37.628 , 20.074 , 0],
        27	:[29.937 , 20.074 , 0],
        28	:[22.637 , 20.074 , 0],
        29	:[15.121 , 20.074 , 0],
        30	:[7.5603 , 20.074 , 0],
        31	:[0  , 20.074 , 0],
        # 5 col
        32	:[52.534 , 26.809 , 0],
        33	:[45.058 , 26.809 , 0],
        34	:[37.628 , 26.809 , 0],
        35	:[29.937 , 26.809 , 0],
        36	:[22.637 , 26.809 , 0],
        37	:[15.121 , 26.809 , 0],
        38	:[7.5603 , 26.809 , 0],
        39	:[0  , 26.809 , 0],
        # 6 col
        40	:[52.534 , 33.413 , 0],
        41	:[45.058 , 33.413 , 0],
        42	:[37.628 , 33.413 , 0],
        43	:[29.937 , 33.413 , 0],
        44	:[22.637 , 33.413 , 0],
        45	:[15.121 ,33.413 , 0],
        46	:[7.5603 , 33.413 , 0],
        47	:[0  , 33.413 , 0],
        # 7 col
        48	:[52.534  , 40.061 , 0],
        49	:[45.058  , 40.061 , 0],
        50	:[37.628 , 40.061 , 0],
        51	:[29.937 , 40.061 , 0],
        52	:[22.637 , 40.061 , 0],
        53	:[15.121 , 40.061, 0],
        54	:[7.5603 , 40.061 , 0],
        55	:[0 , 40.061 , 0],
        # 8 col
        56	:[52.534  , 46.796 , 0],        
        57	:[45.058  , 46.796 , 0],
        58	:[37.628 , 46.796 , 0],
        59	:[29.937 , 46.796 , 0],
        60	:[22.637 , 46.796 , 0],
        61	:[15.121 , 46.796 , 0],
        62	:[7.5603 , 46.796 , 0],
        63	:[0 , 46.796 , 0],
        # 9 col
        64	:[52.534  , 53.4 , 0],
        65	:[45.058 , 53.4 , 0],
        66	:[37.628 , 53.4 , 0],
        67	:[29.937 , 53.4 , 0],
        68	:[22.637 , 53.4 , 0],
        69	:[15.121 , 53.4 , 0],
        70	:[7.5603 , 53.4 , 0],
        71	:[0  , 53.4 , 0], 

        # 10 col
        72	:[52.534  , 60.004 , 0],
        73	:[45.058 , 60.004 , 0],
        74	:[37.628 , 60.004 , 0],
        75	:[29.937 , 60.004 , 0],
        76	:[22.637 , 60.004 , 0],
        77	:[15.121 , 60.004 , 0],
        78	:[7.5603 , 60.004 , 0],
        79	:[0  , 60.004 , 0], 

        # 11 col
        80	:[52.534  , 66.739 , 0],
        81	:[45.058 , 66.739 , 0],
        82	:[37.628 , 66.739 , 0],
        83	:[29.937 , 66.739 , 0],
        84	:[22.637 , 66.739 , 0],
        85	:[15.121 , 66.739 , 0],
        86	:[7.5603 , 66.739 , 0],
        87	:[0  , 66.739 , 0],  

        # 12 col
        88	:[52.534  , 73.344 , 0],
        89	:[45.058 , 73.344 , 0],
        90	:[37.628 , 73.344 , 0],
        91	:[29.937 , 73.344 , 0],
        92	:[22.637 , 73.344 , 0],
        93	:[15.121 , 73.344 , 0],
        94	:[7.5603 , 73.344 , 0],
        95	:[0  , 73.344 , 0],

        # 13 col
        96	:[52.534  , 80.078 , 0],
        97	:[45.058 , 80.078 , 0],
        98	:[37.628 , 80.078 , 0],
        99	:[29.937 , 80.078 , 0],
        #100	:[22.637 , 80.078 , 0],
        101	:[15.121 , 80.078 , 0],
        102	:[7.5603 , 80.078 , 0],
        103	:[0  , 80.078 , 0],   

        # 14 col
        104	:[52.534  , 86.813 , 0],
        105	:[45.058 , 86.813 , 0],
        106	:[37.628 , 86.813 , 0],
        107	:[29.937 , 86.813 , 0],
        108	:[22.637 , 86.813 , 0],
        109	:[15.121 , 86.813 , 0],
        110	:[7.5603 , 86.813 , 0],
        111	:[0  , 86.813 , 0],

        # 15 col
        112	:[52.534  , 93.418 , 0],
        113	:[45.058 ,  93.418 , 0],
        114	:[37.628 ,  93.418 , 0],
        115	:[29.937 ,  93.418 , 0],
        116	:[22.637 ,  93.418 , 0],
        117	:[15.121 ,  93.418 , 0],
        118	:[7.5603 ,  93.418 , 0],
        119	:[0  ,  93.418 , 0],

        }
        """ self.landmark_number = {
        76:[1],
        245:[2],
        55:[3],
        110:[4],
        248:[5],
        64:[6],
        25:[7],
        121:[8],
        5:[9],
        } """
        # Init subscribers
        self.aruco_sub = rospy.Subscriber('/aruco/markers', MarkerArray, self.aruco_meas_callback )

        # Init publishers
        #self.pose_temp = PoseStamped()
        
        
        #self.pub1_ = rospy.Publisher("/ekf_estimate/path", Path, queue_size=10)
        #self.groundtruth_path = Path()

        #self.pub_location = rospy.Publisher("/ekf_estimate/location", Vector3Stamped, queue_size=10)
        

        # # Init Timer
        #self.pub_rate_ = 100. #
        #self.update_timer_ = rospy.Timer(rospy.Duration(1.0/self.pub_rate_), self.pub_est)


   
    def pub_est(self,loc_x,loc_y,loc_z):

        # pack up estimate to ROS msg and publish
        """ 
        location=Vector3Stamped()
        location.header.stamp = rospy.Time.now()
        location.vector.x = self.xhat[0]
        location.vector.y = self.xhat[1]
        location.vector.z = self.xhat[2]
        self.pub_location.publish(location)
        """
        
        xhat_odom = Odometry()
        xhat_odom.header.stamp = rospy.Time.now()
        xhat_odom.header.frame_id="world"
        xhat_odom.child_frame_id="world"
        xhat_odom.pose.pose.position.x = loc_x # pn
        xhat_odom.pose.pose.position.y = loc_y # pe
        xhat_odom.pose.pose.position.z = loc_z # pd

        #quat = tf.transformations.quaternion_from_euler(self.xhat[6].copy(), self.xhat[7].copy(), self.xhat[8].copy())

        xhat_odom.pose.pose.orientation.x = 0
        xhat_odom.pose.pose.orientation.y = 0
        xhat_odom.pose.pose.orientation.z = 0.707107
        xhat_odom.pose.pose.orientation.w = 0.707107
        #xhat_odom.twist.twist.linear.x = 0 # u
        #xhat_odom.twist.twist.linear.y = 0 # v
        #xhat_odom.twist.twist.linear.z = 0 # w

        estimate_pub_.publish(xhat_odom)
        pose_temp=PoseStamped()       
        pose_temp.header=xhat_odom.header
        pose_temp.pose=xhat_odom.pose.pose

        groundtruth_path.header = pose_temp.header
        groundtruth_path.poses.append(pose_temp)
        pub1_.publish(groundtruth_path) 



    def aruco_meas_callback(self, msg):
        #print "True Location:", self.aruco_location[self.aruco_id]
        #h=8
        #w=6.7
        aruco_x = 0
        aruco_y = 0
        aruco_z = 0
        if len(msg.markers)>0:
            num=0
            for i in range (0,len(msg.markers)):
                aruco_id = msg.markers[i].id
                if aruco_id == 0:
                    continue
                num+=1
                #aruco_location_x=(7-(aruco_id%8))*h
                #aruco_location_y=(aruco_id//8)*w
                aruco_location_x=self.aruco_location[aruco_id][0]
                aruco_location_y=self.aruco_location[aruco_id][1]
                aruco_location_z=self.aruco_location[aruco_id][2]+0.1
                #self.aruco_x = (-msg.markers[i].pose.position.x + self.aruco_location[self.aruco_id].[0])/len(msg.markers)
                #self.aruco_y = (-msg.markers[i].pose.position.y +self.aruco_location[self.aruco_id].[1])/len(msg.markers)
                #self.aruco_z = (msg.markers[i].pose.position.z +self.aruco_location[self.aruco_id].[2])/len(msg.markers)
                aruco_x += (aruco_location_y - msg.markers[i].pose.position.x )
                aruco_y += (aruco_location_x + msg.markers[i].pose.position.y )
                aruco_z += (aruco_location_z + msg.markers[i].pose.position.z )
            
            #print ("x", self.xhat[0],"y" ,self.xhat[1],"z" ,self.xhat[2])
            if num>0:
                loc_x = aruco_x/num
                loc_y = aruco_y/num
                loc_z = aruco_z/num
                self.pub_est(loc_x,loc_y,loc_z)


##############################
#### Main Function to Run ####
##############################
if __name__ == '__main__':
    # Initialize Node
    rospy.init_node('slam_estimator')

    # init path_manager_base object
    estimator = ekf_slam()

    while not rospy.is_shutdown():
        rospy.spin()
