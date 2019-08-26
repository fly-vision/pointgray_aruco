# pointgray_aruco
pointgray camera and aruco detection

# install

### download flycapture
Download link [flycapture](https://flir.app.boxcn.net/v/Flycapture2SDK), choose your adaptable version.

For Ubuntu 16.04:
Select Linux/flycapture2-2.13.3.31-amd64-pkg_xenial.tgz

For Ubuntu 18.04:
Select Linux/flycapture2-2.13.3.31-amd64-pkg_bionic.tgz

### install capture depandencies (Ubuntu 16.04)
```
# for Ubuntu 16.04
sudo apt-get install libraw1394-11 libavcodec-ffmpeg56 libavformat-ffmpeg56 libswscale-ffmpeg3 libswresample-ffmpeg1 libavutil-ffmpeg54 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0
# for Ubuntu 18.04
sudo apt-get install libraw1394-11 libavcodec57 libavformat57 libswscale4 libswresample2 libavutil55 libgtkmm-2.4-1v5  libglademm-2.4-1v5 libgtkglextmm-x11-1.2-0v5 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0
```
### decompression the file you download just now 

```
cd your_flycapture_folder
sudo sh install_flycapture.sh
```

# How to run
```
roscore
rosrun graypic graypic
rosrun aruco aruco_ros
# available topic
# /graypic/image_raw   相机图像话题
# /aruco/detection_vis   二维码检测可视化
# /aruco/markers    二维码消息，位置、姿态、编号
```

# 运行

# 运行 ekf_slam3D_ljn.py
按照要求安装fly-vision/xsens_ros_mti_driver
然后依次运行
```
roslaunch xsens_mti_driver xsens_mti_node.launch
rosrun graypic graypic
rosrun aruco aruco_ros
python ekf_slam3D_ljn.py
```
# 运行 Odometry_pub.py
Odometry_pub根据aurco信息计算UAV的位置信息,其中无人机坐标系为相机坐标系的z轴指向下,二维码分布的坐标系为北东天的左手系,输出的位置的坐标系为的东北天
接收消息
```
/aruco/markers
```
发布消息
```
/ekf_estimate/path
/ekf_estimate/path
```
