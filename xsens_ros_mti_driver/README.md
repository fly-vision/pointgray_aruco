# xsens_ros_mti_driver
这个ros软件包是用来读取Mti-G-710惯导元件的数据，并发布一个ros的IMU消息

## dependencies
*1 MT SDK from Xsens MT Software Suite 2019.0.1 or later
download [MT SDK](),choose Linux. Extract the downloaded installation package(MT_Software_Suite_linux-x64_2019.tar.gz), execute the mtsdk_linux-x##_####.#.sh
```
cd  {MT_software_folder}
sudo apt-get install sharutils
sudo sh mtsdk_linux-x64_2019.0.1.sh
```
* 2 ROS Kinetic or Melodic
* 3 C/C++ Compiler: GCC 5.4.0 or MSVC 14.0
* 4 C++11

## Building
Copy **this package** into your catkin workspace 'src' folder.Make sure the permissions are set to o+rw on your files and directories.

Build xspublic from your catkin workspace:
```
pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
```
Build Xsens MTi driver package:
```
catkin_make 
```
Source workspace:
```
source devel/setup.bash
```
## Running:
Configure your MTi device to output desired data use

Launch the Xsens MTi driver from your catkin workspace:
```
roslaunch xsens_mti_driver xsens_mti_node.launch
```
## ros topic
The ROS topic of imu date is /imu/data
