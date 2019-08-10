# pointgray camera
This ros package read graypoint camera CM3-U3-13Y3C-CS and publish color picture.
## install flycapture

### download [flycapture]("https://flir.app.boxcn.net/v/Flycapture2SDK"), choose your adaptable version.

### install capture depandencies (Ubuntu 16.04)
'''
    sudo apt-get install libraw1394-11 libavcodec-ffmpeg56 libavformat-ffmpeg56 libswscale-ffmpeg3 libswresample-ffmpeg1 libavutil-ffmpeg54 libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev libusb-1.0-0
'''

### decompression the file you download just now 
'''
    cd your_flycapture_folder
    sudo sh install_flycapture.sh
'''

## run graypic
copy graypic ROS package to your ~/catkin_ws/src

'''
    cd ~/catkin_ws
    catkin_make --pkg graypic 
    roscore
    rosrun graypic graypic
'''

## rostopic 
 
## /graypic/image_raw
This is the picture this ros node published
