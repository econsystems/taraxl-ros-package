## Introduction to TaraXL

The TaraXL - See3CAM_StereoA is a UVC compliant USB 3.0 SuperSpeed Stereo vision camera from e-con Systems, a leading embedded Product Design Services company which specializes in the advanced camera solutions.

For More Information please visit:
https://www.e-consystems.com/3d-usb-stereo-camera-with-nvidia-accelerated-sdk.asp

STEEReoCAM™ is a 2MP 3D MIPI Stereo camera for NVIDIA® Jetson AGX Xavier™/TX2/Nano developer kit with improved accuracy and depth range. This MIPI Stereo camera is based on 1/2.9" OV2311 global shutter CMOS sensor from OmniVision. STEEReoCAM™ is bundled with a proprietary CUDA® accelerated Stereo SDK that runs on the GPU of NVIDIA® Tegra processors. It provides 3D depth mapping at ((2*1600) x 1300) resolution at 22 fps. 

For More Information please visit:
https://www.e-consystems.com/nvidia-cameras/jetson-agx-xavier-cameras/stereo-camera.asp

## Getting started

1. Download the latest version of the TaraXL SDK at https://developer.e-consystems.com
2. Install the TaraXL SDK on your NVIDIA TX2/Xavier/Nano device or in Linux x86 PC(with NVIDIA Card).

## Prerequisites

- Ubuntu 18.04
- [TARAXL SDK 3.2.2](https://developer.e-consystems.com) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)

## Build the program

The taraxl-ros-package is a catkin package. It depends on the following ROS packages:

   - roscpp
   - rosconsole
   - sensor_msgs
   - stereo_msgs
   - image_transport
   - dynamic_reconfigure

Open a terminal and build the package:
    
    cd ~/catkin/src
    git clone https://github.com/econsystems/taraxl-ros-package
    git clone https://github.com/econsystems/vision_opencv.git
    cd ~/catkin
    catkin_make
    source ./devel/setup.bash

For more information, please follow up the below wiki page

http://wiki.ros.org/taraxl-ros-package

## Published Topics

    /taraxl/left/image_rect - Rectified left image
    /taraxl/right/image_rect - Rectified right image
    /taraxl/left/image_raw - Unrectified left image
    /taraxl/right/image_raw - Unrectified right image 
    /taraxl/stereo/disparity/image - Disparity image
    /taraxl/depth/image - Depth image 
    /taraxl/stereo/pointcloud - pointcloud
    /taraxl/imu/data_raw - Raw IMU data - linear acceleration and angular velocity
    /taraxl/imu/inclination - IMU inclination data w.r.t 3 axes x,y and z
    /taraxl/left/calib/raw - Calibration informations for unrectified left image
    /taraxl/right/calib/raw - Calibration informations for unrectified right image
    /taraxl/left/calib/rect - Calibration informations for rectified left image
    /taraxl/right/calib/rect - Calibration informations for rectified right image

## Dynamic Reconfiguration Settings for TaraXL

     brightness : Controls brightness of the image (1-7)
     exposure : Manual exposure value (10-1000000)
     accuracy : Accuracy of the disparity image (0 - HIGH FRAME RATE, 1 - HIGH ACCURACY, 2 - ULTRA ACCURACY) 
     autoExposure : Enable auto exposure 
     pointcloudQuality : Quality of pointcloud(1 - HIGHEST, 2 - MEDIUM, 3 - STANDARD) 

## Dynamic Reconfiguration Settings for STEEREoCam

     brightness : Controls brightness of the image (1-10 - Works only when auto exposure is enabled) 
     exposure : Manual exposure value (1-7500)
     accuracy : Accuracy of the disparity image (0 - HIGH FRAME RATE, 1 - HIGH ACCURACY, 2 - ULTRA ACCURACY) 
     autoExposure : Enable auto exposure 
     gain : Controls the gain of the camera(1-240) 
     pointcloudQuality : Quality of pointcloud(1 - HIGHEST, 2 - MEDIUM, 3 - STANDARD) 

## Test Package

Open a terminal and enter the following command :

     roslaunch taraxl_ros_package taraxl.launch

To visualize TaraXL/STEEReoCAM image topics

	rqt_image_view

To visualize TaraXL/STEEReoCAM dispariy image topic

	rosrun image_view disparity_view image:=/taraxl/stereo/disparity/image

To visualize TaraXL/STEEReoCAM pointcloud topic

	rosrun rviz rviz

To view imu data

	rostopic echo /taraxl/imu/data_raw
	rostopic echo /taraxl/imu/inclination

To view calibration parameters for unrectified Frames

	rostopic echo /taraxl/left/calib/raw 
	rostopic echo /taraxl/right/calib/raw 

To view calibration parameters for rectified Frames

	rostopic echo /taraxl/left/calib/rect
	rostopic echo /taraxl/right/calib/rect 
	       
Dynamic reconfiguration

	rosrun rqt_reconfigure rqt_reconfigure

## Support

If you need assistance with the TaraXL/STEEREoCam, visit at https://www.e-consystems.com/create-ticket.asp or contact us at techsupport@e-consystems.com

