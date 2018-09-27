## Introduction to TaraXL

The TaraXL - See3CAM_StereoA is a UVC compliant USB 3.0 SuperSpeed Stereo vision camera from e-con Systems, a leading embedded Product Design Services company which specializes in the advanced camera solutions.

For More Information please visit:
https://www.e-consystems.com/3d-usb-stereo-camera-with-nvidia-accelerated-sdk.asp


## Getting started

1. Download the latest version of the TaraXL SDK at https://developer.e-consystems.com
2. Install the TaraXL SDK on you NVIDIA TX2 device or in Linux x86 PC(with NVIDIA Card).


## Prerequisites

- Ubuntu 16.04
- [TARAXL SDK 1.0.0](https://developer.e-consystems.com) and its dependency [CUDA](https://developer.nvidia.com/cuda-downloads)
- [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)

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
    /taraxl/stereo/disparity/image - Disparity image
    /taraxl/depth/image - Depth image 

## Dynamic Reconfiguration Settings

     brightness : Controls brightness of the image (1-7)
     exposure : Manual exposure value (10-1000000)
     accuracy : Accuracy of the disparity image.
     auto_exposure : Enable auto exposure 

## Test Package

Open a terminal and enter the following command :

     roslaunch taraxl_ros_package taraxl.launch

To visualize TaraXL topics, open a terminal and enter the following command:

     rqt_image_view

## Support

If you need assistance with the TaraXL, visit at https://www.e-consystems.com/Request-form.asp?paper=see3cam_stereoa or contact us at techsupport@e-consystems.com
