#include <iostream>

//OpenCV Headers
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/cuda.hpp>


//ROS Headers
#include <ros/ros.h>
#include <ros/time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stereo_msgs/DisparityImage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


#include<dynamic_reconfigure/server.h>
#include<taraxl_ros_package/taraxlrosConfig.h>


//TaraXL headers
#include "TaraXL.h"
#include "TaraXLCam.h"
#include "TaraXLDepth.h"
#include "TaraXLEnums.h"

using namespace std;
using namespace cv;
using namespace TaraXLSDK;


class taraxlros 
{

	
		//SDK Class objects
       		TaraXL cam; 
       		TaraXLCam taraxlCam;
       		TaraXLCamList camList;
       		TaraXLDepth *taraxlDepth;	
		TARAXL_STATUS_CODE status;
		
		ros::NodeHandle nodeHandle;
	
		//publishers
		image_transport::Publisher pubLeft;
		image_transport::Publisher pubRight;
		ros::Publisher pubDisparity;
		image_transport::Publisher pubDepth;
		
	public:
	
		
		void dynamicReconfCallback (taraxl_ros_package::taraxlrosConfig &config, uint32_t level);
		
		void rosPublish ();
		void imagePublisher(sensor_msgs::ImagePtr &imageMsg, Mat image);
		void disparityPublisher(stereo_msgs::DisparityImagePtr &dispMsg, Mat dispImage);
		

};
