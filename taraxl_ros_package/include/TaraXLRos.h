#include <iostream>
#include<thread>

//OpenCV Headers
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

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
#include<taraxl_ros_package/steereocamrosConfig.h>

//IMU headers
#include "geometry_msgs/Point.h"
#include "sensor_msgs/Imu.h"
#include "boost/thread/thread.hpp"
#include <std_msgs/Bool.h>


//Point cloud headers
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


//TaraXL headers
#include "TaraXL.h"
#include "TaraXLCam.h"
#include "TaraXLDepth.h"
#include "TaraXLEnums.h"
#include "TaraXLPoseTracking.h"
#include "TaraXLPointcloud.h"

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
		TaraXLPoseTracking *taraxlPose;
		TaraXLPointcloud *taraxl3d;

		ros::NodeHandle nodeHandle;

		//publishers
		image_transport::Publisher pubLeft;
		image_transport::Publisher pubRight;
		ros::Publisher pubDisparity;
		image_transport::Publisher pubDepth;

		ros::Publisher pubPointCloud;
		ros::Publisher pubImu;
		ros::Publisher pubInclination;

	public:

		void dynamicReconfCallback (taraxl_ros_package::taraxlrosConfig &config, uint32_t level);
		void dynamicReconfCallbackSteereocam (taraxl_ros_package::steereocamrosConfig &config, uint32_t level);

		void rosPublish ();
		void imagePublisher(sensor_msgs::ImagePtr &imageMsg, Mat image,std::string frame);
		void disparityPublisher(stereo_msgs::DisparityImagePtr &dispMsg, Mat dispImage);

		void getImu();
		void getImages();

		std::thread image_thread;
		std::thread imu_thread;


		~taraxlros ();

};
