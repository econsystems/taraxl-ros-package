#include "TaraXLRos.h"

float maxDisp = 64;
enum accuracyMode {HIGH_FRAMERATE,HIGH_ACCURACY,ULTRA_ACCURACY};
accuracyMode currentAccuracy = HIGH_ACCURACY;

void taraxlros::rosPublish ()
{

	status = cam.enumerateDevices (camList);

	if (status != TARAXL_SUCCESS)
	{
		cout << "Camera enumeration failed" << endl;
		exit (0);
	}


	if (camList.size ()==0)
	{
		cout << "No cameras connected" << endl;
		exit (0);
	}

	taraxlCam = camList.at (0);

	status = taraxlCam.connect ();


	if (status != TARAXL_SUCCESS)
	{
		cout << "Camera connect failed" << endl;
		exit (0);
	}


	taraxlDepth = new TaraXLDepth (taraxlCam);

	if (taraxlDepth == NULL)
	{
		cout << "Unable to create instance to TaraXLDepth" << endl;
    		exit (0);
	}

	taraxlPose = new TaraXLPoseTracking(taraxlCam);

	if (taraxlPose == NULL)
	{
		cout << "Unable to create instance to TaraXLPose" << endl;
    		exit (0);
	}

	string cameraName;
	status = taraxlCam.getFriendlyName(cameraName);

	if (cameraName == "See3CAM_StereoA")
		maxDisp = 128;
	else
		maxDisp = 64;


	image_transport::ImageTransport itTaraXL (nodeHandle);


	//publishers-advertise to topics
	pubLeft = itTaraXL.advertise ("left/image_rect", 1);
	pubRight = itTaraXL.advertise ("right/image_rect", 1);
	pubDisparity = nodeHandle.advertise<stereo_msgs::DisparityImage> ("stereo/disparity/image", 1);
	pubDepth = itTaraXL.advertise ("depth/image", 1);
	pubPointCloud = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("stereo/pointcloud", 1);
	pubImu = nodeHandle.advertise<sensor_msgs::Imu>("imu/data_raw",1);
	pubInclination = nodeHandle.advertise<geometry_msgs::Point>("imu/inclination",1);


	image_thread = std::thread(std::bind(&taraxlros::getImages,this));
	imu_thread = std::thread(std::bind(&taraxlros::getImu,this));

}


void taraxlros::imagePublisher(sensor_msgs::ImagePtr &imageMsg, Mat image,std::string frame)
{

	int num = 1; // for endianness detection

	//Convert to ROS message
	imageMsg = cv_bridge::CvImage (std_msgs::Header (), "mono8", image).toImageMsg ();
	if (imageMsg == NULL)
	{
		cout << "Failed to get image "<< endl;
		exit (0);
	}

	imageMsg->header.stamp = ros::Time::now ();
	imageMsg->header.frame_id = frame;
	imageMsg->is_bigendian = ! (* (char*)&num == 1);

}


void taraxlros::disparityPublisher(stereo_msgs::DisparityImagePtr &dispMsg, Mat dispImage)
{

	int num = 1; // for endianness detection

	dispImage.convertTo(dispImage,CV_8U);
	
	Mat QMat;
	status = taraxlCam.getQMatrix(QMat);

	//Convert to ROS message
	dispMsg->min_disparity = 0;
	dispMsg->max_disparity = maxDisp;

	dispMsg->header.stamp = ros::Time::now ();
	dispMsg->header.frame_id = "taraxl_left";
	sensor_msgs::Image& dimage = dispMsg->image;
	
	dimage.width  = dispImage.size ().width ;
	dimage.height = dispImage.size ().height ;
	dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	dimage.step = dimage.width * sizeof (float);
	dimage.data.resize (dimage.step * dimage.height);

	cv::Mat_<float> dmat (dimage.height, dimage.width, reinterpret_cast<float*> (&dimage.data[0]), dimage.step);
	dispImage.copyTo(dmat);
//	dispImage.convertTo (dmat, dmat.type ());

	dispMsg->image.header = dispMsg->header;
	dispMsg->image.is_bigendian = ! (* (char*)&num == 1);
	dispMsg->valid_window.x_offset = 0;
	dispMsg->valid_window.y_offset = 0;
	dispMsg->valid_window.width    = 0;
	dispMsg->valid_window.height   = 0;
	dispMsg->T		       = ((double)1.0f / QMat.at<double>(3, 2)) * 1000;
	dispMsg->f		       = QMat.at<double>(2, 3);
	dispMsg->delta_d               = 0;

	if (dispMsg == NULL)
	{
		cout << "Failed to get disparity image "<< endl;
	  	exit (0);
	}


}

//Dynamic reconfiguration parameters for TaraXL camera
void taraxlros::dynamicReconfCallback (taraxl_ros_package::taraxlrosConfig &config, uint32_t level)
{

	enum SETTINGS {BRIGHTNESS, EXPOSURE, ACCURACY, AUTOEXPOSURE, PCLQUALITY};

	switch (level)
	{

		case BRIGHTNESS:
			status = taraxlCam.setBrightness (config.brightness);
			break;

		case EXPOSURE:
			config.autoExposure = false;
			status = taraxlCam.setExposure (config.exposure);
			break;

		case ACCURACY:
			switch (config.accuracy)
			{		
				case 0:
					status = taraxlDepth->setAccuracy (TaraXLSDK::LOW);
	                                currentAccuracy = HIGH_FRAMERATE;
        	                        maxDisp = 64;
					break;
				case 1:
					status = taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
        	                        currentAccuracy = HIGH_ACCURACY;
	                                maxDisp = 128;
					break;
				case 2:
					status = taraxlDepth->setAccuracy (TaraXLSDK::ULTRA);
	                                currentAccuracy = ULTRA_ACCURACY;
        	                        maxDisp = 128;
					break;
			}

			break;

		case AUTOEXPOSURE:
			if (config.autoExposure)
				status = taraxlCam.enableAutoExposure ();
			else
				status = taraxlCam.setExposure (config.exposure);
			break;

		case PCLQUALITY:
			switch(config.pointcloudQuality)
			{
				case 1:
					status = taraxl3d->setPointcloudQuality(TaraXLSDK::HIGHEST);
					break;
				case 2:
					status = taraxl3d->setPointcloudQuality(TaraXLSDK::MEDIUM);
					break;
				case 3:
					status = taraxl3d->setPointcloudQuality(TaraXLSDK::STANDARD);
					break;
			}
			break;

	}

}

//Dynamic reconfiguration parameters for STEEReoCAM camera
void taraxlros::dynamicReconfCallbackSteereocam (taraxl_ros_package::steereocamrosConfig &config, uint32_t level)
{

	enum SETTINGS {BRIGHTNESS, EXPOSURE, ACCURACY, AUTOEXPOSURE, PCLQUALITY, GAIN};

	switch (level)
	{

		case BRIGHTNESS:
			status = taraxlCam.setBrightness (config.brightness);
			break;

		case EXPOSURE:
			config.autoExposure = false;
			status = taraxlCam.setExposure (config.exposure);
			break;

		case ACCURACY:
			switch (config.accuracy)
			{
				case 0:
					status = taraxlDepth->setAccuracy (TaraXLSDK::LOW);
	                                currentAccuracy = HIGH_FRAMERATE;
        	                        maxDisp = 64;
					break;
				case 1:
					status = taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
        	                        currentAccuracy = HIGH_ACCURACY;
	                                maxDisp = 64;
					break;
				case 2:
					status = taraxlDepth->setAccuracy (TaraXLSDK::ULTRA);
	                                currentAccuracy = ULTRA_ACCURACY;
        	                        maxDisp = 128;
					break;

			}
			break;

		case AUTOEXPOSURE:
			if (config.autoExposure)
				status = taraxlCam.enableAutoExposure ();
			else
				status = taraxlCam.setExposure (config.exposure);
			break;

		case PCLQUALITY:

			switch(config.pointcloudQuality)
			{
				case 1:
					status = taraxl3d->setPointcloudQuality(TaraXLSDK::HIGHEST);
					break;
				case 2:
					status = taraxl3d->setPointcloudQuality(TaraXLSDK::MEDIUM);
					break;
				case 3:
					status = taraxl3d->setPointcloudQuality(TaraXLSDK::STANDARD);
					break;
			}

			break;
		
		case GAIN:
			taraxlCam.setGain(config.gain);
			break;

	}

}


void taraxlros::getImages()
{

		int num = 1; // for endianness detection

		//openCV Mat objects
		Mat left, right, grayDisp, depthMap;


		//ROS messages
		sensor_msgs::ImagePtr leftMsg;
		sensor_msgs::ImagePtr rightMsg;
		sensor_msgs::ImagePtr depthMsg;
		stereo_msgs::DisparityImagePtr disparityMsg = boost::make_shared<stereo_msgs::DisparityImage> ();

		//Dynamic Reconfiguration

		string cameraName;
		taraxlCam.getFriendlyName (cameraName);
		
		if (cameraName == "See3CAM_StereoA")
		{
			dynamic_reconfigure::Server<taraxl_ros_package::taraxlrosConfig> *s = new dynamic_reconfigure::Server<taraxl_ros_package::taraxlrosConfig>(nodeHandle);
		
		        dynamic_reconfigure::Server<taraxl_ros_package::taraxlrosConfig>::CallbackType settings;	
			settings = boost::bind (&taraxlros::dynamicReconfCallback, this, _1, _2);
			s->setCallback (settings);
		}
		
		else
		{
			dynamic_reconfigure::Server<taraxl_ros_package::steereocamrosConfig> *s = new dynamic_reconfigure::Server<taraxl_ros_package::steereocamrosConfig>(nodeHandle);

         	        dynamic_reconfigure::Server<taraxl_ros_package::steereocamrosConfig>::CallbackType settings2;
			settings2 = boost::bind (&taraxlros::dynamicReconfCallbackSteereocam, this, _1, _2);
			s->setCallback (settings2);
		}

		int dispImgSuber, depthImgSuber, leftImgSuber , rightImgSuber, pointCloudSuber;
		dispImgSuber = depthImgSuber = leftImgSuber = rightImgSuber = pointCloudSuber = 0;

		std::string leftFrame = "taraxl_left";
		std::string rightFrame = "taraxl_right";

		taraxl3d = new TaraXLPointcloud(taraxlCam);
		if (taraxl3d == NULL)
		{
			cout<<"Unable to create instance to TaraXLPoints";
			exit(0);
		}

		Points::Ptr currentCloud (new Points);

		ROS_INFO ("TaraXL ROS running");

		Mat colorDisp;

		while (ros::ok ())
		{

			//Get number of subscribers
			dispImgSuber = pubDisparity.getNumSubscribers ();
			depthImgSuber = pubDepth.getNumSubscribers ();
			leftImgSuber = pubLeft.getNumSubscribers ();
			rightImgSuber = pubRight.getNumSubscribers ();
			pointCloudSuber = pubPointCloud.getNumSubscribers();

			//Obtain and publish pointcloud if subscribed
			if (pointCloudSuber > 0)
			{
					status = taraxl3d->getPoints (currentCloud); //SDK function to obtain Point Cloud
					currentCloud->header.frame_id = "taraxl_left";
					pcl_conversions::toPCL(ros::Time::now(), currentCloud->header.stamp);

					pubPointCloud.publish(currentCloud); //Publish point cloud
	 		}


			//Obtain and publish disparity and depth image if both are subscribed
			if ( dispImgSuber > 0 && depthImgSuber > 0 )
			{

				if (currentAccuracy == HIGH_ACCURACY)
					taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
				else if (currentAccuracy == HIGH_FRAMERATE)
					taraxlDepth->setAccuracy (TaraXLSDK::LOW);
				else if (currentAccuracy == ULTRA_ACCURACY)
					taraxlDepth->setAccuracy (TaraXLSDK::ULTRA);

				status = taraxlDepth->getMap (left, right, grayDisp, true, depthMap, true);  //SDK function to obtain Disparity image

				if (status != TARAXL_SUCCESS)
				{
					cout << "Failed to get Disparity and depth image" << endl;
					exit (0);
				}


				//Publish disparity image
				disparityPublisher(disparityMsg, grayDisp);
				pubDisparity.publish (disparityMsg);

				//Publish depth map
				depthMap.convertTo (depthMap, CV_8UC1);
				imagePublisher (depthMsg, depthMap, leftFrame);
				pubDepth.publish (depthMsg);

			}

			//Obtain and publish disparity image if subscribed
			else if (dispImgSuber > 0)
			{

				if (currentAccuracy == HIGH_ACCURACY)
					taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
				else if (currentAccuracy == HIGH_FRAMERATE)
					taraxlDepth->setAccuracy (TaraXLSDK::LOW);
				else if (currentAccuracy == ULTRA_ACCURACY)
					taraxlDepth->setAccuracy (TaraXLSDK::ULTRA);

				status = taraxlDepth->getMap (left, right, grayDisp, true, depthMap, false);
				if (status != TARAXL_SUCCESS)
				{
					cout << "Failed to get Disparity image" << endl;
					exit (0);
				}

				//Publish disparity image
				disparityPublisher(disparityMsg, grayDisp);
				pubDisparity.publish (disparityMsg);

			}

			//Obtain and publish depth image if subscribed
			else if (depthImgSuber > 0)
			{

				if (currentAccuracy == HIGH_ACCURACY)
					taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
				else if (currentAccuracy == HIGH_FRAMERATE)
					taraxlDepth->setAccuracy (TaraXLSDK::LOW);
				else if (currentAccuracy == ULTRA_ACCURACY)
					taraxlDepth->setAccuracy (TaraXLSDK::ULTRA);

				status = taraxlDepth->getMap (left, right, grayDisp, false, depthMap, true);  //SDK function to obtain Depth image

				if (status != TARAXL_SUCCESS)
				{

					cout << "Failed to get Depth image" << endl;
					exit (0);
				}

				depthMap.convertTo (depthMap, CV_8UC1);
				
				//Publish depth image
				imagePublisher (depthMsg, depthMap, leftFrame);
				pubDepth.publish(depthMsg);

			}

			//Obtain and publish left/right image if subscribed
			else if (leftImgSuber > 0 || rightImgSuber > 0)
			{

				status = taraxlDepth->getMap (left, right, grayDisp, false, depthMap, false);  //SDK function to obtain left and right image

				if (status != TARAXL_SUCCESS)
				{
					cout << "Failed to get left and right images" << endl;
					exit (0);
				}

			}

			//Publish left and right image
			imagePublisher (leftMsg, left, leftFrame);
			imagePublisher (rightMsg, right, rightFrame);
			pubLeft.publish (leftMsg);
			pubRight.publish (rightMsg);

			ros::spinOnce ();

		}


}

void taraxlros::getImu()
{

	ROS_INFO ("TaraXL ROS for IMU running");

	sensor_msgs::Imu imuMsg;

	geometry_msgs::Point inclination;

	struct TaraXLIMUData imuData;

	Vector3 inclnData;

	while(ros::ok())
	{

		status = taraxlPose->getIMUData(imuData); //SDK function to obtain imu data

		imuMsg.angular_velocity.x =  imuData.angularVelocity[0] * 0.0174533;
		imuMsg.angular_velocity.y =  imuData.angularVelocity[1] * 0.0174533;
		imuMsg.angular_velocity.z =  imuData.angularVelocity[2] * 0.0174533;

		imuMsg.linear_acceleration.x = imuData.linearAcceleration[0] * 9.80665 / 1000;
		imuMsg.linear_acceleration.y = imuData.linearAcceleration[1] * 9.80665 / 1000;
		imuMsg.linear_acceleration.z = imuData.linearAcceleration[2] * 9.80665 / 1000;

		imuMsg.header.frame_id = "taraxl_IMU";
		imuMsg.header.stamp = ros::Time::now ();

		imuMsg.orientation_covariance[0] = -1;

		inclnData = imuData.getInclination ();
		inclination.x =  inclnData [0] ;
		inclination.y =  inclnData [1] ;
		inclination.z =  inclnData [2] ;


		pubInclination.publish (inclination);   //publish imu inclination
		pubImu.publish (imuMsg);   //publish imu data

	}

}

taraxlros::~taraxlros ()
{
	image_thread.join();
	imu_thread.join();
	delete taraxlDepth;
	delete taraxl3d;
	taraxlCam.disconnect();
}

int main (int argc, char **argv)
{

	ros::init (argc, argv, "taraxl_ros_package");

	taraxlros roswrapper;

	roswrapper.rosPublish ();

	return 0;

}
