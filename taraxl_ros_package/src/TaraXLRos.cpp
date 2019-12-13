#include "TaraXLRos.h"

int maxDisp;
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
                taraxlDepth->getMaxDisparity(maxDisp);
	else
		taraxlDepth->getMaxDisparity(maxDisp);


	image_transport::ImageTransport itTaraXL (nodeHandle);


	//publishers-advertise to topics
	pubLeft = itTaraXL.advertise ("left/image_rect", 1);
	pubRight = itTaraXL.advertise ("right/image_rect", 1);
        pubLeftRaw=itTaraXL.advertise ("left/image_raw", 1);
        pubRightRaw=itTaraXL.advertise ("right/image_raw", 1);
	pubDisparity = nodeHandle.advertise<stereo_msgs::DisparityImage> ("stereo/disparity/image", 1);
	pubDepth = itTaraXL.advertise ("depth/image", 1);
	pubPointCloud = nodeHandle.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("stereo/pointcloud", 1);
	pubImu = nodeHandle.advertise<sensor_msgs::Imu>("imu/data_raw",1);
        pubLeftRawCalib = nodeHandle.advertise<sensor_msgs::CameraInfo>("left/calib/raw",1);
        pubRightRawCalib = nodeHandle.advertise<sensor_msgs::CameraInfo>("right/calib/raw",1);
        pubLeftRectCalib = nodeHandle.advertise<sensor_msgs::CameraInfo>("left/calib/rect",1);
        pubRightRectCalib = nodeHandle.advertise<sensor_msgs::CameraInfo>("right/calib/rect",1);
	pubInclination = nodeHandle.advertise<geometry_msgs::Point>("imu/inclination",1);


	image_thread = std::thread(std::bind(&taraxlros::getImages,this));
	imu_thread = std::thread(std::bind(&taraxlros::getImu,this));
        calib_thread = std::thread(std::bind(&taraxlros::getCalib,this));

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
	dispMsg->max_disparity = taraxlDepth->getMaxDisparity(maxDisp);

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
        	                        taraxlDepth->getMaxDisparity(maxDisp);
					break;
				case 1:
					status = taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
        	                        currentAccuracy = HIGH_ACCURACY;
	                                taraxlDepth->getMaxDisparity(maxDisp);
					break;
				case 2:
					status = taraxlDepth->setAccuracy (TaraXLSDK::ULTRA);
	                                currentAccuracy = ULTRA_ACCURACY;
        	                        taraxlDepth->getMaxDisparity(maxDisp);
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
        	                        taraxlDepth->getMaxDisparity(maxDisp);
					break;
				case 1:
					status = taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
        	                        currentAccuracy = HIGH_ACCURACY;
	                                taraxlDepth->getMaxDisparity(maxDisp);
					break;
				case 2:
					status = taraxlDepth->setAccuracy (TaraXLSDK::ULTRA);
	                                currentAccuracy = ULTRA_ACCURACY;
        	                        taraxlDepth->getMaxDisparity(maxDisp);
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



void taraxlros::publishImage (sensor_msgs::ImagePtr leftMsg,Mat left,std::string leftFrame,sensor_msgs::ImagePtr rightMsg,Mat right,std::string rightFrame)
{
                        imagePublisher (leftMsg, left, leftFrame);
			imagePublisher (rightMsg, right, rightFrame);
			pubLeft.publish (leftMsg);
			pubRight.publish (rightMsg);

}

void taraxlros::getImages()
{

		int num = 1; // for endianness detection

		//openCV Mat objects
		Mat left, right, grayDisp, depthMap,leftRaw,rightRaw;
                

		//ROS messages
		sensor_msgs::ImagePtr leftMsg;
		sensor_msgs::ImagePtr rightMsg;
		sensor_msgs::ImagePtr depthMsg;
                sensor_msgs::ImagePtr leftrawMsg;
                sensor_msgs::ImagePtr rightrawMsg;
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

		int dispImgSuber, depthImgSuber, leftImgSuber , rightImgSuber, pointCloudSuber,leftRawSuber,rightRawSuber;
		dispImgSuber = depthImgSuber = leftImgSuber = rightImgSuber = pointCloudSuber = leftRawSuber = rightRawSuber= 0;

		std::string leftFrame = "taraxl_left";
		std::string rightFrame = "taraxl_right";
                std::string leftRawFrame="taraxl_leftRaw";
                std::string rightRawFrame="taraxl_righRaw";   

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
                        leftRawSuber=pubLeftRaw.getNumSubscribers ();
                        rightRawSuber=pubRightRaw.getNumSubscribers ();
			pointCloudSuber = pubPointCloud.getNumSubscribers();

			//Obtain and publish pointcloud if subscribed
			if (pointCloudSuber > 0)
			{
					status = taraxl3d->getPoints (currentCloud); //SDK function to obtain Point Cloud
					currentCloud->header.frame_id = "taraxl";
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
                                publishImage(leftMsg,left,leftFrame,rightMsg, right,rightFrame);

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
                                publishImage(leftMsg,left,leftFrame,rightMsg, right,rightFrame);

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
                                publishImage(leftMsg,left,leftFrame,rightMsg, right,rightFrame);

			}

			//Obtain and publish left/right image if subscribed
			else if (leftImgSuber > 0 || rightImgSuber > 0)
			{

				status = taraxlDepth->getMap (left, right, grayDisp, false, depthMap, false); //SDK function to obtain left and right image

				if (status != TARAXL_SUCCESS)
				{
					cout << "Failed to get left and right images" << endl;
					exit (0);
				}
                                publishImage(leftMsg,left,leftFrame,rightMsg, right,rightFrame);

			}
                        //Publish left and right Unrectified image
                        else if(leftRawSuber >0 || rightRawSuber >0)
                        {
                                
                                status = taraxlCam.getUnrectifiedFrame (leftRaw,rightRaw);  //SDK function to obtain left and right Unrectified image 

				if (status != TARAXL_SUCCESS)
				{
					cout << "Failed to get left and right unrectified images" << endl;
					exit (0);
				} 
                               
                                imagePublisher (leftrawMsg,leftRaw,leftRawFrame);
                                imagePublisher(rightrawMsg,rightRaw,rightRawFrame);
                                pubLeftRaw.publish(leftrawMsg);
                                pubRightRaw.publish(rightrawMsg);
                        }
		   
			
                       
                        

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
                //0.0174533 is used for degree to radian  conversion
		imuMsg.angular_velocity.x =  imuData.angularVelocity[0] * 0.0174533;
		imuMsg.angular_velocity.y =  imuData.angularVelocity[1] * 0.0174533;
		imuMsg.angular_velocity.z =  imuData.angularVelocity[2] * 0.0174533;
                //9.80665 / 1000  is used for g to m/s2  conversion
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


void taraxlros::fillCalib(sensor_msgs::CameraInfo &leftCalibMsg,sensor_msgs::CameraInfo &rightCalibMsg,Mat R,Mat T,CalibrationParams left,CalibrationParams right,int width ,int height,string frameId)
{
        Mat qMat;
	leftCalibMsg.distortion_model =
            	sensor_msgs::distortion_models::PLUMB_BOB;
                //Distortion parameters
        leftCalibMsg.D=left.distortionMatrix;

                //Intrinsic camera matrix
	leftCalibMsg.K.fill(0);
        leftCalibMsg.K[0]=left.cameraMatrix.at<double>(0);
        leftCalibMsg.K[2]=left.cameraMatrix.at<double>(2);
        leftCalibMsg.K[4]=left.cameraMatrix.at<double>(4);
        leftCalibMsg.K[5]=left.cameraMatrix.at<double>(5);
        leftCalibMsg.K[8]=left.cameraMatrix.at<double>(8);

		//Rotation matrix
        for(int i=0;i<9;i++)
        {
        	leftCalibMsg.R[i]=R.at<double>(i);
        }
                        //Projection/camera matrix
        leftCalibMsg.P.fill(0.0);
        leftCalibMsg.P[0]=left.cameraMatrix.at<double>(0);

        leftCalibMsg.P[2]=left.cameraMatrix.at<double>(2);

        leftCalibMsg.P[4]=left.cameraMatrix.at<double>(4);

        leftCalibMsg.P[5]=left.cameraMatrix.at<double>(5);

        leftCalibMsg.P[10]=1.0;

        leftCalibMsg.width  =width;
    	leftCalibMsg.height =height;
        leftCalibMsg.header.frame_id =frameId+"Left";

        status = taraxlCam.getQMatrix(qMat);
        rightCalibMsg.distortion_model =
            sensor_msgs::distortion_models::PLUMB_BOB;
                        //Distortion parameters
        rightCalibMsg.D=right.distortionMatrix;

			//Intrinsic camera matrix
	rightCalibMsg.K.fill(0);
        rightCalibMsg.K[0]=right.cameraMatrix.at<double>(0);
        rightCalibMsg.K[2]=right.cameraMatrix.at<double>(2);
        rightCalibMsg.K[4]=right.cameraMatrix.at<double>(4);
        rightCalibMsg.K[5]=right.cameraMatrix.at<double>(5);
        rightCalibMsg.K[8]=right.cameraMatrix.at<double>(8);
                        //Rotation matrix
        for(int i=0;i<9;i++)
        {
         	rightCalibMsg.R[i]=R.at<double>(i);
        }
                        //Projection/camera matrix
        rightCalibMsg.P.fill(0.0);
        rightCalibMsg.P[3]=(-1 *right.cameraMatrix.at<double>(0) * qMat.at<double>(3,2));
        rightCalibMsg.P[0]=right.cameraMatrix.at<double>(0);
        rightCalibMsg.P[2]=right.cameraMatrix.at<double>(2);
        rightCalibMsg.P[4]=right.cameraMatrix.at<double>(4);
        rightCalibMsg.P[5]=right.cameraMatrix.at<double>(5);
        rightCalibMsg.P[10]=1.0;
        rightCalibMsg.width  =width;
        rightCalibMsg.height =height;

        rightCalibMsg.header.frame_id =frameId+"Right" ;

}


void taraxlros::getCalib()
{

        uint32_t seqLeftRaw=0,seqRightRaw=0,seqLeftRect=0,seqRightRect=0;
        int leftCalibRawSuber=0,rightCalibRawSuber=0,leftCalibRectSuber=0,rightCalibRectSuber=0;
             
        sensor_msgs::CameraInfo leftRawCalibMsg,rightRawCalibMsg,leftRectCalibMsg,rightRectCalibMsg;
        ROS_INFO ("TaraXL ROS for calib running");
        Mat R,T,QMat;
        struct CalibrationParams leftRawCalib,rightRawCalib,leftRectCalib,rightRectCalib;
        struct Resolution Resolution;
        status = taraxlCam.getResolution(Resolution);
        status=taraxlCam.getCalibrationParameters(R,T,leftRawCalib,rightRawCalib,leftRectCalib,rightRectCalib);
        fillCalib(leftRawCalibMsg,rightRawCalibMsg,R,T,leftRawCalib,rightRawCalib,Resolution.width,Resolution.height,"taraXLCalibRaw");
        fillCalib(leftRectCalibMsg,rightRectCalibMsg,R,T,leftRectCalib,rightRectCalib,Resolution.width,Resolution.height,"taraXLCalibRect");

        while(ros::ok()){
        	leftCalibRawSuber=pubLeftRawCalib.getNumSubscribers ();
        	rightCalibRawSuber=pubRightRawCalib.getNumSubscribers ();
                leftCalibRectSuber=pubLeftRectCalib.getNumSubscribers ();
        	rightCalibRectSuber=pubRightRectCalib.getNumSubscribers ();
        	if(leftCalibRawSuber > 0)
		{
        	
                	leftRawCalibMsg.header.stamp = ros::Time::now ();
        		leftRawCalibMsg.header.seq =seqLeftRaw ;
        		pubLeftRawCalib.publish(leftRawCalibMsg);//Publish left Camera information(calibration parameter for left raw image)
        		seqLeftRaw++;	
        	}
        	if( rightCalibRawSuber>0)
        	{
         		
               		rightRawCalibMsg.header.stamp = ros::Time::now ();
        		rightRawCalibMsg.header.seq =seqRightRaw ;
        		pubRightRawCalib.publish(rightRawCalibMsg);//Publish right camera information(calibration parameter for right raw image)
        		seqRightRaw++;
        	     
        	}
                if(leftCalibRectSuber > 0)
		{
        	
                	leftRectCalibMsg.header.stamp = ros::Time::now ();
        		leftRectCalibMsg.header.seq =seqLeftRect;
        		pubLeftRectCalib.publish(leftRectCalibMsg);//Publish left Camera information(calibration parameter for left rectifide image)
        		seqLeftRect++;	
        	}
        	if( rightCalibRectSuber>0)
        	{
         		
               		rightRectCalibMsg.header.stamp = ros::Time::now ();
        		rightRectCalibMsg.header.seq =seqRightRect ;
        		pubRightRectCalib.publish(rightRectCalibMsg);//Publish right camera information(calibration parameter for right rectifide image)
        		seqRightRect++;

        	}
              
	}
       
	
             
}

taraxlros::~taraxlros ()
{
	image_thread.join();
	imu_thread.join();
        calib_thread.join();
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
