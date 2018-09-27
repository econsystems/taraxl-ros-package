#include "TaraXLRos.h"

float maxDisp = 64;

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
		cout << "Unable to create instance to TaraDepth" << endl;
    		exit (0);
	}	

	ROS_INFO ("TaraXL ROS running");


	image_transport::ImageTransport itTaraXL (nodeHandle);

	//openCV Mat objects
	Mat left, right, grayDisp, depthMap;	


	//ROS messages
	sensor_msgs::ImagePtr leftMsg;
	sensor_msgs::ImagePtr rightMsg;
	sensor_msgs::ImagePtr depthMsg;
	stereo_msgs::DisparityImagePtr disparityMsg = boost::make_shared<stereo_msgs::DisparityImage> ();
	

	//publishers-advertise to topics
	pubLeft = itTaraXL.advertise ("left/image_rect", 1);
	pubRight = itTaraXL.advertise ("right/image_rect", 1);
	pubDisparity = nodeHandle.advertise<stereo_msgs::DisparityImage> ("stereo/disparity/image", 1);
	pubDepth = itTaraXL.advertise ("depth/image", 1);


	//Dynamic Reconfiguration
	dynamic_reconfigure::Server<taraxl_ros_package::taraxlrosConfig> server;	
	dynamic_reconfigure::Server<taraxl_ros_package::taraxlrosConfig>::CallbackType settings;
    	settings = boost::bind (&taraxlros::dynamicReconfCallback, this, _1, _2);
   	server.setCallback (settings);
	

	while (ros::ok ())
	{

		//Get number of subscribers
		int dispImgSuber = pubDisparity.getNumSubscribers ();
		int depthImgSuber = pubDepth.getNumSubscribers ();
		int leftImgSuber = pubLeft.getNumSubscribers ();
		int rightImgSuber = pubRight.getNumSubscribers ();

		//Obtain and publish disparity and depth image if both are subscribed
		if ( dispImgSuber >0 && depthImgSuber > 0 )
		{
			
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
			imagePublisher (depthMsg, depthMap);
			pubDepth.publish (depthMsg); 

		}

		//Obtain and publish disparity image if subscribed
		else if (dispImgSuber > 0)
		{
			
			status = taraxlDepth->getMap (left, right, grayDisp, true, depthMap, false);  //SDK function to obtain Disparity image

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

			status = taraxlDepth->getMap (left, right, grayDisp, false, depthMap, true);  //SDK function to obtain Depth image

			if (status != TARAXL_SUCCESS) 
			{ 

				cout << "Failed to get Depth image" << endl;
				exit (0);
			}

			//Publish depth map
			depthMap.convertTo (depthMap, CV_8UC1);
			imagePublisher (depthMsg, depthMap);
			pubDepth.publish (depthMsg);  
			
		}
		
		//Obtain and publish left/right image if subscribed
		if (leftImgSuber > 0 || rightImgSuber > 0)
		{

			status = taraxlDepth->getMap (left, right, grayDisp, false, depthMap, false);  //SDK function to obtain left and right image

			if (status != TARAXL_SUCCESS) 
			{
				cout << "Failed to get left and right images" << endl;
				exit (0);
			}

			//Publish left and right image
			imagePublisher (leftMsg,left);
			imagePublisher (rightMsg,right);
			pubLeft.publish (leftMsg);    
			pubRight.publish (rightMsg); 
			
		}

		ros::spinOnce ();
			
	}

}	


void taraxlros::imagePublisher(sensor_msgs::ImagePtr &imageMsg, Mat image)
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
	imageMsg->header.frame_id = "taraxl";
	imageMsg->is_bigendian = ! (* (char*)&num == 1);

	
}


void taraxlros::disparityPublisher(stereo_msgs::DisparityImagePtr &dispMsg, Mat dispImage)
{

	int num = 1; // for endianness detection

	//Convert to ROS message
	dispMsg->min_disparity = 0;
	dispMsg->max_disparity = maxDisp;
	dispMsg->header.stamp = ros::Time::now ();
    	dispMsg->header.frame_id = "taraxl";
	sensor_msgs::Image& dimage = dispMsg->image;
	dimage.width  = dispImage.size ().width ;
	dimage.height = dispImage.size ().height ;
	dimage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
	dimage.step = dimage.width * sizeof (float);
	dimage.data.resize (dimage.step * dimage.height);
	cv::Mat_<float> dmat (dimage.height, dimage.width, reinterpret_cast<float*> (&dimage.data[0]), dimage.step);
	dispImage.convertTo (dmat, dmat.type ());

	dispMsg->image.header = dispMsg->header;
	dispMsg->image.is_bigendian = ! (* (char*)&num == 1);

	dispMsg->valid_window.x_offset = 0;
	dispMsg->valid_window.y_offset = 0;
	dispMsg->valid_window.width    = 0;
	dispMsg->valid_window.height   = 0;
	dispMsg->T                     = 0.06;
	dispMsg->f                     = 7.0966755474656929e+002;
	dispMsg->delta_d               = 0;

	if (dispMsg == NULL) 
	{
		cout << "Failed to get disparity image "<< endl;
  		exit (0);
	}

}	




void taraxlros::dynamicReconfCallback (taraxl_ros_package::taraxlrosConfig &config, uint32_t level)
{
  
	enum SETTINGS {BRIGHTNESS, EXPOSURE, ACCURACY, AUTOEXPOSURE};

	switch (level)
	{
	
		case BRIGHTNESS:
			status = taraxlCam.setBrightness (config.brightness);
			break;

		case EXPOSURE:
			status = taraxlCam.setExposure (config.exposure);
			config.autoExposure = false;
			break;

		case ACCURACY:
			if (config.accuracy == 0) 
			{
				status = taraxlDepth->setAccuracy (TaraXLSDK::HIGH);
			}
			else if (config.accuracy == 1) 
			{	
				status = taraxlDepth->setAccuracy (TaraXLSDK::LOW);
			}
			maxDisp = (config.accuracy == 1) ? 64 : 128 ;
			break;

		case AUTOEXPOSURE:
			if (config.autoExposure)
				status = taraxlCam.enableAutoExposure ();
			break;
	}

}


int main (int argc, char **argv)
{
	
	ros::init (argc, argv, "taraxl_ros_package");
	 
	taraxlros roswrapper;
		
	roswrapper.rosPublish ();

	return 0;
	

}
