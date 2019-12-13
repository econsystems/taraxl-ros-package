///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018, e-con Systems.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS.
// IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT/INDIRECT DAMAGES HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
/**********************************************************************
TaraXLCam.h :  TaraXLCam.h contains APIs about
             one tara cam instance. They can be
             used to modify the camera settings
             like brightness, exposure, resolution, etc.
**********************************************************************/
#ifndef TARAXL_CAM_H_
#define TARAXL_CAM_H_

#include "TaraXLEnums.h"
#include <vector>
#include "opencv2/core/core.hpp"
#include <memory>
#define TARAXL_SDK_VERSION "3.2.2"

namespace TaraXLSDK
{
  struct Resolution
  {
      int width;
      int height;
  };
  struct CalibrationParams
  {
      cv::Mat cameraMatrix;
      double apertureWidth;
      double apertureHeight;
      double fovX;
      double fovY;
      double focalLength;	
      cv::Mat rectifiedCameraMatrix;
      cv::Mat distortionMatrix;
  };
  typedef std::vector<Resolution> ResolutionList;
  class TaraXLCamImpl;
  class TaraXLCam
  {
  public:
        TaraXLCam();
        ~TaraXLCam();

        //Initialises the selected device with the selected resolution
        TARAXL_STATUS_CODE connect();

        //Free the device connected
        TARAXL_STATUS_CODE disconnect();

        //Grabs the frames
        TARAXL_STATUS_CODE grabFrame(cv::Mat &leftFrame, cv::Mat &rightFrame);

        //Gets the list of resolutions supported by the camera connected
        TARAXL_STATUS_CODE getResolutionList(ResolutionList &resolutionList);

        //Sets the resolution to the connected camera
        TARAXL_STATUS_CODE setResolution(Resolution &resolution);

        //Gets the current resolution to the connected camera
        TARAXL_STATUS_CODE getResolution(Resolution &resolution);

        //Sets the exposure of the connected camera
        TARAXL_STATUS_CODE setExposure(int exposureVal);

        //Gets the exposure of the connected camera
        TARAXL_STATUS_CODE getExposure(int &exposureVal);

        //Sets auto exposure to the connected camera
        TARAXL_STATUS_CODE enableAutoExposure();

        //Sets the Brightness Val of the connected camera
        TARAXL_STATUS_CODE setBrightness(int brightnessVal);

	//Sets the Gain Val of the connected camera
        TARAXL_STATUS_CODE setGain(int gainVal);

        //Gets the brightness value of the connected camera
        TARAXL_STATUS_CODE getBrightness(int &brightnessVal);

	//Gets the gain value of the connected camera
        TARAXL_STATUS_CODE getGain(int &gainVal);

        //Gets the Q matrix of the connected camera
        TARAXL_STATUS_CODE getQMatrix(cv::Mat &Q);

        //Gets the friendly Name of the camera.
        TARAXL_STATUS_CODE getFriendlyName(std::string &name);

	//Gets the unique serial number of the connected camera
        TARAXL_STATUS_CODE getCameraUniqueId(std::string &uniqueId);

        //Grabs the unrectified raw frames.
        TARAXL_STATUS_CODE getUnrectifiedFrame(cv::Mat &leftUnrectified, cv::Mat &rightUnrectified);

        //Gets the calibration parameters fo the camera.
        TARAXL_STATUS_CODE getCalibrationParameters(cv::Mat &R, cv::Mat &T, CalibrationParams &left, CalibrationParams &right,CalibrationParams &leftRectified, CalibrationParams &rightRectified);

	//Gets the version of the camera
        TARAXL_STATUS_CODE getSDKVersion(std::string &version);

    private:

        friend class TaraXL;
	friend class TaraXLPointcloud;
      	friend class TaraXLPoseTracking;
      	friend class TaraXLDepth;

        std::shared_ptr<TaraXLCamImpl> taraXLCamImpl;

  };
  typedef std::vector<TaraXLCam> TaraXLCamList;
}
#endif  /* TARAXL_CAM_H_ */
