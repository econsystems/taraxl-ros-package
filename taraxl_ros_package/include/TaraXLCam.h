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

namespace TaraXLSDK
{
  struct Resolution
  {
      int width;
      int height;
  };
  typedef std::vector<Resolution> ResolutionList;

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

        //Sets the exposure of the connected camera
        TARAXL_STATUS_CODE setExposure(int exposureVal);

        //Gets the exposure of the connected camera
        TARAXL_STATUS_CODE getExposure(int &exposureVal);

        //Sets auto exposure to the connected camera
        TARAXL_STATUS_CODE enableAutoExposure();

        //Sets the Brightness Val of the connected camera
        TARAXL_STATUS_CODE setBrightness(int brightnessVal);

        //Gets the brightness value of the connected camera
        TARAXL_STATUS_CODE getBrightness(int &brightnessVal);

        //Gets the Q matrix of the connected camera
        TARAXL_STATUS_CODE getQMatrix(cv::Mat &Q);

        //Gets the friendly Name of the camera.
        TARAXL_STATUS_CODE getFriendlyName(std::string &name);

    private:

        friend class TaraXL;
        int exposure;
        double brightness;
        cv::Mat Q;
        ResolutionList supportedResolutions;
        std::string friendlyName;
        std::string busInfo;
        Resolution selectedResolution;
        int deviceId;
  };
  typedef std::vector<TaraXLCam> TaraXLCamList;
}
#endif  /* TARAXL_CAM_H_ */
