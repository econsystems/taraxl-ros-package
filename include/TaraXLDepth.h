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
TaraXLDepth.h :  Contains the depth and disparity related APIs
**********************************************************************/
#ifndef TARAXL_DEPTH_H_
#define TARAXL_DEPTH_H_

#include "TaraXLCam.h"

namespace TaraXLSDK
{
  class TaraXLDepth
  {
  public:
    TaraXLDepth(TaraXLCam &camera);
    ~TaraXLDepth();

    //Sets the accuracy for the disparity images.
    TARAXL_STATUS_CODE setAccuracy(ACCURACY_MODE mode);

    //Gets the disparity and depth maps.
    TARAXL_STATUS_CODE getMap(cv::Mat &leftImage, cv::Mat &rightImage, cv::Mat &disparityMap, bool fillDisparityMap, cv::Mat &depthMap, bool fillDepthMap);
  };
}
#endif  /* TARAXL_DEPTH_H_ */
