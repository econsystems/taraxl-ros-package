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
TaraXLPoseTracking.h : Pose tracking APIs for IMUs
**********************************************************************/
#ifndef TARAXL_POSE_TRACKING_H_
#define TARAXL_POSE_TRACKING_H_

#include "TaraXLEnums.h"

namespace TaraXLSDK
{
    class TaraXLPoseTrackingImpl;

    //Struct that contains the IMU data.
    struct TaraXLIMUData
    {
        Vector3 angularVelocity;
        Vector3 linearAcceleration;
	Vector3 inclination;
        Vector3 getInclination();
        TaraXLIMUData();
    };


    class TaraXLPoseTracking
    {
        public:
            TaraXLPoseTracking(TaraXLCam &camera);
            ~TaraXLPoseTracking();

            //Sets the output frequency
            TARAXL_STATUS_CODE setIMUOutputFrequency(TARAXL_IMU_OUTPUT_FREQUENCY frequency);

            //Gets the output frequency
            TARAXL_STATUS_CODE getIMUOutputFrequency(TARAXL_IMU_OUTPUT_FREQUENCY &frequency);

            //Gets the IMU data.
            TARAXL_STATUS_CODE getIMUData(TaraXLIMUData &data);
        private:
            std::shared_ptr<TaraXLPoseTrackingImpl> taraXLPoseTrackingImpl;
    };
}
#endif  /* TARAXL_POSE_TRACKING_H_ */
