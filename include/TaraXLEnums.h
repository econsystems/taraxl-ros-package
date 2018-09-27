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
TaraXLEnums.h :  TaraXLEnums contains descriptions about all
                 enums used in TaraXLSDK.
**********************************************************************/
#ifndef TARAXL_ENUMS_H_
#define TARAXL_ENUMS_H_

namespace TaraXLSDK
{
  /*
  *	TaraXL supports two accuracy modes
  *	LOW accuracy  : Gives high performance
  *	HIGH accuracy : Gives low performance
  */
  enum ACCURACY_MODE
  {
    HIGH = 0,
    LOW = 1
  };

  /*
  *	Various status codes that
  *	are returned in TaraXL APIs
  */
  enum TARAXL_STATUS_CODE
  {
    TARAXL_FAILURE = 0,
    TARAXL_SUCCESS = 1,
    NO_DEVICES_CONNECTED = 100,
    INVALID_DEVICE_ID = 101,
    FIRMWARE_NOT_COMPATIBLE = 102,
    NODE_FAILURE = 103,
    INTERNAL_PIPELINE_ERROR = 104,
    CAMERA_OPEN_FAILED = 201,
    CAMERA_NOT_AVAILABLE = 202,
    INVALID_OPTION = 203,
    NOT_INITIALISED = 204,
    UNSUPPORTED_RESOLUTION = 205,
    EXTENSION_UNIT_FAILED = 206,
    RECTIFICATION_FAILED = 207,
    EXPOSURE_SETTING_FAILED = 301,
    FAILED_TO_LOAD_INTRINSIC_AND_EXTRINSIC_FILES =302,
    INVALID_INTRINSIC_AND_EXTRINSIC_FILE_LENGTH =303,
    INVALID_INTRINSIC_AND_EXTRINSIC_FILE_DATA=304,
    INPUT_MATRIX_EMPTY =305,
    EXPOSURE_OUT_OF_BOUNDS = 306,
    EXPOSURE_GETTING_FAILED =307,
    AUTO_EXPOSURE_SETTING_FAILED = 308,
    BRIGHTNESS_OUT_OF_BOUNDS = 309
  };
}

#endif
