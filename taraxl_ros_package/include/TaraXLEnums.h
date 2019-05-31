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

#include <opencv2/highgui/highgui.hpp>
#include <iostream>

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
    LOW = 1,
    ULTRA =2
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
    UNSUPPORTED_FREQUENCY_FOR_THIS_REVISION = 208,
    IMU_EXTENSION_UNIT_FAILED = 209,
    EXPOSURE_SETTING_FAILED = 301,
    FAILED_TO_LOAD_INTRINSIC_AND_EXTRINSIC_FILES =302,
    INVALID_INTRINSIC_AND_EXTRINSIC_FILE_LENGTH =303,
    INVALID_INTRINSIC_AND_EXTRINSIC_FILE_DATA=304,
    INPUT_MATRIX_EMPTY =305,
    EXPOSURE_OUT_OF_BOUNDS = 306,
    EXPOSURE_GETTING_FAILED =307,
    AUTO_EXPOSURE_SETTING_FAILED = 308,
    BRIGHTNESS_OUT_OF_BOUNDS = 309,
    GAIN_OUT_OF_BOUNDS = 310,
    GET_UNIQUE_ID_FAILURE = 311,
    SET_IMU_FREQUENCY_FAILURE = 312,
    BRIGHTNESS_SETTING_FAILED = 313,
    GAIN_SETTING_FAILED = 314,
    BRIGHTNESS_GETTING_FAILED = 315,
    CALIBRATION_DATA_NOT_FOUND = 316,
    CALIBRATION_DATA_CORRUPTED = 317
  };

  /*
  * Quality with which
  * pointcloud is calculated.
  */
  enum TARAXL_POINTCLOUD_QUALITY
  {
    STANDARD = 0,
    MEDIUM = 1,
    HIGHEST = 2
  };

  /*
  * Format in which PointCloud
  * is to be saved.
  */
  enum TARAXL_POINTCLOUD_FORMAT
  {
    TARAXL_PLY_CLOUD = 0,
    TARAXL_PCD_CLOUD = 1,
    TARAXL_VTK_CLOUD = 2
  };

  /*
  * Output frequency of
  * IMU sensor.
  */
  enum TARAXL_IMU_OUTPUT_FREQUENCY
  {
   IMU_119_HZ = (0x03),
   IMU_238_HZ = (0x04),
   IMU_476_HZ = (0x05),
   IMU_952_HZ = (0x06),
   IMU_12_5_HZ = (0x01),
   IMU_26_HZ = (0x02),
   IMU_52_HZ = (0x03),
   IMU_104_HZ = (0x04),
   IMU_208_HZ = (0x05),
   IMU_416_HZ = (0x06),
   IMU_833_HZ = (0x07),
   IMU_1666_HZ = (0x08)
  };

  enum TARAXL_FILTER_TYPE
  {
    TARAXL_DEFAULT_FILTER = 0,
    TARAXL_MEDIAN_FILTER = 1
  };


 typedef cv::Vec3f Vector3;
}

#endif
