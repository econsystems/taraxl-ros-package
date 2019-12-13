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
TaraXLPointcloud.h : implementations of pointcloud and other 3D options.
**********************************************************************/
#ifndef TARAXL_POINTCLOUD_H_
#define TARAXL_POINTCLOUD_H_

#include "TaraXLCam.h"
#include "pcl/common/common_headers.h"

namespace TaraXLSDK
{

    typedef pcl::PointCloud<pcl::PointXYZRGB> Points;

    class TaraXLPointcloudImpl;
    class TaraXLPointcloud
    {
    public:
        TaraXLPointcloud(TaraXLCam &camera);
	~TaraXLPointcloud();

        //Sets the quality of the pointcloud that is to be rendered.
        TARAXL_STATUS_CODE setPointcloudQuality(TARAXL_POINTCLOUD_QUALITY pointcloudQuality);

        //Saves the pointcloud at that particular frame using the given file name
        TARAXL_STATUS_CODE savePoints(TARAXL_POINTCLOUD_FORMAT pointcloudFormat, std::string filename);

        //Gets the pointcloud in the specified format.
        TARAXL_STATUS_CODE getPoints(Points::Ptr currentCloud);
    
    private:

    	std::shared_ptr<TaraXLPointcloudImpl> taraXLPointcloudImpl;
    };
}
#endif  /* TARAXL_POINTCLOUD_H_ */
