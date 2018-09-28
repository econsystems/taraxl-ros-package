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
TaraXL.h : All the implementations of camera enumeration
**********************************************************************/

#ifndef _TARAXL_H
#define _TARAXL_H


#include "TaraXLCam.h"

namespace TaraXLSDK
{
  class TaraXL
  {
  public:
    TaraXL();
    ~TaraXL();

    //Gets the information of the devices enumerated
    TARAXL_STATUS_CODE enumerateDevices(TaraXLCamList &taraCamList);
  };
}
#endif  /* TARAXL_H_ */
