/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceController.h>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

//constructeur
HapticAvatar_ArticulatedDeviceController::HapticAvatar_ArticulatedDeviceController()
    : HapticAvatar_BaseDeviceController()
    , d_toolPosition(initData(&d_toolPosition, "toolPosition", "Output data position of the tool"))
    , m_forceFeedback(nullptr)
{
    this->f_listening.setValue(true);
}


} // namespace sofa::HapticAvatar
