/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatarDriver.h>
#include <SofaHapticAvatar/HapticAvatarDefines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa
{

namespace component
{

namespace controller
{

//constructeur
HapticAvatarDriver::HapticAvatarDriver()
    : d_positionBase(initData(&d_positionBase, Vec3d(0, 0, 0), "positionBase", "Position of the interface base in the scene world coordinates"))
    , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the interface base in the scene world coordinates"))
    , d_orientationTool(initData(&d_orientationTool, Quat(0, 0, 0, 1), "orientationTool", "Orientation of the tool"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Phantom Coordinates"))
    , d_forceScale(initData(&d_forceScale, 1.0, "forceScale", "Default forceScale applied to the force feedback. "))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))
    , d_drawDevice(initData(&d_drawDevice, false, "drawDevice", "draw device"))
{
    this->f_listening.setValue(true);

}


HapticAvatarDriver::~HapticAvatarDriver()
{
    clearDevice();
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatarDriver::init()
{
    msg_info() << "HapticAvatarDriver::init()";
    m_HA_API = new HapticAvatarAPI("//./COM3");

}


void HapticAvatarDriver::clearDevice()
{
    msg_info() << "HapticAvatarDriver::clearDevice()";
}


void HapticAvatarDriver::bwdInit()
{
    msg_info() << "HapticAvatarDriver::bwdInit()";
}


void HapticAvatarDriver::reinit()
{
    msg_info() << "HapticAvatarDriver::reinit()";
}

void HapticAvatarDriver::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!d_drawDevice.getValue())
        return;

    //vparams->drawTool()->saveLastState();
    //vparams->drawTool()->restoreLastState();
}


void HapticAvatarDriver::handleEvent(core::objectmodel::Event *event)
{
    //if(m_errorDevice != 0)
    //    return;

    //if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    //{
    //    if (m_hStateHandles.size() && m_hStateHandles[0] == HD_INVALID_HANDLE)
    //        return;

    //    updatePosition();
    //}
}

int HapticAvatarDriverClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatarDriver >()
;

} // namespace controller

} // namespace component

} // namespace sofa
