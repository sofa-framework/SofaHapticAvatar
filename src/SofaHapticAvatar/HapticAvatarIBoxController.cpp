/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatarIBoxController.h>
#include <SofaHapticAvatar/HapticAvatarDefines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>

namespace sofa
{

namespace component
{

namespace controller
{

using namespace HapticAvatar;

int HapticAvatarIBoxControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatarIBoxController >()
    ;


//constructeur
HapticAvatarIBoxController::HapticAvatarIBoxController()
    : d_portName(initData(&d_portName, std::string("//./COM5"), "portName", "position of the base of the part of the device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "position of the base of the part of the device"))
    , m_HA_driver(nullptr)
    , m_deviceReady(false)    
{
    this->f_listening.setValue(true);
    
    
}


HapticAvatarIBoxController::~HapticAvatarIBoxController()
{
    clearDevice();
    if (m_HA_driver)
    {
        delete m_HA_driver;
        m_HA_driver = nullptr;
    }
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatarIBoxController::init()
{
    msg_info() << "HapticAvatarIBoxController::init()";
    m_HA_driver = new HapticAvatarDriver(d_portName.getValue());

    if (!m_HA_driver->IsConnected()) {
        msg_error() << "HapticAvatarIBoxController driver creation failed";
        return;
    }

    // get identity
    std::string identity = m_HA_driver->getIdentity();
    d_hapticIdentity.setValue(identity);
    std::cout << "HapticAvatarDeviceController identity: '" << identity << "'" << std::endl;

    return;
}

float HapticAvatarIBoxController::getJawOpeningAngle()
{
    sofa::helper::fixed_array<float, 4> values = m_HA_driver->getAngles_AndLength();
    return values[0];
}

void HapticAvatarIBoxController::clearDevice()
{
    msg_info() << "HapticAvatarIBoxController::clearDevice()";
   
}


void HapticAvatarIBoxController::bwdInit()
{   
    msg_info() << "HapticAvatarIBoxController::bwdInit()";
    
}


void HapticAvatarIBoxController::reinit()
{
    msg_info() << "HapticAvatarIBoxController::reinit()";
}



} // namespace controller

} // namespace component

} // namespace sofa
