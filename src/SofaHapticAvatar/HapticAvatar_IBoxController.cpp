/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>
#include <SofaHapticAvatar/HapticAvatar_Defines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>

namespace sofa::component::controller
{

using namespace HapticAvatar;

int HapticAvatar_IBoxControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_IBoxController >()
    ;


//constructeur
HapticAvatar_IBoxController::HapticAvatar_IBoxController()
    : d_portName(initData(&d_portName, std::string("//./COM5"), "portName", "position of the base of the part of the device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "position of the base of the part of the device"))
    , m_HA_driver(nullptr)
    , m_deviceReady(false)    
{
    this->f_listening.setValue(true);
    
    
}


HapticAvatar_IBoxController::~HapticAvatar_IBoxController()
{
    clearDevice();
    if (m_HA_driver)
    {
        delete m_HA_driver;
        m_HA_driver = nullptr;
    }
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_IBoxController::init()
{
    msg_info() << "HapticAvatar_IBoxController::init()";
    m_HA_driver = new HapticAvatar_Driver(d_portName.getValue());

    if (!m_HA_driver->IsConnected()) {
        msg_error() << "HapticAvatar_IBoxController driver creation failed";
        return;
    }

    // get identity
    std::string identity = m_HA_driver->getIdentity();
    d_hapticIdentity.setValue(identity);
    std::cout << "HapticAvatarDeviceController identity: '" << identity << "'" << std::endl;

    return;
}

float HapticAvatar_IBoxController::getJawOpeningAngle()
{
    sofa::helper::fixed_array<float, 4> values = m_HA_driver->getAngles_AndLength();
    return values[0];
}

void HapticAvatar_IBoxController::clearDevice()
{
    msg_info() << "HapticAvatar_IBoxController::clearDevice()";
   
}


void HapticAvatar_IBoxController::bwdInit()
{   
    msg_info() << "HapticAvatar_IBoxController::bwdInit()";
    
}


void HapticAvatar_IBoxController::reinit()
{
    msg_info() << "HapticAvatar_IBoxController::reinit()";
}



} // namespace sofa::component::controller
