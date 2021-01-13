/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

#include <sofa/core/ObjectFactory.h>


namespace sofa::HapticAvatar
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
    m_HA_driver = new HapticAvatar_IboxDriver(d_portName.getValue());

    if (!m_HA_driver->IsConnected()) {
        msg_error() << "HapticAvatar_IBoxController driver creation failed";
        return;
    }

    // get identity
    std::string identity = m_HA_driver->getIdentity();
    d_hapticIdentity.setValue(identity);
    std::cout << "HapticAvatarDeviceController identity: '" << identity << "'" << std::endl;

	setLoopGain(120000);

    return;
}

float HapticAvatar_IBoxController::getJawOpeningAngle()
{
    sofa::helper::fixed_array<float, 4> values = m_HA_driver->getOpeningValues();
    return values[0];
}


void HapticAvatar_IBoxController::setHandleForces(float upperJawForce, float lowerJawForce)
{
    return m_HA_driver->setHandleForces(upperJawForce, lowerJawForce);
}

void HapticAvatar_IBoxController::setLoopGain(int loopGain)
{
	return m_HA_driver->setLoopGain(loopGain);
}


void HapticAvatar_IBoxController::clearDevice()
{
    msg_info() << "HapticAvatar_IBoxController::clearDevice()";
   
}


} // namespace sofa::HapticAvatar
