/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatarDeviceController.h>
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

HapticEmulatorTask::HapticEmulatorTask(HapticAvatarDeviceController* ptr, CpuTask::Status* pStatus)
    :CpuTask(pStatus)
    , m_driver(ptr)
{

}

HapticEmulatorTask::MemoryAlloc HapticEmulatorTask::run()
{
    std::cout << "haptic run task" << std::endl;

    return MemoryAlloc::Dynamic;
}


//constructeur
HapticAvatarDeviceController::HapticAvatarDeviceController()
    : d_positionBase(initData(&d_positionBase, Vec3d(0, 0, 0), "positionBase", "Position of the interface base in the scene world coordinates"))
    , d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the interface base in the scene world coordinates"))
    , d_orientationTool(initData(&d_orientationTool, Quat(0, 0, 0, 1), "orientationTool", "Orientation of the tool"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Phantom Coordinates"))
    , d_forceScale(initData(&d_forceScale, 1.0, "forceScale", "Default forceScale applied to the force feedback. "))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))
    , d_drawDevice(initData(&d_drawDevice, false, "drawDevice", "draw device"))
    , d_portName(initData(&d_portName, std::string("//./COM3"),"portName", "position of the base of the part of the device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "position of the base of the part of the device"))
    , portId(-1)
    , l_portalMgr(initLink("portalManager", "link to portalManager"))
    , m_HA_driver(nullptr)
    , m_portalMgr(nullptr)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);
}


HapticAvatarDeviceController::~HapticAvatarDeviceController()
{
    clearDevice();
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatarDeviceController::init()
{
    msg_info() << "HapticAvatarDeviceController::init()";
    m_HA_driver = new HapticAvatarDriver(d_portName.getValue());

    if (!m_HA_driver->IsConnected())
        return;
        
    // get identity
    std::string identity = m_HA_driver->getIdentity();
    d_hapticIdentity.setValue(identity);
    std::cout << "HapticAvatarDeviceController identity: " << identity << std::endl;

    // get access to portalMgr
    if (l_portalMgr.empty())
    {
        msg_error() << "Link to HapticAvatarPortalManager not set.";
        return;
    }

    m_portalMgr = l_portalMgr.get();
    if (m_portalMgr == nullptr)
    {
        msg_error() << "HapticAvatarPortalManager access failed.";
        return;
    }

    // reset all force
    m_HA_driver->writeData("15 \n");
    char incomingData[INCOMING_DATA_LEN];
    int res = m_HA_driver->getData(incomingData, false);
    std::cout << "reset: " << incomingData << std::endl;

    return;
}


void HapticAvatarDeviceController::clearDevice()
{
    msg_info() << "HapticAvatarDeviceController::clearDevice()";
}


void HapticAvatarDeviceController::bwdInit()
{   
    msg_info() << "HapticAvatarDeviceController::bwdInit()";
    if (!m_deviceReady)
        return;
    
    portId = m_portalMgr->getPortalId(d_portName.getValue());
    if (portId == -1)
    {
        msg_error("HapticAvatarDeviceController no portal id found");
        m_deviceReady = false;
        return;
    }

    msg_info() << "Portal Id found: " << portId;
}


void HapticAvatarDeviceController::reinit()
{
    msg_info() << "HapticAvatarDeviceController::reinit()";
}

void HapticAvatarDeviceController::updatePosition()
{
    if (!m_HA_driver)
        return;

   // m_HA_driver->getAnglesAndLength();
}

void HapticAvatarDeviceController::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!d_drawDevice.getValue())
        return;

    //vparams->drawTool()->saveLastState();
    //vparams->drawTool()->restoreLastState();
}


void HapticAvatarDeviceController::handleEvent(core::objectmodel::Event *event)
{
    //if(m_errorDevice != 0)
    //    return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
    //    if (m_hStateHandles.size() && m_hStateHandles[0] == HD_INVALID_HANDLE)
    //        return;

        updatePosition();
    }
}

int HapticAvatarDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatarDeviceController >()
;

} // namespace controller

} // namespace component

} // namespace sofa