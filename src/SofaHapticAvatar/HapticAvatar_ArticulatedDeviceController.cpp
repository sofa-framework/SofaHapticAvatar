/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_HapticThreadManager.h>
#include <sofa/simulation/Node.h>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

//constructeur
HapticAvatar_ArticulatedDeviceController::HapticAvatar_ArticulatedDeviceController()
    : HapticAvatar_BaseDeviceController()
    , d_toolPosition(initData(&d_toolPosition, "toolPosition", "Output data position of the tool"))    
    , l_forceFeedback(initLink("forceFeedBack", "link to the forceFeedBack component, if not set will search through graph and take first one encountered."))
    , l_portalMgr(initLink("portalManager", "link to portalManager"))
    , m_forceFeedback(nullptr)
    , m_portalMgr(nullptr)
{
    m_toolRot.identity();
}

HapticAvatar_ArticulatedDeviceController::~HapticAvatar_ArticulatedDeviceController()
{
    HapticAvatar_HapticThreadManager::kill();
    clearDevice();
}

void HapticAvatar_ArticulatedDeviceController::initDevice()
{
    msg_info() << "HapticAvatar_ArticulatedDeviceController::initDevice()";
    m_HA_driver = new HapticAvatar_DriverPort(d_portName.getValue());

    if (!m_HA_driver->IsConnected())
        return;

    // get identity
    std::string identity = m_HA_driver->getDeviceType();
    d_hapticIdentity.setValue(identity);
    std::cout << "HapticAvatar_BaseDeviceController identity: '" << identity << "'" << std::endl;

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

    // release force
    m_HA_driver->releaseForce();


    // Retrieve ForceFeedback component pointer
    if (l_forceFeedback.empty())
    {
        simulation::Node* context = dynamic_cast<simulation::Node*>(this->getContext()); // access to current node
        m_forceFeedback = context->get<LCPForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    }
    else
    {
        m_forceFeedback = l_forceFeedback.get();
    }

    m_HA_driver->setDeadBandPWMWidth(100, 0, 0, 0);
    if (m_forceFeedback == nullptr)
    {
        msg_warning() << "ForceFeedback not found";
    }
}


void HapticAvatar_ArticulatedDeviceController::bwdInit()
{
    msg_info() << "HapticAvatar_ArticulatedDeviceController::bwdInit()";
    if (!m_portalMgr)
        return;

    m_portId = m_portalMgr->getPortalId(d_portName.getValue());
    if (m_portId == -1)
    {
        msg_error("HapticAvatar_ArticulatedDeviceController no portal id found");
        m_deviceReady = false;
        return;
    }
    msg_info() << "Portal Id found: " << m_portId;

    m_deviceReady = createHapticThreads();
}


void HapticAvatar_ArticulatedDeviceController::clearDevice()
{
    msg_info() << "HapticAvatar_ArticulatedDeviceController::clearDevice()";
    if (m_terminate == false && m_deviceReady)
    {
        m_terminate = true;
        copy_thread.join();
    }

    if (m_HA_driver)
    {
        delete m_HA_driver;
        m_HA_driver = nullptr;
    }
}


void HapticAvatar_ArticulatedDeviceController::simulation_updateData()
{
    // update virtual device position from haptic information
    //updatePosition();
    // For the moment squeeze the portal position. 
    updatePositionImpl();
}


void HapticAvatar_ArticulatedDeviceController::updatePosition()
{
    if (!m_deviceReady)
        return;

    // get info from simuData
    sofa::type::fixed_array<float, 4> dofV = m_simuData.anglesAndLength;

    // propagate info to portal: updatePortalAnglesAndLength
    m_portalMgr->updatePostion(m_portId, dofV[Dof::YAW], dofV[Dof::PITCH]);

    // copy data for debug draw
    if (d_drawDebug.getValue())
    {
        m_debugData = m_simuData;
    }

    // compute portal and tool rotation matrices
    const sofa::type::Mat4x4f& portalMtx = m_portalMgr->getPortalTransform(m_portId);
    sofa::type::Quatf rotRot = sofa::type::Quatf::fromEuler(0.0f, dofV[Dof::ROT], 0.0f);
    sofa::type::Mat4x4f T_insert = sofa::type::Mat4x4f::transformTranslation(type::Vec3f(0.0f, dofV[Dof::Z], 0.0f));
    sofa::type::Mat4x4f R_rot = sofa::type::Mat4x4f::transformRotation(rotRot);
    m_instrumentMtx = portalMtx * R_rot * T_insert;

    for (unsigned int i = 0; i < 3; i++) {
        for (unsigned int j = 0; j < 3; j++) {
            m_toolRot[i][j] = m_instrumentMtx[i][j];
            m_PortalRot[i][j] = portalMtx[i][j];
        }
    }
    m_toolRotInv = m_toolRot.inverted();

    // update tool positions depending on the specialisation type
    return updatePositionImpl();
}



} // namespace sofa::HapticAvatar
