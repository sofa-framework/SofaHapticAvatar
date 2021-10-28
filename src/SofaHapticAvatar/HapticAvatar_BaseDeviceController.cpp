/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <iomanip> 

namespace sofa::HapticAvatar
{

//constructeur
HapticAvatar_BaseDeviceController::HapticAvatar_BaseDeviceController()
    : d_portName(initData(&d_portName, std::string("//./COM3"), "portName", "Name of the port used by this device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "Data to store Information received by HW device"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the tool Coordinates"))
    , m_forceScale(initData(&m_forceScale, SReal(1.0), "forceScale", "jaws opening angle"))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "Position of the base of the device"))
    , d_dumpThreadInfo(initData(&d_dumpThreadInfo, false, "dumpThreadInfo", "Parameter to dump thread info"))
    , d_drawDeviceAxis(initData(&d_drawDeviceAxis, false, "drawDeviceAxis", "Parameter to draw dof axis"))
    , d_drawDebug(initData(&d_drawDebug, false, "drawDebugForce", "Parameter to draw debug information"))
    , d_drawLogOutputs(initData(&d_drawLogOutputs, false, "drawLogOutputs", "Parameter to draw output logs"))
    , l_portalMgr(initLink("portalManager", "link to portalManager"))    
    , m_simulationStarted(false)
    , m_terminate(true)
    , m_HA_driver(nullptr)
    , m_portalMgr(nullptr)
    , m_deviceReady(false)
    , m_portId(-1)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

}


HapticAvatar_BaseDeviceController::~HapticAvatar_BaseDeviceController()
{
    clearDevice();
    if (m_HA_driver)
    {
        delete m_HA_driver;
        m_HA_driver = nullptr;
    }
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_BaseDeviceController::init()
{
    msg_info() << "HapticAvatar_BaseDeviceController::init()";
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

    initImpl();
}


void HapticAvatar_BaseDeviceController::clearDevice()
{
    msg_info() << "HapticAvatar_BaseDeviceController::clearDevice()";
    if (m_terminate == false && m_deviceReady)
    {
        m_terminate = true;
        haptic_thread.join();
        copy_thread.join();
    }
}


void HapticAvatar_BaseDeviceController::bwdInit()
{   
    msg_info() << "HapticAvatar_BaseDeviceController::bwdInit()";
    if (!m_portalMgr)
        return;
    
    m_portId = m_portalMgr->getPortalId(d_portName.getValue());
    if (m_portId == -1)
    {
        msg_error("HapticAvatar_BaseDeviceController no portal id found");
        m_deviceReady = false;
        return;
    }
    msg_info() << "Portal Id found: " << m_portId;
        
    m_deviceReady = createHapticThreads();
}



void HapticAvatar_BaseDeviceController::updatePortalAnglesAndLength(sofa::type::fixed_array<float, 4> values)
{
    m_portalMgr->updatePostion(m_portId, values[Dof::YAW], values[Dof::PITCH]);
}



void HapticAvatar_BaseDeviceController::updatePosition()
{
    if (!m_HA_driver)
        return;

    // get info from simuData
    sofa::type::fixed_array<float, 4> dofV = m_simuData.anglesAndLength;

    // propagate info to portal
    updatePortalAnglesAndLength(dofV);

    // copy data for debug draw
    if (d_drawLogOutputs.getValue())
    {
        m_debugData = m_simuData;
    }
    
    // compute portal and tool rotation matrices
    const sofa::type::Mat4x4f& portalMtx = m_portalMgr->getPortalTransform(m_portId);
    sofa::type::Quatf rotRot = sofa::type::Quatf::fromEuler(0.0f, dofV[Dof::ROT], 0.0f);
    sofa::type::Mat4x4f T_insert = sofa::type::Mat4x4f::transformTranslation(Vec3f(0.0f, dofV[Dof::Z], 0.0f));
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


void HapticAvatar_BaseDeviceController::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    // draw internal specific info
    drawImpl(vparams);

    // If true draw debug information
    if (d_drawDebug.getValue())
    {
        drawDebug(vparams);
    }
}


void HapticAvatar_BaseDeviceController::drawDebug(const sofa::core::visual::VisualParams* vparams)
{
    if (d_drawLogOutputs.getValue())
    {
        int newLine = 12;
        int fontS = 12;
        const sofa::type::fixed_array<float, 4>& dofV = m_debugData.anglesAndLength;
        const sofa::type::fixed_array<float, 4>& motV = m_debugData.motorValues;
        sofa::type::RGBAColor color(0.0, 1.0, 0.0, 1.0);

        std::string title = "       Yaw   Pitch   Rot   Z";
        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, title.c_str());
        newLine += fontS * 2;

        std::stringstream ss;
        ss << std::fixed << std::setprecision(2) << "Value  "
            << dofV[Dof::YAW] << "  "
            << dofV[Dof::PITCH] << "  "
            << dofV[Dof::ROT] << "  "
            << dofV[Dof::Z];

        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
        newLine += fontS * 2;

        ss.str(std::string());
        ss << std::fixed << std::setprecision(2) << "Motor  "
            << motV[Dof::YAW] << "  "
            << motV[Dof::PITCH] << "  "
            << motV[Dof::ROT] << "  "
            << motV[Dof::Z];

        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
        newLine += fontS * 4;



        std::string title2 = "           XForce  YForce  Zforce  JTorq";
        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, title2.c_str());
        newLine += fontS * 2;

        const sofa::type::fixed_array<float, 3>& colF = m_debugData.collisionForces;

        ss.str(std::string());
        ss << std::fixed << std::setprecision(2) << "Collision  "
            << colF[0] << "    "
            << colF[1] << "    "
            << colF[2] << "    ";

        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
        newLine += fontS * 2;

        ss.str(std::string());
        ss << std::fixed << std::setprecision(2) << "Jaws opening  "
            << m_debugData.jawOpening << "    ";

        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
        newLine += fontS * 2;
    }
}


void HapticAvatar_BaseDeviceController::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        m_simulationStarted = true;
        updatePosition();
    }
}

} // namespace sofa::HapticAvatar