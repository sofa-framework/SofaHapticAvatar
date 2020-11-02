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

namespace sofa::HapticAvatar
{

//constructeur
HapticAvatar_BaseDeviceController::HapticAvatar_BaseDeviceController()
    : d_portName(initData(&d_portName, std::string("//./COM3"), "portName", "Name of the port used by this device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "Data to store Information received by HW device"))
    , d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the tool Coordinates"))
    , m_forceScale(initData(&m_forceScale, SReal(1.0), "forceScale", "jaws opening angle"))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "Position of the base of the device"))
    , d_toolPosition(initData(&d_toolPosition, "toolPosition", "Output data position of the tool"))
    , d_dumpThreadInfo(initData(&d_dumpThreadInfo, false, "dumpThreadInfo", "Parameter to dump thread info"))
    , d_drawDeviceAxis(initData(&d_drawDeviceAxis, false, "drawDeviceAxis", "Parameter to draw dof axis"))
    , d_drawDebug(initData(&d_drawDebug, false, "drawDebugForce", "Parameter to draw debug information"))
    , d_drawLogOutputs(initData(&d_drawLogOutputs, false, "drawLogOutputs", "Parameter to draw output logs"))
    , l_portalMgr(initLink("portalManager", "link to portalManager"))
    , m_forceFeedback(nullptr)
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

    HapticAvatar_BaseDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition.resize(8);
    d_toolPosition.endEdit();    
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
    m_HA_driver = new HapticAvatar_Driver(d_portName.getValue());

    if (!m_HA_driver->IsConnected())
        return;
        
    // get identity
    std::string identity = m_HA_driver->getIdentity();
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

    return;
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

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_forceFeedback = context->get<LCPForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback != nullptr)
    {
        msg_info() << "ForceFeedback found";
    }
    
    m_terminate = false;
    m_deviceReady = createHapticThreads();
}



void HapticAvatar_BaseDeviceController::updatePortalAnglesAndLength(sofa::helper::fixed_array<float, 4> values)
{
    m_portalMgr->updatePostion(m_portId, values[Dof::YAW], values[Dof::PITCH]);
}



void HapticAvatar_BaseDeviceController::updatePosition()
{
    if (!m_HA_driver)
        return;

    // get info from simuData
    sofa::helper::fixed_array<float, 4> dofV = m_simuData.anglesAndLength;

    // propagate info to portal
    updatePortalAnglesAndLength(dofV);

    // copy data for debug draw
    if (d_drawLogOutputs.getValue())
    {
        m_debugData = m_simuData;
    }
    
    // compute portal and tool rotation matrices
    const sofa::defaulttype::Mat4x4f& portalMtx = m_portalMgr->getPortalTransform(m_portId);
    sofa::defaulttype::Quat rotRot = sofa::defaulttype::Quat::fromEuler(0.0f, dofV[Dof::ROT], 0.0f);
    sofa::defaulttype::Mat4x4f T_insert = sofa::defaulttype::Mat4x4f::transformTranslation(Vec3f(0.0f, dofV[Dof::Z], 0.0f));
    sofa::defaulttype::Mat4x4f R_rot = sofa::defaulttype::Mat4x4f::transformRotation(rotRot);
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

    // draw tool rigid dof arrows
    if (d_drawDeviceAxis.getValue())
    {
        const HapticAvatar_BaseDeviceController::VecCoord & toolPosition = d_toolPosition.getValue();
        float glRadius = float(d_scale.getValue());

        for (unsigned int i = 0; i < toolPosition.size(); ++i)
        {
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));
        }
    }
    
    if (d_drawDebug.getValue())
    {
        drawDebug(vparams);
    }
}


void HapticAvatar_BaseDeviceController::drawDebug(const sofa::core::visual::VisualParams* vparams)
{
    const HapticAvatar_BaseDeviceController::VecCoord & toolPosition = d_toolPosition.getValue();
    const HapticAvatar_BaseDeviceController::VecDeriv& force = m_debugData.hapticForces;

    Vec3 dirTotal, angTotal;
    for (int i = 0; i < force.size(); i++)
    {
        Vec3 dir = force[i].getLinear();
        Vec3 ang = force[i].getAngular();
        //vparams->drawTool()->drawLine(toolPosition[i].getCenter(), toolPosition[i].getCenter() + dir * 50, defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0));
        //vparams->drawTool()->drawLine(toolPosition[i].getCenter(), toolPosition[i].getCenter() + ang * 50, defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0));

        dirTotal += dir;
        angTotal += ang;
    }

    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + dirTotal * 50, defaulttype::Vec4f(1.0f, 1.0f, 1.0f, 1.0));
    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + angTotal * 50, defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0));
    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + m_toolRotInv * dirTotal * 50, defaulttype::Vec4f(0.0f, 0.0f, 1.0f, 1.0));
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