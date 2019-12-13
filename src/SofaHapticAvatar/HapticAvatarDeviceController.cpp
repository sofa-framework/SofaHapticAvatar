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
#include <chrono>
#include <iomanip>

namespace sofa
{

namespace component
{

namespace controller
{

using namespace HapticAvatar;

int HapticAvatarDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatarDeviceController >()
    ;


HapticEmulatorTask::HapticEmulatorTask(HapticAvatarDeviceController* ptr, CpuTask::Status* pStatus)
    : CpuTask(pStatus)
    , m_controller(ptr)
{

}

HapticEmulatorTask::MemoryAlloc HapticEmulatorTask::run()
{
    std::cout << "haptic run task" << std::endl;

   /* if (m_driver->m_terminate == false)
    {
        TaskScheduler::getInstance()->addTask(new HapticEmulatorTask(m_driver, &m_driver->_simStepStatus));
        Sleep(100);
    }*/
    return MemoryAlloc::Dynamic;
}


//constructeur
HapticAvatarDeviceController::HapticAvatarDeviceController()
    : d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Phantom Coordinates"))
    //, d_positionBase(initData(&d_positionBase, Vec3d(0, 0, 0), "positionBase", "Position of the interface base in the scene world coordinates"))
    //, d_orientationBase(initData(&d_orientationBase, Quat(0, 0, 0, 1), "orientationBase", "Orientation of the interface base in the scene world coordinates"))
    //, d_orientationTool(initData(&d_orientationTool, Quat(0, 0, 0, 1), "orientationTool", "Orientation of the tool"))
    , d_posDevice(initData(&d_posDevice, "positionDevice", "position of the base of the part of the device"))        
    , d_toolValues(initData(&d_toolValues, "toolValues (Rot angle, Pitch angle, z Length, Yaw Angle)", "Device values: Rot angle, Pitch angle, z Length, Yaw Angle"))
    , d_motorOutput(initData(&d_motorOutput, "motorOutput (Rot, Pitch Z, Yaw)", "Motor values: Rot angle, Pitch angle, z Length, Yaw Angle"))
    

    , d_portName(initData(&d_portName, std::string("//./COM3"),"portName", "position of the base of the part of the device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "position of the base of the part of the device"))
    , m_portId(-1)
    , l_portalMgr(initLink("portalManager", "link to portalManager"))

    , d_drawDevice(initData(&d_drawDevice, false, "drawDevice", "draw device"))
    , d_fontSize(initData(&d_fontSize, 12, "fontSize", "font size of statistics to display"))
    , m_deviceReady(false)
    , m_terminate(true)

    , m_HA_driver(nullptr)
    , m_portalMgr(nullptr)
    , m_forceFeedback(nullptr)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_debugToolPosition = Vector3(0.0, 0.0, 0.0);
    m_debugForceVector = Vector3(0.0, 0.0, 0.0);
    m_debugRootPosition = Vector3(0.0, 0.0, 0.0);
    m_toolRot.identity();
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
    std::cout << "HapticAvatarDeviceController identity: '" << identity << "'" << std::endl;

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
    int res = m_HA_driver->resetDevice(15);
    if (res == -1)
        std::cerr << "## Error, Reset failed!" << std::endl;
    else
        std::cout << "Reset succeed return value: '" << res << "'" << std::endl;

    sofa::defaulttype::Vector3 force;
    
    // create task scheduler
    //unsigned int mNbThread = 2;
    //m_taskScheduler = sofa::simulation::TaskScheduler::getInstance();
    //m_taskScheduler->init(mNbThread);
    //m_taskScheduler->addTask(new HapticEmulatorTask(this, &m_simStepStatus));
   

    return;
}


void HapticAvatarDeviceController::clearDevice()
{
    msg_info() << "HapticAvatarDeviceController::clearDevice()";
    if (m_terminate == false)
    {
        m_terminate = true;
        haptic_thread.join();
    }
}


void HapticAvatarDeviceController::bwdInit()
{   
    msg_info() << "HapticAvatarDeviceController::bwdInit()";
    if (!m_portalMgr)
        return;
    
    m_portId = m_portalMgr->getPortalId(d_portName.getValue());
    if (m_portId == -1)
    {
        msg_error("HapticAvatarDeviceController no portal id found");
        m_deviceReady = false;
        return;
    }

    msg_info() << "Portal Id found: " << m_portId;

    m_terminate = false;
    m_deviceReady = true;
    haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_forceFeedback = context->get<ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback != nullptr)
    {
        msg_info() << "ForceFeedback found";
    }
}


void HapticAvatarDeviceController::reinit()
{
    msg_info() << "HapticAvatarDeviceController::reinit()";
}


void HapticAvatarDeviceController::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics thread" << std::endl;

    HapticAvatarDeviceController* _deviceCtrl = static_cast<HapticAvatarDeviceController*>(p_this);
    HapticAvatarDriver* _driver = static_cast<HapticAvatarDriver*>(p_driver);

    if (_deviceCtrl == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatarDeviceController cast failed");
        return;
    }

    if (_driver == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatarDriver cast failed");
        return;
    }

    // Loop Timer
    HANDLE h_timer;
    long t_ms = 1; // 1ms loop speed

    // Haptics Loop
    const auto wait_duration = std::chrono::milliseconds(t_ms);
    while (!terminate)
    {
        auto t1 = std::chrono::high_resolution_clock::now();

        sofa::helper::fixed_array<float, 4> toolValues = _driver->getAngles_AndLength();
        sofa::helper::fixed_array<float, 4> motorValues = _driver->getLastPWM();
        sofa::helper::fixed_array<float, 3> collForces = _driver->getLastCollisionForce();
        float jtorq = _driver->getJawTorque();
        //float angle = _driver->getJawOpeningAngle();

        //std::cout << "results: " << results << std::endl;
        _deviceCtrl->updateAnglesAndLength(toolValues);
        _deviceCtrl->d_motorOutput.setValue(motorValues);
        _deviceCtrl->d_collisionForce.setValue(collForces);
        _deviceCtrl->d_jawTorq.setValue(jtorq);
        //_deviceCtrl->d_jawOpening.setValue(angle);

        Vector3 currentForce;
        double maxInputForceFeedback = 0.001;//driver->d_maxInputForceFeedback.getValue();
        if (_deviceCtrl->m_forceFeedback)
        {
            Vector3 pos_in_world = _deviceCtrl->d_posDevice.getValue().getCenter();
            _deviceCtrl->m_forceFeedback->computeForce(pos_in_world[0], pos_in_world[1], pos_in_world[2], 0, 0, 0, 0, currentForce[0], currentForce[1], currentForce[2]);
            
            _deviceCtrl->m_debugToolPosition = pos_in_world;
            _deviceCtrl->m_debugForceVector = currentForce;

            bool contact = false;
            for (int i = 0; i < 3; i++)
            {
                if (currentForce[i] != 0.0)
                {
                    //driver->m_isInContact = true;
                    contact = true;
                    break;
                }
            }
            
            if (contact)
            {
                std::cout << "_deviceCtrl->m_toolRot: " << _deviceCtrl->m_toolRot << std::endl;
                std::cout << "haptic force: " << currentForce << std::endl;

                _driver->setForceVector(_deviceCtrl->m_toolRot * currentForce);
            }
            else
                _driver->releaseForce();
        }

        std::this_thread::sleep_for(wait_duration);
        auto t2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
        //std::cout << "Haptics loop: " << duration << std::endl;
    }

    // ensure no force
    _driver->releaseForce();
    std::cout << "Haptics thread END!!" << std::endl;
}


void HapticAvatarDeviceController::updateAnglesAndLength(sofa::helper::fixed_array<float, 4> values)
{
    m_portalMgr->updatePostion(m_portId, values[Dof::YAW], values[Dof::PITCH]);
    d_toolValues.setValue(values);
}

void HapticAvatarDeviceController::updatePosition()
{
    if (!m_HA_driver)
        return;

    const sofa::defaulttype::Mat4x4f& portalMtx = m_portalMgr->getPortalTransform(m_portId);
    m_debugRootPosition = m_portalMgr->getPortalPosition(m_portId);
    //std::cout << "portalMtx: " << portalMtx << std::endl;    

    const sofa::helper::fixed_array<float, 4>& dofV = d_toolValues.getValue();

    sofa::defaulttype::Quat rotRot = sofa::defaulttype::Quat::fromEuler(0.0f, dofV[Dof::ROT], 0.0f);
    sofa::defaulttype::Mat4x4f T_insert = sofa::defaulttype::Mat4x4f::transformTranslation(Vec3f(0.0f, dofV[Dof::Z], 0.0f));
    sofa::defaulttype::Mat4x4f R_rot = sofa::defaulttype::Mat4x4f::transformRotation(rotRot);
    
    sofa::defaulttype::Mat4x4f instrumentMtx = portalMtx * R_rot * T_insert;

    sofa::defaulttype::Mat3x3f rotM;
    for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++) {
            rotM[i][j] = instrumentMtx[i][j];
            m_toolRot[i][j] = portalMtx[i][j];
        }

    m_toolRot = m_toolRot.inverted();

    HapticAvatarDeviceController::Coord & posDevice = *d_posDevice.beginEdit();
    sofa::defaulttype::Quat orien;
    orien.fromMatrix(rotM);
    posDevice.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    posDevice.getOrientation() = orien;
    //std::cout << "posDevice: " << posDevice << std::endl;

    //Vec3f test = Vec3f(0, 1, 0);
    //Vec3f testT = rotM* test;
    //std::cout << "testT: " << testT << std::endl;

    d_posDevice.endEdit();
}



void HapticAvatarDeviceController::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!d_drawDevice.getValue() || !m_deviceReady)
        return;

    //vparams->drawTool()->saveLastState();
    //vparams->drawTool()->restoreLastState();

    float scale = 10.0f;
    vparams->drawTool()->drawSphere(m_debugToolPosition, 0.1f, defaulttype::Vec4f(1.0, 0.0, 0.0, 1.0));
    vparams->drawTool()->drawLine(m_debugToolPosition, m_debugToolPosition + m_debugForceVector*scale, defaulttype::Vec4f(1.0, 0.0, 0.0f, 1.0));
 

    {
        vparams->drawTool()->disableLighting();

        const HapticAvatarDeviceController::Coord & posDevice = d_posDevice.getValue();
        float glRadius = float(d_scale.getValue());
        vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
        vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
        vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));

        vparams->drawTool()->drawArrow(m_debugRootPosition.getCenter(), m_debugRootPosition.getCenter() + m_debugRootPosition.getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
        vparams->drawTool()->drawArrow(m_debugRootPosition.getCenter(), m_debugRootPosition.getCenter() + m_debugRootPosition.getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
        vparams->drawTool()->drawArrow(m_debugRootPosition.getCenter(), m_debugRootPosition.getCenter() + m_debugRootPosition.getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));
    }


    size_t newLine = d_fontSize.getValue();    
    int fontS = d_fontSize.getValue();
    const sofa::helper::fixed_array<float, 4>& dofV = d_toolValues.getValue();
    const sofa::helper::fixed_array<float, 4>& motV = d_motorOutput.getValue();
    defaulttype::Vec4f color(0.0, 1.0, 0.0, 1.0);

    std::string title =      "       Yaw   Pitch   Rot   Z";
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

    const sofa::helper::fixed_array<float, 3>& colF = d_collisionForce.getValue();
    float jTorq = d_jawTorq.getValue();

    ss.str(std::string());
    ss << std::fixed << std::setprecision(2) << "Collision  "
        << colF[0] << "    "
        << colF[1] << "    "
        << colF[2] << "    "
        << jTorq;

    vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
    newLine += fontS * 2;

    ss.str(std::string());
    ss << std::fixed << std::setprecision(2) << "Jaws opening  "
        << d_jawOpening.getValue() << "    ";

    vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
    newLine += fontS * 2;

}


void HapticAvatarDeviceController::handleEvent(core::objectmodel::Event *event)
{
    //if(m_errorDevice != 0)
    //    return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
       if (!m_deviceReady)
            return;

        updatePosition();
    }
}

} // namespace controller

} // namespace component

} // namespace sofa
