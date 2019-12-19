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


HapticAvatarJaws::HapticAvatarJaws()
    : m_MaxOpeningAngle(60.0f)
    , m_jawLength(20.0f)
    , m_jaw1Radius(1.5f)
    , m_jaw2Radius(1.5f)
    , m_shaftRadius(2.5f)
{

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
    
    , d_jawUp(initData(&d_jawUp, "jawUp", "jaws opening position"))
    , d_jawDown(initData(&d_jawDown, "jawDown", "jaws opening position"))
    , d_jawOpening(initData(&d_jawOpening, 0.0f, "jawOpening", "jaws opening angle"))
    , d_testPosition(initData(&d_testPosition, "testPosition", "jaws opening position"))
    

    , d_portName(initData(&d_portName, std::string("//./COM3"),"portName", "position of the base of the part of the device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "position of the base of the part of the device"))
    , m_portId(-1)
    , l_portalMgr(initLink("portalManager", "link to portalManager"))
    , l_iboxCtrl(initLink("iboxController", "link to portalManager"))    

    , d_drawDeviceAxis(initData(&d_drawDeviceAxis, false, "drawDeviceAxis", "draw device"))
    , d_drawDebugForce(initData(&d_drawDebugForce, false, "drawDebugForce", "draw device"))
    , d_fontSize(initData(&d_fontSize, 12, "fontSize", "font size of statistics to display"))
    , m_deviceReady(false)
    , m_terminate(true)

    , m_HA_driver(nullptr)
    , m_portalMgr(nullptr)
    , m_iboxCtrl(nullptr)
    , m_forceFeedback(nullptr)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_debugRootPosition = Vector3(0.0, 0.0, 0.0);
    m_debugForces.resize(6);
    m_toolRot.identity();

    HapticAvatarDeviceController::VecCoord & testPosition = *d_testPosition.beginEdit();
    testPosition.resize(6);
    d_testPosition.endEdit();
}


HapticAvatarDeviceController::~HapticAvatarDeviceController()
{
    clearDevice();
    if (m_HA_driver)
    {
        delete m_HA_driver;
        m_HA_driver = nullptr;
    }
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
    //int res = m_HA_driver->resetDevice(15);
    //if (res == -1)
    //    std::cerr << "## Error, Reset failed!" << std::endl;
    //else
    //    std::cout << "Reset succeed return value: '" << res << "'" << std::endl;

    // release force
    m_HA_driver->releaseForce();



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


    // get ibox if one
    if (!l_iboxCtrl.empty())
    {
        m_iboxCtrl = l_iboxCtrl.get();
        if (m_iboxCtrl != nullptr)
        {
            msg_info() << "Device " << d_hapticIdentity.getValue() << " connected with IBox: " << m_iboxCtrl->d_hapticIdentity.getValue();
        }
    }


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

        // Get all info from devices
        sofa::helper::fixed_array<float, 4> toolValues = _driver->getAngles_AndLength();
        sofa::helper::fixed_array<float, 4> motorValues = _driver->getLastPWM();
        sofa::helper::fixed_array<float, 3> collForces = _driver->getLastCollisionForce();

        _deviceCtrl->updateAnglesAndLength(toolValues);
        _deviceCtrl->d_motorOutput.setValue(motorValues);
        _deviceCtrl->d_collisionForce.setValue(collForces);


        // get info regarding jaws
        float jtorq = _driver->getJawTorque();
        if (_deviceCtrl->m_iboxCtrl)
        {
            float angle = _deviceCtrl->m_iboxCtrl->getJawOpeningAngle();
            _deviceCtrl->d_jawOpening.setValue(angle);
        }


        // Force feedback computation
        if (_deviceCtrl->m_forceFeedback)
        {
            const HapticAvatarDeviceController::VecCoord& testPosition = _deviceCtrl->d_testPosition.getValue();

            // Check main force feedback
            Vector3 tipPosition = testPosition[1].getCenter();
            Vector3 shaftForce;
            _deviceCtrl->m_forceFeedback->computeForce(tipPosition[0], tipPosition[1], tipPosition[2], 0, 0, 0, 0, shaftForce[0], shaftForce[1], shaftForce[2]);

            bool contactShaft = false;
            for (int i = 0; i < 3; i++)
            {
                if (shaftForce[i] != 0.0)
                {
                    contactShaft = true;
                    break;
                }
            }            

            // Check jaws force feedback
            Vector3 jawUpForce, jawDownForce;
            Vector3 jawUpPosition = testPosition[3].getCenter();
            Vector3 jawDownPosition = testPosition[5].getCenter();
            
            _deviceCtrl->m_forceFeedback->computeForce(jawUpPosition[0], jawUpPosition[1], jawUpPosition[2], 0, 0, 0, 0, jawUpForce[0], jawUpForce[1], jawUpForce[2]);
            _deviceCtrl->m_forceFeedback->computeForce(jawDownPosition[0], jawDownPosition[1], jawDownPosition[2], 0, 0, 0, 0, jawDownForce[0], jawDownForce[1], jawDownForce[2]);

            // save debug info
            _deviceCtrl->m_debugForces[0] = tipPosition;
            _deviceCtrl->m_debugForces[1] = shaftForce;
            _deviceCtrl->m_debugForces[2] = jawUpPosition;
            _deviceCtrl->m_debugForces[3] = jawUpForce;
            _deviceCtrl->m_debugForces[4] = jawDownPosition;
            _deviceCtrl->m_debugForces[5] = jawDownForce;

            /*std::cout << "tipPosition: " << tipPosition << " | shaftForce: " << shaftForce << std::endl;
            std::cout << "jawUpPosition: " << jawUpPosition << " | jawUpForce: " << jawUpForce << std::endl;
            std::cout << "jawDownPosition: " << jawDownPosition << " | jawDownForce: " << jawDownForce << std::endl;*/

            if (contactShaft)
            {
                std::cout << "_deviceCtrl->m_toolRot: " << _deviceCtrl->m_toolRot << std::endl;
                std::cout << "haptic shaftForce: " << shaftForce << std::endl;

                _driver->setForceVector(_deviceCtrl->m_toolRot * shaftForce);
            }
            else
                _driver->releaseForce();

            
            //bool contact = false;
            //for (int i = 0; i < 3; i++)
            //{
            //    if (currentForce[i] != 0.0)
            //    {
            //        //driver->m_isInContact = true;
            //        contact = true;
            //        break;
            //    }
            //}
 
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

    HapticAvatarDeviceController::VecCoord & testPosition = *d_testPosition.beginEdit();
    testPosition.resize(6);

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
    testPosition[0] = posDevice;
    testPosition[1] = posDevice;

    // update jaws
    HapticAvatarDeviceController::Coord & jawUp = *d_jawUp.beginEdit();    
    HapticAvatarDeviceController::Coord & jawDown = *d_jawDown.beginEdit();
    float _OpeningAngle = d_jawOpening.getValue() * m_jawsData.m_MaxOpeningAngle * 0.01f;

    jawUp.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, _OpeningAngle) + orien;
    jawDown.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, -_OpeningAngle) + orien;

    jawUp.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    jawDown.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    
    HapticAvatarDeviceController::Coord jawUpExtrem = jawUp;
    HapticAvatarDeviceController::Coord jawDownExtrem = jawDown;


    Vec3f posExtrem = Vec3f(0.0, -m_jawsData.m_jawLength, 0.0);
    jawUpExtrem.getCenter() += jawUpExtrem.getOrientation().rotate(posExtrem);
    jawDownExtrem.getCenter() += jawDownExtrem.getOrientation().rotate(posExtrem);
    
    testPosition[2] = jawUp;
    testPosition[3] = jawUpExtrem;

    testPosition[4] = jawDown;
    testPosition[5] = jawDownExtrem;
    
    d_jawUp.endEdit();
    d_jawDown.endEdit();
    d_testPosition.endEdit();
}



void HapticAvatarDeviceController::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    // vparams->drawTool()->disableLighting();

    if (d_drawDeviceAxis.getValue())
    {
        //    const HapticAvatarDeviceController::Coord & posDevice = d_posDevice.getValue();
        //    float glRadius = float(d_scale.getValue());
        //    vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
        //    vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
        //    vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));

        const HapticAvatarDeviceController::VecCoord & testPosition = d_testPosition.getValue();
        float glRadius = float(d_scale.getValue());
        for (unsigned int i = 0; i < testPosition.size(); ++i)
        {
            vparams->drawTool()->drawArrow(testPosition[i].getCenter(), testPosition[i].getCenter() + testPosition[i].getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
            vparams->drawTool()->drawArrow(testPosition[i].getCenter(), testPosition[i].getCenter() + testPosition[i].getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
            vparams->drawTool()->drawArrow(testPosition[i].getCenter(), testPosition[i].getCenter() + testPosition[i].getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));
        }
    }
        

    if (!d_drawDebugForce.getValue())
        return;
    

    for (unsigned int i = 0; i < m_debugForces.size()*0.5; ++i)
    {
        vparams->drawTool()->drawLine(m_debugForces[i * 2], m_debugForces[i * 2] + m_debugForces[(i * 2) + 1] * 5.0f, defaulttype::Vec4f(1.0, 0.0, 0.0f, 1.0));
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
