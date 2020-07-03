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
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>

namespace sofa::component::controller
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
    
    , d_jawOpening(initData(&d_jawOpening, 0.0f, "jawOpening", "jaws opening angle"))
    , d_toolPosition(initData(&d_toolPosition, "toolPosition", "jaws opening position"))
    
    , m_forceScale(initData(&m_forceScale, SReal(1.0), "forceScale", "jaws opening angle"))

    , d_portName(initData(&d_portName, std::string("//./COM3"),"portName", "position of the base of the part of the device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "position of the base of the part of the device"))
    , m_portId(-1)
    , l_portalMgr(initLink("portalManager", "link to portalManager"))
    , l_iboxCtrl(initLink("iboxController", "link to portalManager"))    

    , d_drawDeviceAxis(initData(&d_drawDeviceAxis, false, "drawDeviceAxis", "draw device"))
    , d_drawDebugForce(initData(&d_drawDebugForce, false, "drawDebugForce", "draw device"))
    , d_dumpThreadInfo(initData(&d_dumpThreadInfo, false, "dumpThreadInfo", "draw device"))
    
    , d_fontSize(initData(&d_fontSize, 12, "fontSize", "font size of statistics to display"))
    , m_deviceReady(false)
    , m_terminate(true)

    , m_HA_driver(nullptr)
    , m_portalMgr(nullptr)
    , m_iboxCtrl(nullptr)
    , m_forceFeedback(nullptr)
    , m_simulationStarted(false)
    , m_firstStep(true)
    , m_distance(1.0f)
    , m_detectionNP(nullptr)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_debugRootPosition = Vector3(0.0, 0.0, 0.0);
    m_debugForces.resize(6);
    m_toolRot.identity();

    HapticAvatarDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition.resize(8);
    d_toolPosition.endEdit();    
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


    m_intersectionMethod = getContext()->get<core::collision::Intersection>();
    m_detectionNP = getContext()->get<core::collision::NarrowPhaseDetection>();
    if (m_intersectionMethod == nullptr) { msg_error() << "m_intersectionMethod not found. Add an Intersection method in your scene.";}
    if (m_detectionNP == nullptr) { msg_error() << "NarrowPhaseDetection not found. Add a NarrowPhaseDetection method in your scene.";}


    SReal alarmDist = m_intersectionMethod->getAlarmDistance();
    SReal contactDist = m_intersectionMethod->getContactDistance();
    m_distance = alarmDist - contactDist;

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
        copy_thread.join();
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
    m_hapticData.hapticForces.resize(5);
    m_simuData.hapticForces.resize(5);
    haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_forceFeedback = context->get<LCPForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback != nullptr)
    {
        msg_info() << "ForceFeedback found";
    }
}


void HapticAvatarDeviceController::reinit()
{
    msg_info() << "HapticAvatarDeviceController::reinit()";
}


using namespace sofa::helper::system::thread;

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
    long targetSpeedLoop = 1; // Target loop speed: 1ms
    
    // Use computer tick for timer
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());
    
    ctime_t lastTime = CTime::getRefTime();
    std::cout << "start time: " << lastTime << " speed: " << speedTimerMs << std::endl;
    std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
    int cptLoop = 0;

    bool debugThread = _deviceCtrl->d_dumpThreadInfo.getValue();

    // Haptics Loop
    while (!terminate)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        ctime_t startTime = CTime::getRefTime();

        // Get all info from devices
        _deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();
        _deviceCtrl->m_hapticData.motorValues = _driver->getLastPWM();
        //_deviceCtrl->m_hapticData.collisionForces = _driver->getLastCollisionForce();

        // get info regarding jaws
        //float jtorq = _driver->getJawTorque();
        if (_deviceCtrl->m_iboxCtrl)
        {
            float angle = _deviceCtrl->m_iboxCtrl->getJawOpeningAngle();
            _deviceCtrl->m_hapticData.jawOpening = angle;
        }


        // Force feedback computation
        if (_deviceCtrl->m_simulationStarted && _deviceCtrl->m_forceFeedback)
        {            
            const HapticAvatarDeviceController::VecCoord& toolPosition = _deviceCtrl->d_toolPosition.getValue();
            sofa::defaulttype::Vector3 totalForce = sofa::defaulttype::Vector3(0, 0, 0);

            // Check main force feedback
            _deviceCtrl->m_forceFeedback->computeForce(toolPosition, _deviceCtrl->m_hapticData.hapticForces);

            bool contactShaft = false;
            totalForce = _deviceCtrl->m_hapticData.hapticForces[3].getLinear() + _deviceCtrl->m_hapticData.hapticForces[4].getLinear() + _deviceCtrl->m_hapticData.hapticForces[5].getLinear();
            

            
            for (int i = 0; i < 3; i++)
            {
                if (totalForce[i] > 0.0)
                {
                    contactShaft = true;
                    break;
                }
            }

            float damping = _deviceCtrl->m_forceScale.getValue();            
            if (contactShaft)
            {
                //std::cout << "_deviceCtrl->m_toolRot: " << _deviceCtrl->m_toolRot << std::endl;
                //_deviceCtrl->m_hapticData.collisionForces[0] = shaftForce[0];
                //_deviceCtrl->m_hapticData.collisionForces[1] = shaftForce[1];
                //_deviceCtrl->m_hapticData.collisionForces[2] = shaftForce[2];
                //std::cout << "haptic shaftForce: " << shaftForce << std::endl;  
                _driver->setManualForceVector(_deviceCtrl->m_toolRotInv * totalForce * damping, true);
            }
            else
                _driver->releaseForce();
            
        }

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        //if (duration < targetTicksPerLoop)
        //    std::cout << "Need to Wait!!!" << std::endl;
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }

        // timer dump
        cptLoop++;

        if (debugThread && cptLoop % 100 == 0)
        {
            ctime_t stepTime = CTime::getRefTime();
            ctime_t diffLoop = stepTime - lastTime;
            lastTime = stepTime;
            
            //_deviceCtrl->m_times.push_back(diffLoop* speedTimerMs);

            auto t2 = std::chrono::high_resolution_clock::now();
            
            auto duration = std::chrono::milliseconds(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
            t1 = t2;
            std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << " | " << duration.count() << std::endl;
        }
    }

    // ensure no force
    _driver->releaseForce();
    std::cout << "Haptics thread END!!" << std::endl;
}


void HapticAvatarDeviceController::CopyData(std::atomic<bool>& terminate, void * p_this)
{
    HapticAvatarDeviceController* _deviceCtrl = static_cast<HapticAvatarDeviceController*>(p_this);
    
    // Use computer tick for timer
    double targetSpeedLoop = 0.5; // Target loop speed: 0.5ms
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());

    ctime_t lastTime = CTime::getRefTime();
    std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
    int cptLoop = 0;
    // Haptics Loop
    while (!terminate)
    {
        ctime_t startTime = CTime::getRefTime();
        _deviceCtrl->m_simuData = _deviceCtrl->m_hapticData;

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }


        //if (cptLoop % 100 == 0)
        //{
        //    ctime_t stepTime = CTime::getRefTime();
        //    ctime_t diffLoop = stepTime - lastTime;
        //    lastTime = stepTime;
        //    //std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;
        //    std::cout << "Copy nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;            
        //}
        cptLoop++;
    }
}


void HapticAvatarDeviceController::updateAnglesAndLength(sofa::helper::fixed_array<float, 4> values)
{
    m_portalMgr->updatePostion(m_portId, values[Dof::YAW], values[Dof::PITCH]);
    d_toolValues.setValue(values);
}

int cpt = 0;

void HapticAvatarDeviceController::updatePosition()
{
    if (!m_HA_driver)
        return;

    updateAnglesAndLength(m_simuData.anglesAndLength);
    d_motorOutput.setValue(m_simuData.motorValues);
    d_collisionForce.setValue(m_simuData.collisionForces);
    d_jawOpening.setValue(m_simuData.jawOpening);

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
            m_toolRot[i][j] = instrumentMtx[i][j];
            m_PortalRot[i][j] = portalMtx[i][j];
        }

    m_toolRotInv = m_toolRot.inverted();


   // m_toolRot = rotM.inverted();
    sofa::defaulttype::Quat orien;
    orien.fromMatrix(rotM);

    // compute bati position
    HapticAvatarDeviceController::Coord rootPos = m_portalMgr->getPortalPosition(m_portId);
    rootPos.getOrientation() = orien;

    HapticAvatarDeviceController::Coord & posDevice = *d_posDevice.beginEdit();    
    posDevice.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    posDevice.getOrientation() = orien;
    d_posDevice.endEdit();

    // Update jaws rigid
    float _OpeningAngle = d_jawOpening.getValue() * m_jawsData.m_MaxOpeningAngle * 0.01f;
    HapticAvatarDeviceController::Coord jawUp;
    HapticAvatarDeviceController::Coord jawDown;
    
    jawUp.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, _OpeningAngle) + orien;
    jawDown.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, -_OpeningAngle) + orien;

    jawUp.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    jawDown.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    
    // Update jaws exterimies
    Vec3f posExtrem = Vec3f(0.0, -m_jawsData.m_jawLength, 0.0);
    HapticAvatarDeviceController::Coord jawUpExtrem = jawUp;
    HapticAvatarDeviceController::Coord jawDownExtrem = jawDown;
        
    jawUpExtrem.getCenter() += jawUpExtrem.getOrientation().rotate(posExtrem);
    jawDownExtrem.getCenter() += jawDownExtrem.getOrientation().rotate(posExtrem);

    // Udpate articulated device
    HapticAvatarDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition[0] = rootPos;
    toolPosition[1] = rootPos;
    toolPosition[2] = rootPos;
    toolPosition[3] = posDevice;

    toolPosition[4] = jawUp;
    toolPosition[5] = jawDown;
    toolPosition[6] = jawUpExtrem;
    toolPosition[7] = jawDownExtrem;
    d_toolPosition.endEdit();

   /* if (cpt == 20)
    {
        bool test = false;
        for (int i = 0; i < m_simuData.hapticForces.size(); i++)
        {
            double norm = m_simuData.hapticForces[i].getLinear().norm();
            if (norm > 0.001)
            {
                test = true;
                std::cout << i << " | Position: " << toolPosition[i] << " | force: " << m_simuData.hapticForces[i].getLinear() << " | angular: " << m_simuData.hapticForces[i].getAngular()
                    << " => " << m_simuData.hapticForces[i].getLinear().norm() << std::endl;
            }
            
        }

        if (test)
            std::cout << std::endl;
        cpt = 0;
    }
    cpt++;*/
}



void HapticAvatarDeviceController::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    if (d_drawDeviceAxis.getValue())
    {
        const HapticAvatarDeviceController::VecCoord & toolPosition = d_toolPosition.getValue();
        float glRadius = float(d_scale.getValue());

        for (unsigned int i = 0; i < toolPosition.size(); ++i)
        {
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));
        }
    }
    
    if (d_drawDebugForce.getValue())
    {
        const HapticAvatarDeviceController::VecCoord & toolPosition = d_toolPosition.getValue();
        const HapticAvatarDeviceController::VecDeriv& force = m_simuData.hapticForces;

        //if (force.size() > 6)
        //{
        //    Vec3 dir = force[3].getLinear();
        //    Vec3 ang = force[3].getAngular();
        //    //dir.normalize();
        //    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + dir * 50, defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0));
        //    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + ang * 50, defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0));

        //    Vec3 dir2 = force[4].getLinear();
        //    Vec3 ang2 = force[4].getAngular();
        //    //dir2.normalize();
        //    vparams->drawTool()->drawLine(toolPosition[6].getCenter(), toolPosition[6].getCenter() + dir2 * 50, defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0));
        //    vparams->drawTool()->drawLine(toolPosition[6].getCenter(), toolPosition[6].getCenter() + ang2 * 50, defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0));

        //    Vec3 dir3 = force[5].getLinear();
        //    Vec3 ang3 = force[5].getAngular();
        //    //dir3.normalize();
        //    vparams->drawTool()->drawLine(toolPosition[7].getCenter(), toolPosition[7].getCenter() + dir3 * 50, defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0));
        //    vparams->drawTool()->drawLine(toolPosition[7].getCenter(), toolPosition[7].getCenter() + ang3 * 50, defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0));

        //    Vec3 dirTotal = dir + dir2 + dir3;
        //    Vec3 angTotal = ang + ang2 + ang3;
        //    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + dirTotal * 50, defaulttype::Vec4f(1.0f, 1.0f, 1.0f, 1.0));
        //    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + angTotal * 50, defaulttype::Vec4f(0.0f, 0.0f, 1.0f, 1.0));

        //}
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
    
    return;

    



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


void HapticAvatarDeviceController::retrieveCollisions()
{
    contactsSimu.clear();

    // get the collision output
    const core::collision::NarrowPhaseDetection::DetectionOutputMap& detectionOutputs = m_detectionNP->getDetectionOutputs();
    if (detectionOutputs.size() == 0) {
        contactsHaptic = contactsSimu;
        return;
    }

    //std::cout << "detectionOutputs: " << detectionOutputs.size() << " count: " << m_detectionNP->getPrimitiveTestCount() << std::endl;    
    const ContactVector* contacts = NULL;
    for (core::collision::NarrowPhaseDetection::DetectionOutputMap::const_iterator it = detectionOutputs.begin(); it != detectionOutputs.end(); ++it)
    {
        contacts = dynamic_cast<const ContactVector*>(it->second);
        if (contacts == nullptr || contacts->size() == 0) {
            continue;
        }

        size_t ncontacts = contacts->size();
        if (ncontacts == 0) {
            continue;
        }
        
        sofa::core::CollisionModel* collMod1 = it->first.first;
        sofa::core::CollisionModel* collMod2 = it->first.second;

        //std::cout << " - collMod1: " << collMod1->getName() << " | collMod2: " << collMod2->getName() << std::endl;

        int id = -1;
        if (collMod1->hasTag(sofa::core::objectmodel::Tag("toolCollision")))
        {
            id = 0;
        }
        else if (collMod2->hasTag(sofa::core::objectmodel::Tag("toolCollision")))
        {
            id = 1;
        }
               
        if (id == -1) { // others collision than with tool
            continue;
        }
        
        //std::cout << " - ncontacts: " << ncontacts << std::endl;
        for (size_t j = 0; j < ncontacts; ++j)
        {
            const ContactVector::value_type& c = (*contacts)[j];
            HapticContact contact;
            if (id == 0)
            {
                contact.m_toolPosition = c.point[0];
                contact.m_objectPosition = c.point[1];
                contact.m_normal = c.normal * (-1); /// Normal of the contact, pointing outward from the first model                
            }
            else
            {
                contact.m_toolPosition = c.point[1];
                contact.m_objectPosition = c.point[0];
                contact.m_normal = c.normal ;
            }
            contact.m_force = (contact.m_toolPosition - contact.m_objectPosition).normalized();
            contact.distance = (m_distance - c.value)*(m_distance - c.value);
            //std::cout << "contact.distance: " << contact.distance << std::endl;

            contactsSimu.push_back(contact);
        }
    }

    contactsHaptic = contactsSimu;
    //std::cout << " - contactsSimu: " << contactsSimu.size() << std::endl;
}


void HapticAvatarDeviceController::handleEvent(core::objectmodel::Event *event)
{
    //if(m_errorDevice != 0)
    //    return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        if (!m_deviceReady)
            return;
        
        m_simulationStarted = true;
        updatePosition();
    }
    else if (simulation::CollisionEndEvent::checkEventType(event))
    {
        retrieveCollisions();
    }
}

} // namespace sofa::component::controller