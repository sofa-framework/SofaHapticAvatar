/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_Defines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>

namespace sofa::HapticAvatar
{

using namespace HapticAvatar;
using namespace sofa::helper::system::thread;

//constructeur
HapticAvatar_BaseDeviceController::HapticAvatar_BaseDeviceController()
    : d_scale(initData(&d_scale, 1.0, "scale", "Default scale applied to the Phantom Coordinates"))
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

    , d_drawDeviceAxis(initData(&d_drawDeviceAxis, false, "drawDeviceAxis", "draw device"))
    , d_drawDebugForce(initData(&d_drawDebugForce, false, "drawDebugForce", "draw device"))
    , d_dumpThreadInfo(initData(&d_dumpThreadInfo, false, "dumpThreadInfo", "draw device"))
    
    , d_fontSize(initData(&d_fontSize, 12, "fontSize", "font size of statistics to display"))
    , m_deviceReady(false)
    , m_terminate(true)

    , m_HA_driver(nullptr)
    , m_portalMgr(nullptr)
    , m_forceFeedback(nullptr)
    , m_simulationStarted(false)
    , m_firstStep(true)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_debugRootPosition = Vector3(0.0, 0.0, 0.0);
    m_debugForces.resize(6);
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
    if (m_terminate == false)
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

    //haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    //copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);
    //m_hapticData.hapticForces.resize(5);
    //m_simuData.hapticForces.resize(5);

    m_terminate = false;
    createHapticThreads();
    
    m_deviceReady = true;
}


//void HapticAvatar_BaseDeviceController::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
//{ 
//    std::cout << "Haptics thread" << std::endl;
//
//    HapticAvatar_BaseDeviceController* _deviceCtrl = static_cast<HapticAvatar_BaseDeviceController*>(p_this);
//    HapticAvatar_Driver* _driver = static_cast<HapticAvatar_Driver*>(p_driver);
//
//    if (_deviceCtrl == nullptr)
//    {
//        msg_error("Haptics Thread: HapticAvatar_BaseDeviceController cast failed");
//        return;
//    }
//
//    if (_driver == nullptr)
//    {
//        msg_error("Haptics Thread: HapticAvatar_Driver cast failed");
//        return;
//    }
//
//    // Loop Timer
//    HANDLE h_timer;
//    long targetSpeedLoop = 1; // Target loop speed: 1ms
//    
//    // Use computer tick for timer
//    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
//    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
//    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());
//    
//    ctime_t lastTime = CTime::getRefTime();
//    std::cout << "start time: " << lastTime << " speed: " << speedTimerMs << std::endl;
//    std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
//    int cptLoop = 0;
//
//    bool debugThread = _deviceCtrl->d_dumpThreadInfo.getValue();
//
//    // Haptics Loop
//    while (!terminate)
//    {
//        auto t1 = std::chrono::high_resolution_clock::now();
//        ctime_t startTime = CTime::getRefTime();
//
//        // Get all info from devices
//        _deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();
//        _deviceCtrl->m_hapticData.motorValues = _driver->getLastPWM();
//        //_deviceCtrl->m_hapticData.collisionForces = _driver->getLastCollisionForce();
//
//        // Force feedback computation
//        if (_deviceCtrl->m_simulationStarted && _deviceCtrl->m_forceFeedback)
//        {            
//            const HapticAvatar_BaseDeviceController::VecCoord& toolPosition = _deviceCtrl->d_toolPosition.getValue();
//            sofa::defaulttype::Vector3 totalForce = sofa::defaulttype::Vector3(0, 0, 0);
//
//            // Check main force feedback
//            _deviceCtrl->m_forceFeedback->computeForce(toolPosition, _deviceCtrl->m_hapticData.hapticForces);
//
//            bool contactShaft = false;
//            totalForce = _deviceCtrl->m_hapticData.hapticForces[3].getLinear() + _deviceCtrl->m_hapticData.hapticForces[4].getLinear() + _deviceCtrl->m_hapticData.hapticForces[5].getLinear();
//            
//
//            
//            for (int i = 0; i < 3; i++)
//            {
//                if (totalForce[i] > 0.0)
//                {
//                    contactShaft = true;
//                    break;
//                }
//            }
//
//            float damping = _deviceCtrl->m_forceScale.getValue();            
//            if (contactShaft)
//            {
//                //std::cout << "_deviceCtrl->m_toolRot: " << _deviceCtrl->m_toolRot << std::endl;
//                //_deviceCtrl->m_hapticData.collisionForces[0] = shaftForce[0];
//                //_deviceCtrl->m_hapticData.collisionForces[1] = shaftForce[1];
//                //_deviceCtrl->m_hapticData.collisionForces[2] = shaftForce[2];
//                //std::cout << "haptic shaftForce: " << shaftForce << std::endl;  
//                _driver->setManualForceVector(_deviceCtrl->m_toolRotInv * totalForce * damping, true);
//            }
//            else
//                _driver->releaseForce();
//            
//        }
//
//        ctime_t endTime = CTime::getRefTime();
//        ctime_t duration = endTime - startTime;
//
//        // If loop is quicker than the target loop speed. Wait here.
//        //if (duration < targetTicksPerLoop)
//        //    std::cout << "Need to Wait!!!" << std::endl;
//        while (duration < targetTicksPerLoop)
//        {
//            endTime = CTime::getRefTime();
//            duration = endTime - startTime;
//        }
//
//        // timer dump
//        cptLoop++;
//
//        if (debugThread && cptLoop % 100 == 0)
//        {
//            ctime_t stepTime = CTime::getRefTime();
//            ctime_t diffLoop = stepTime - lastTime;
//            lastTime = stepTime;
//            
//            //_deviceCtrl->m_times.push_back(diffLoop* speedTimerMs);
//
//            auto t2 = std::chrono::high_resolution_clock::now();
//            
//            auto duration = std::chrono::milliseconds(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
//            t1 = t2;
//            std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << " | " << duration.count() << std::endl;
//        }
//    }
//
//    // ensure no force
//    _driver->releaseForce();
//    std::cout << "Haptics thread END!!" << std::endl;
//}


//void HapticAvatar_BaseDeviceController::CopyData(std::atomic<bool>& terminate, void * p_this)
//{
//    HapticAvatar_BaseDeviceController* _deviceCtrl = static_cast<HapticAvatar_BaseDeviceController*>(p_this);
//    
//    // Use computer tick for timer
//    double targetSpeedLoop = 0.5; // Target loop speed: 0.5ms
//    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
//    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
//    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());
//
//    ctime_t lastTime = CTime::getRefTime();
//    std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
//    int cptLoop = 0;
//    // Haptics Loop
//    while (!terminate)
//    {
//        ctime_t startTime = CTime::getRefTime();
//        _deviceCtrl->m_simuData = _deviceCtrl->m_hapticData;
//
//        ctime_t endTime = CTime::getRefTime();
//        ctime_t duration = endTime - startTime;
//
//        // If loop is quicker than the target loop speed. Wait here.
//        while (duration < targetTicksPerLoop)
//        {
//            endTime = CTime::getRefTime();
//            duration = endTime - startTime;
//        }
//
//
//        //if (cptLoop % 100 == 0)
//        //{
//        //    ctime_t stepTime = CTime::getRefTime();
//        //    ctime_t diffLoop = stepTime - lastTime;
//        //    lastTime = stepTime;
//        //    //std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;
//        //    std::cout << "Copy nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;            
//        //}
//        cptLoop++;
//    }
//}


void HapticAvatar_BaseDeviceController::updateAnglesAndLength(sofa::helper::fixed_array<float, 4> values)
{
    m_portalMgr->updatePostion(m_portId, values[Dof::YAW], values[Dof::PITCH]);
    d_toolValues.setValue(values);
}



void HapticAvatar_BaseDeviceController::updatePosition()
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
    HapticAvatar_BaseDeviceController::Coord rootPos = m_portalMgr->getPortalPosition(m_portId);
    rootPos.getOrientation() = orien;

    HapticAvatar_BaseDeviceController::Coord & posDevice = *d_posDevice.beginEdit();    
    posDevice.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    posDevice.getOrientation() = orien;
    d_posDevice.endEdit();

    // Update jaws rigid
    float _OpeningAngle = 0.0f;// d_jawOpening.getValue() * m_jawsData.m_MaxOpeningAngle * 0.01f;
    HapticAvatar_BaseDeviceController::Coord jawUp;
    HapticAvatar_BaseDeviceController::Coord jawDown;
    
    jawUp.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, _OpeningAngle) + orien;
    jawDown.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, -_OpeningAngle) + orien;

    jawUp.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    jawDown.getCenter() = Vec3f(instrumentMtx[0][3], instrumentMtx[1][3], instrumentMtx[2][3]);
    
    // Update jaws exterimies
    Vec3f posExtrem = Vec3f(0.0, /*-m_jawsData.m_jawLength*/-20.0, 0.0);
    HapticAvatar_BaseDeviceController::Coord jawUpExtrem = jawUp;
    HapticAvatar_BaseDeviceController::Coord jawDownExtrem = jawDown;
        
    jawUpExtrem.getCenter() += jawUpExtrem.getOrientation().rotate(posExtrem);
    jawDownExtrem.getCenter() += jawDownExtrem.getOrientation().rotate(posExtrem);

    // Udpate articulated device
    HapticAvatar_BaseDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition[0] = rootPos;
    toolPosition[1] = rootPos;
    toolPosition[2] = rootPos;
    toolPosition[3] = posDevice;

    toolPosition[4] = jawUp;
    toolPosition[5] = jawDown;
    toolPosition[6] = jawUpExtrem;
    toolPosition[7] = jawDownExtrem;
    d_toolPosition.endEdit();

}



void HapticAvatar_BaseDeviceController::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

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
    
    if (d_drawDebugForce.getValue())
    {
        const HapticAvatar_BaseDeviceController::VecCoord & toolPosition = d_toolPosition.getValue();
        const HapticAvatar_BaseDeviceController::VecDeriv& force = m_simuData.hapticForces;

        
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