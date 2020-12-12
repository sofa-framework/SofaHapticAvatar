/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_DirectGrasperDeviceController.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

int HapticAvatar_DirectGrasperDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_DirectGrasperDeviceController >()
    ;


//constructeur
HapticAvatar_DirectGrasperDeviceController::HapticAvatar_DirectGrasperDeviceController()
    : HapticAvatar_RigidDeviceController()
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

    HapticAvatar_DirectGrasperDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition.resize(8);
    d_toolPosition.endEdit();    
}



//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_DirectGrasperDeviceController::initImpl()
{
    // get ibox if one
    if (!l_iboxCtrl.empty())
    {
        m_iboxCtrl = l_iboxCtrl.get();
        if (m_iboxCtrl != nullptr)
        {
            msg_info() << "Device " << d_hapticIdentity.getValue() << " connected with IBox: " << m_iboxCtrl->d_hapticIdentity.getValue();
        }
    }

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_forceFeedback = context->get<LCPForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback != nullptr)
    {
        msg_info() << "ForceFeedback found";
    }
}


bool HapticAvatar_DirectGrasperDeviceController::createHapticThreads()
{   
    m_terminate = false;
    haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);

    return true;
}


void HapticAvatar_DirectGrasperDeviceController::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics thread" << std::endl;

    HapticAvatar_DirectGrasperDeviceController* _deviceCtrl = static_cast<HapticAvatar_DirectGrasperDeviceController*>(p_this);
    HapticAvatar_Driver* _driver = static_cast<HapticAvatar_Driver*>(p_driver);

    if (_deviceCtrl == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_DirectGrasperDeviceController cast failed");
        return;
    }

    if (_driver == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_Driver cast failed");
        return;
    }

    // Loop Timer
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
    SReal damping = _deviceCtrl->m_forceScale.getValue();

    int cptF = 0;
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

        
        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
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


void HapticAvatar_DirectGrasperDeviceController::CopyData(std::atomic<bool>& terminate, void * p_this)
{
    HapticAvatar_DirectGrasperDeviceController* _deviceCtrl = static_cast<HapticAvatar_DirectGrasperDeviceController*>(p_this);
    
    // Use computer tick for timer
    ctime_t targetSpeedLoop = 1/2; // Target loop speed: 0.5ms
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

void HapticAvatar_DirectGrasperDeviceController::updatePositionImpl()
{
    if (!m_HA_driver)
        return;

    sofa::defaulttype::Quat orien;
    orien.fromMatrix(m_toolRot);

    // compute bati position
    HapticAvatar_DirectGrasperDeviceController::Coord rootPos = m_portalMgr->getPortalPosition(m_portId);
    rootPos.getOrientation() = orien;

    HapticAvatar_DirectGrasperDeviceController::Coord & posDevice = *d_posDevice.beginEdit();    
    posDevice.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    posDevice.getOrientation() = orien;
    d_posDevice.endEdit();

    // Update jaws rigid
    float _OpeningAngle = m_simuData.jawOpening * m_jawsData.m_MaxOpeningAngle * 0.01f;
    HapticAvatar_DirectGrasperDeviceController::Coord jawUp;
    HapticAvatar_DirectGrasperDeviceController::Coord jawDown;
    
    jawUp.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, _OpeningAngle) + orien;
    jawDown.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, -_OpeningAngle) + orien;

    jawUp.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    jawDown.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    
    // Update jaws exterimies
    Vec3f posExtrem = Vec3f(0.0, -m_jawsData.m_jawLength, 0.0);
    HapticAvatar_DirectGrasperDeviceController::Coord jawUpExtrem = jawUp;
    HapticAvatar_DirectGrasperDeviceController::Coord jawDownExtrem = jawDown;
        
    jawUpExtrem.getCenter() += jawUpExtrem.getOrientation().rotate(posExtrem);
    jawDownExtrem.getCenter() += jawDownExtrem.getOrientation().rotate(posExtrem);

    // Udpate articulated device
    HapticAvatar_DirectGrasperDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
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



void HapticAvatar_DirectGrasperDeviceController::handleEvent(core::objectmodel::Event *event)
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