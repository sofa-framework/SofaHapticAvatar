/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_TipDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_Defines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>

namespace sofa::HapticAvatar
{

using namespace HapticAvatar;
using namespace sofa::helper::system::thread;

int HapticAvatar_TipDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_TipDeviceController >()
    ;


//constructeur
HapticAvatar_TipDeviceController::HapticAvatar_TipDeviceController()
    : HapticAvatar_BaseDeviceController()
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

    HapticAvatar_TipDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition.resize(8);
    d_toolPosition.endEdit();    
}


bool HapticAvatar_TipDeviceController::createHapticThreads()
{
    haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);
    m_hapticData.hapticForces.resize(5);
    m_simuData.hapticForces.resize(5);

    return true;
}


void HapticAvatar_TipDeviceController::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics thread" << std::endl;

    HapticAvatar_TipDeviceController* _deviceCtrl = static_cast<HapticAvatar_TipDeviceController*>(p_this);
    HapticAvatar_Driver* _driver = static_cast<HapticAvatar_Driver*>(p_driver);

    if (_deviceCtrl == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_TipDeviceController cast failed");
        return;
    }

    if (_driver == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_Driver cast failed");
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

        // Force feedback computation
        if (_deviceCtrl->m_simulationStarted && _deviceCtrl->m_forceFeedback)
        {            
            const HapticAvatar_TipDeviceController::VecCoord& toolPosition = _deviceCtrl->d_toolPosition.getValue();
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


void HapticAvatar_TipDeviceController::CopyData(std::atomic<bool>& terminate, void * p_this)
{
    HapticAvatar_TipDeviceController* _deviceCtrl = static_cast<HapticAvatar_TipDeviceController*>(p_this);
    
    // Use computer tick for timer
    double targetSpeedLoop = 0.5; // Target loop speed: 0.5ms
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());

    ctime_t lastTime = CTime::getRefTime();
    //std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
    //int cptLoop = 0;

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
        //cptLoop++;
    }
}


void HapticAvatar_TipDeviceController::updatePositionImpl()
{
    // m_toolRot = rotM.inverted();
    sofa::defaulttype::Quat orien;
    orien.fromMatrix(m_toolRot);

    // compute bati position
    HapticAvatar_BaseDeviceController::Coord rootPos = m_portalMgr->getPortalPosition(m_portId);
    rootPos.getOrientation() = orien;

    HapticAvatar_BaseDeviceController::Coord & posDevice = *d_posDevice.beginEdit();
    posDevice.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    posDevice.getOrientation() = orien;
    d_posDevice.endEdit();

    // Update jaws rigid
    float _OpeningAngle = 0.0f;// d_info_jawOpening.getValue() * m_jawsData.m_MaxOpeningAngle * 0.01f;
    HapticAvatar_BaseDeviceController::Coord jawUp;
    HapticAvatar_BaseDeviceController::Coord jawDown;

    jawUp.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, _OpeningAngle) + orien;
    jawDown.getOrientation() = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, -_OpeningAngle) + orien;

    jawUp.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);
    jawDown.getCenter() = Vec3f(m_instrumentMtx[0][3], m_instrumentMtx[1][3], m_instrumentMtx[2][3]);

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


} // namespace sofa::HapticAvatar