/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatarEmulator.h>
#include <SofaHapticAvatar/HapticAvatarDefines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>

namespace sofa::component::controller
{

using namespace HapticAvatar;

int HapticAvatarEmulatorClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatarEmulator >()
    ;


//constructeur
HapticAvatarEmulator::HapticAvatarEmulator()
    : HapticAvatarDeviceController()
{
    this->f_listening.setValue(true);
}



void HapticAvatarEmulator::bwdInit()
{   
    msg_info() << "HapticAvatarEmulator::bwdInit()";

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
    haptic_thread = std::thread(HapticsEmulated, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);

    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_forceFeedback = context->get<ForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback != nullptr)
    {
        msg_info() << "ForceFeedback found";
    }

}



using namespace sofa::helper::system::thread;

void HapticAvatarEmulator::HapticsEmulated(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics Emulator thread" << std::endl;
    HapticAvatarEmulator* _deviceCtrl = static_cast<HapticAvatarEmulator*>(p_this);
    HapticAvatarDriver* _driver = static_cast<HapticAvatarDriver*>(p_driver);
   
    if (_deviceCtrl == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatarEmulator cast failed");
        return;
    }

    if (_driver == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatarEmulator cast failed");
        return;
    }

    // Loop Timer
    HANDLE h_timer;
    long targetSpeedLoop = 0.5; // Target loop speed: 1ms

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

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;


        // If loop is quicker than the target loop speed. Wait here.
        //if (duration < targetTicksPerLoop)
        //    std::cout << "Need to Wait!!!" << std::endl;
        /*while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }*/

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
    std::cout << "Haptics Emulator thread END!!" << std::endl;
}

//
//void HapticAvatarEmulator::updatePosition()
//{
//    //_deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();
//
//    // Loop Timer
//    HANDLE h_timer;
//    long targetSpeedLoop = 0.5; // Target loop speed: 1ms
//
//                                // Use computer tick for timer
//    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
//    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
//    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());
//
//    ctime_t lastTime = CTime::getRefTime();
//    std::cout << "start time: " << lastTime << " speed: " << speedTimerMs << std::endl;
//    std::cout << "refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
//    int cptLoop = 0;
//
//    // Haptics Loop
//    for (int i=0; i<1000; i++)
//    {
//        auto t1 = std::chrono::high_resolution_clock::now();
//        ctime_t startTime = CTime::getRefTime();
//
//        // Get all info from devices
//        m_hapticData.anglesAndLength = m_HA_driver->getAngles_AndLength();
//        m_hapticData.motorValues = m_HA_driver->getLastPWM();
//        m_hapticData.collisionForces = m_HA_driver->getLastCollisionForce();
//
//        ctime_t endTime = CTime::getRefTime();
//        ctime_t duration = endTime - startTime;
//
//
//        // If loop is quicker than the target loop speed. Wait here.
//        //if (duration < targetTicksPerLoop)
//        //    std::cout << "Need to Wait!!!" << std::endl;
//        /*while (duration < targetTicksPerLoop)
//        {
//        endTime = CTime::getRefTime();
//        duration = endTime - startTime;
//        }*/
//
//        // timer dump
//        cptLoop++;
//
//        if (cptLoop % 100 == 0)
//        {
//            ctime_t stepTime = CTime::getRefTime();
//            ctime_t diffLoop = stepTime - lastTime;
//            lastTime = stepTime;
//            //std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;
//            m_times.push_back(diffLoop* speedTimerMs);
//
//            auto t2 = std::chrono::high_resolution_clock::now();
//
//            auto duration = std::chrono::milliseconds(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
//            t1 = t2;
//            std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << " | " << duration.count() << std::endl;
//        }
//    }
//}



void HapticAvatarEmulator::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;
}


void HapticAvatarEmulator::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
       if (!m_deviceReady)
            return;

        updatePosition();
    }
}

} // namespace sofa::component::controller