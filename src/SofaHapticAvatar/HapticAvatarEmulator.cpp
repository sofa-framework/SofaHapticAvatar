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
    : m_HA_driver(nullptr)
    , d_portName(initData(&d_portName, std::string("//./COM3"), "portName", "position of the base of the part of the device"))
    , m_portId(-1)
    , l_portalMgr(initLink("portalManager", "link to portalManager"))
    , l_iboxCtrl(initLink("iboxController", "link to portalManager"))
{
    this->f_listening.setValue(true);
   

}


HapticAvatarEmulator::~HapticAvatarEmulator()
{
    clearDevice();
    if (m_HA_driver)
    {
        delete m_HA_driver;
        m_HA_driver = nullptr;
    }
}


//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatarEmulator::init()
{
    msg_info() << "HapticAvatarEmulator::init()";
    m_HA_driver = new HapticAvatarDriver(d_portName.getValue());

    if (!m_HA_driver->IsConnected())
        return;

    // get identity
    std::string identity = m_HA_driver->getIdentity();
    d_hapticIdentity.setValue(identity);
    std::cout << "HapticAvatarDeviceController identity: '" << identity << "'" << std::endl;

    // release force
    m_HA_driver->releaseForce();

    return;
}


void HapticAvatarEmulator::clearDevice()
{
    if (m_terminate == false)
    {
        m_terminate = true;
        haptic_thread.join();
        copy_thread.join();
    }
}


void HapticAvatarEmulator::bwdInit()
{   
    msg_info() << "HapticAvatarEmulator::bwdInit()";

    m_terminate = false;
    m_deviceReady = true;

    //haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    //copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);

}



using namespace sofa::helper::system::thread;

void HapticAvatarEmulator::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics thread" << std::endl;
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

    // Haptics Loop
    while (!terminate)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        ctime_t startTime = CTime::getRefTime();

        // Get all info from devices
        _deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();
        _deviceCtrl->m_hapticData.motorValues = _driver->getLastPWM();
        _deviceCtrl->m_hapticData.collisionForces = _driver->getLastCollisionForce();

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

        if (cptLoop % 100 == 0)
        {
            ctime_t stepTime = CTime::getRefTime();
            ctime_t diffLoop = stepTime - lastTime;
            lastTime = stepTime;
            //std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;
            _deviceCtrl->m_times.push_back(diffLoop* speedTimerMs);

            auto t2 = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::milliseconds(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
            t1 = t2;
            std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << " | " << duration.count() << std::endl;
        }
    }


    // ensure no force
    _driver->releaseForce();
    std::cout << "Haptics thread END!!" << std::endl;

    for (unsigned int i = 0; i < _deviceCtrl->m_times.size(); i++)
    {
        std::cout << _deviceCtrl->m_times[i] << std::endl;
    }
}


void HapticAvatarEmulator::CopyData(std::atomic<bool>& terminate, void * p_this)
{
    HapticAvatarEmulator* _deviceCtrl = static_cast<HapticAvatarEmulator*>(p_this);

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

void HapticAvatarEmulator::updatePosition()
{
    //_deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();

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

    // Haptics Loop
    for (int i=0; i<1000; i++)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        ctime_t startTime = CTime::getRefTime();

        // Get all info from devices
        m_hapticData.anglesAndLength = m_HA_driver->getAngles_AndLength();
        m_hapticData.motorValues = m_HA_driver->getLastPWM();
        m_hapticData.collisionForces = m_HA_driver->getLastCollisionForce();

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

        if (cptLoop % 100 == 0)
        {
            ctime_t stepTime = CTime::getRefTime();
            ctime_t diffLoop = stepTime - lastTime;
            lastTime = stepTime;
            //std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << std::endl;
            m_times.push_back(diffLoop* speedTimerMs);

            auto t2 = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::milliseconds(std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
            t1 = t2;
            std::cout << "loop nb: " << cptLoop << " -> " << diffLoop * speedTimerMs << " | " << duration.count() << std::endl;
        }
    }
}



void HapticAvatarEmulator::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    // vparams->drawTool()->disableLighting();

    //if (d_drawDeviceAxis.getValue())
    //{
    //    //    const HapticAvatarEmulator::Coord & posDevice = d_posDevice.getValue();
    //    //    float glRadius = float(d_scale.getValue());
    //    //    vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
    //    //    vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
    //    //    vparams->drawTool()->drawArrow(posDevice.getCenter(), posDevice.getCenter() + posDevice.getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));

    //    const HapticAvatarEmulator::VecCoord & testPosition = d_testPosition.getValue();
    //    float glRadius = float(d_scale.getValue());
    //    for (unsigned int i = 0; i < testPosition.size(); ++i)
    //    {
    //        vparams->drawTool()->drawArrow(testPosition[i].getCenter(), testPosition[i].getCenter() + testPosition[i].getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
    //        vparams->drawTool()->drawArrow(testPosition[i].getCenter(), testPosition[i].getCenter() + testPosition[i].getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
    //        vparams->drawTool()->drawArrow(testPosition[i].getCenter(), testPosition[i].getCenter() + testPosition[i].getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));
    //    }
    //}
        

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