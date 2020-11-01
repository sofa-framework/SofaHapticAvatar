/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_DeviceEmulator.h>
#include <SofaHapticAvatar/HapticAvatar_Defines.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>

namespace sofa::component::controller
{

using namespace HapticAvatar;

int HapticAvatar_DeviceEmulatorClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_DeviceEmulator >()
    ;

using namespace sofa::defaulttype;

//constructeur
HapticAvatar_DeviceEmulator::HapticAvatar_DeviceEmulator()
    : HapticAvatar_DeviceController()
    , m_floorHeight(initData(&m_floorHeight, SReal(0.0), "floorHeight", "jaws opening angle"))
    , m_damping(initData(&m_damping, SReal(1.0), "damping", "jaws opening angle"))
    , m_testMode(initData(&m_testMode, 0, "testMode", "jaws opening angle"))
    , m_activeTest(false)
{
    this->f_listening.setValue(true);
    m_targetPosition = Vector3(0.0, 0.0, 0.0);
    m_roughForce = sofa::helper::fixed_array<float, 4>(0.0, 0.0, 0.0, 0.0);
    m_roughIntensity = 1.0;
}



void HapticAvatar_DeviceEmulator::bwdInit()
{   
    msg_info() << "HapticAvatar_DeviceEmulator::bwdInit()";

    m_portId = m_portalMgr->getPortalId(d_portName.getValue());
    if (m_portId == -1)
    {
        msg_error("HapticAvatar_DeviceController no portal id found");
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
    m_forceFeedback = context->get<LCPForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback != nullptr)
    {
        msg_info() << "ForceFeedback found";
    }

}



using namespace sofa::helper::system::thread;

void HapticAvatar_DeviceEmulator::HapticsEmulated(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics Emulator thread" << std::endl;
    HapticAvatar_DeviceEmulator* _deviceCtrl = static_cast<HapticAvatar_DeviceEmulator*>(p_this);
    HapticAvatar_Driver* _driver = static_cast<HapticAvatar_Driver*>(p_driver);
   
    if (_deviceCtrl == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_DeviceEmulator cast failed");
        return;
    }

    if (_driver == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_DeviceEmulator cast failed");
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
        
    bool contact = false;
    bool firstTimeTest = true;
    Vector3 _targetPosition;
    // Haptics Loop
    while (!terminate)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        ctime_t startTime = CTime::getRefTime();

        // Get all info from devices
        _deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();
        _deviceCtrl->m_hapticData.motorValues = _driver->getLastPWM();

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

        const HapticAvatar_DeviceController::VecCoord& testPosition = _deviceCtrl->d_toolPosition.getValue();
        // Check main force feedback
        Vector3 tipPosition = testPosition[3].getCenter();
        int testMode = _deviceCtrl->m_testMode.getValue();

        bool hasContact = false;
        if (_deviceCtrl->m_activeTest)
        {
            float damping = _deviceCtrl->m_damping.getValue();

            if (testMode == 1) // floor test
            {
                float height = _deviceCtrl->m_floorHeight.getValue();
                Vector3 floorPosition = tipPosition;
                if (tipPosition.y() < height)
                {
                    if (!contact)
                    {
                        std::cout << "First contact!" << std::endl;
                    }
                    floorPosition[1] = height;
                    Vector3 force = damping * (floorPosition - tipPosition);
                    _driver->setManualForceVector(_deviceCtrl->m_toolRotInv *force);
                    contact = true;
                    hasContact = true;
                }
            }
            else if (testMode == 2 || testMode == 3)
            {
                if (firstTimeTest)
                {
                    _targetPosition = tipPosition;
                    _deviceCtrl->m_targetPosition = _targetPosition;
                    firstTimeTest = false;
                }

                Vector3 force = damping * (_targetPosition - tipPosition);
                if (testMode == 2)
                    _driver->setManualForceVector(_deviceCtrl->m_toolRotInv *force, true);
                if (testMode == 3)
                    //_driver->setManualForceVector(_deviceCtrl->m_toolRotInv *force, false);
                    _driver->setTipForceVector(_deviceCtrl->m_toolRotInv *force);

                contact = true;
                hasContact = true;
            }
            else if (testMode == 4)
            {
                _driver->setManual_PWM(_deviceCtrl->m_roughForce[0], _deviceCtrl->m_roughForce[1], _deviceCtrl->m_roughForce[2], _deviceCtrl->m_roughForce[3]);
                contact = true;
                hasContact = true;
            }
        }


        if (!hasContact && contact) // was in contact
        {
            _driver->releaseForce();
            // Vector3 force = Vector3(0.0, 0.0, 0.0);
            // _driver->setForceVector(force);
            contact = false;
            firstTimeTest = true;
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
    std::cout << "Haptics Emulator thread END!!" << std::endl;
}


void HapticAvatar_DeviceEmulator::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    if (!m_activeTest)
        return;

    if (d_drawDeviceAxis.getValue())
    {
        const HapticAvatar_DeviceController::VecCoord & toolPosition = d_toolPosition.getValue();
        float glRadius = float(d_scale.getValue());

        for (unsigned int i = 0; i < toolPosition.size(); ++i)
        {
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, Vec4f(1, 0, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, Vec4f(0, 1, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, Vec4f(0, 0, 1, 1));
        }
    }

    int testMode = m_testMode.getValue();
    if (testMode == 1) // floor test
    {
        SReal floorH = m_floorHeight.getValue();
        SReal quadL = 100;
        
        vparams->drawTool()->drawQuad(defaulttype::Vector3(-quadL, floorH, -quadL),
            defaulttype::Vector3(-quadL, floorH, quadL),
            defaulttype::Vector3(quadL, floorH, quadL),
            defaulttype::Vector3(quadL, floorH, -quadL),
            defaulttype::Vector3(0.0, 1.0, 0.0), sofa::helper::types::RGBAColor::green());
    }
    else if (testMode == 2 || testMode == 3)
    {
        vparams->drawTool()->drawSphere(m_targetPosition, 3, sofa::helper::types::RGBAColor::green());
        const HapticAvatar_DeviceController::VecCoord& testPosition = d_toolPosition.getValue();
        // Check main force feedback
        Vector3 tipPosition = testPosition[3].getCenter();

        vparams->drawTool()->drawSphere(tipPosition, 3, sofa::helper::types::RGBAColor::red());
        vparams->drawTool()->drawLine(m_targetPosition, tipPosition, sofa::helper::types::RGBAColor::green());
    }
}


void HapticAvatar_DeviceEmulator::handleEvent(core::objectmodel::Event *event)
{
    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
       if (!m_deviceReady)
            return;

        m_simulationStarted = true;
        updatePosition();
    }
    else if (sofa::core::objectmodel::KeypressedEvent::checkEventType(event))
    {
        sofa::core::objectmodel::KeypressedEvent* ke = static_cast<sofa::core::objectmodel::KeypressedEvent*>(event);
        if (ke->getKey() == '0')
            m_activeTest = !m_activeTest;



        // rotTorque    
        if (ke->getKey() == '4')
        {
            m_roughForce[0] += m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        else if (ke->getKey() == '6')
        {
            m_roughForce[0] -= m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        // pitch torque
        else if (ke->getKey() == '7')
        {
            m_roughForce[1] += m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        else if (ke->getKey() == '9')
        {
            m_roughForce[1] -= m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        // zforce
        else if (ke->getKey() == '8')
        {
            m_roughForce[2] += m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        else if (ke->getKey() == '2')
        {
            m_roughForce[2] -= m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        // yaw torque
        else if (ke->getKey() == '1')
        {
            m_roughForce[3] += m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        else if (ke->getKey() == '3')
        {
            m_roughForce[3] -= m_roughIntensity;
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        else if (ke->getKey() == '5')
        {
            m_roughForce = sofa::helper::fixed_array<float, 4>(0.0, 0.0, 0.0, 0.0);
            std::cout << "m_roughForce: " << m_roughForce << std::endl;
        }
        // force value
        else if (ke->getKey() == '+')
        {
            m_roughIntensity += 0.1;
            std::cout << "m_roughIntensity: " << m_roughIntensity << std::endl;
        }
        else if (ke->getKey() == '-')
        {
            m_roughIntensity -= 0.1;
            std::cout << "m_roughIntensity: " << m_roughIntensity << std::endl;
        }

    }
}

} // namespace sofa::component::controller