/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceEmulator.h>


#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>


#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <chrono>

namespace sofa::HapticAvatar
{

using namespace HapticAvatar;
using namespace sofa::helper::system::thread;

int HapticAvatar_ArticulatedDeviceEmulatorClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_ArticulatedDeviceEmulator >()
    ;


//constructeur
HapticAvatar_ArticulatedDeviceEmulator::HapticAvatar_ArticulatedDeviceEmulator()
    : HapticAvatar_BaseDeviceController()
    , l_iboxCtrl(initLink("iboxController", "link to IBoxController"))
    , d_articulations(initData(&d_articulations, "articulations", "Output data position of the tool"))
    , m_iboxCtrl(nullptr)
    , m_forceFeedback1D(nullptr)
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations.resize(6);
    articulations[0] = 0;
    articulations[1] = 0;
    articulations[2] = 0;
    articulations[3] = 0;

    articulations[4] = 0;
    articulations[5] = 0;
    d_articulations.endEdit();

    //articulations[0] = dofV[Dof::YAW];
    //articulations[1] = -dofV[Dof::PITCH];
    //articulations[2] = dofV[Dof::ROT];
    //articulations[3] = dofV[Dof::Z];

    //articulations[4] = morsUP;
    //articulations[5] = morsDown;

}


void HapticAvatar_ArticulatedDeviceEmulator::initImpl()
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
    m_forceFeedback1D = context->get<LCPForceFeedback1D>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback1D == nullptr)
    {
        msg_warning() << "ForceFeedback not found";
    }
    else
        std::cout << "OOOOOOOOOOKKKKKKK" << std::endl;
    
}

bool HapticAvatar_ArticulatedDeviceEmulator::createHapticThreads()
{
    m_terminate = false;
    haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);
    m_hapticData.hapticForces.resize(5);
    m_simuData.hapticForces.resize(5);

    return true;
}



void HapticAvatar_ArticulatedDeviceEmulator::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{
    std::cout << "Haptics thread" << std::endl;
    HapticAvatar_ArticulatedDeviceEmulator* _deviceCtrl = static_cast<HapticAvatar_ArticulatedDeviceEmulator*>(p_this);
    HapticAvatar_Driver* _driver = static_cast<HapticAvatar_Driver*>(p_driver);

    if (_deviceCtrl == nullptr)
    {
        msg_error("Haptics Thread: HapticAvatar_GrasperDeviceController cast failed");
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

    VecArtiDeriv resForces;
    resForces.resize(6);
    int cptLoop = 0;
    while (!terminate)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        ctime_t startTime = CTime::getRefTime();

        // Get all info from devices
        _deviceCtrl->m_hapticData.anglesAndLength = _driver->getAngles_AndLength();
        _deviceCtrl->m_hapticData.motorValues = _driver->getLastPWM();

        // get info regarding jaws
        //float jtorq = _driver->getJawTorque();
        if (_deviceCtrl->m_iboxCtrl)
        {
            float angle = _deviceCtrl->m_iboxCtrl->getJawOpeningAngle();
            _deviceCtrl->m_hapticData.jawOpening = angle;
        }

        // Force feedback computation
        if (_deviceCtrl->m_simulationStarted && _deviceCtrl->m_forceFeedback1D)
        {
            const HapticAvatar_ArticulatedDeviceEmulator::VecArticulation& articulations = _deviceCtrl->d_articulations.getValue();
            _deviceCtrl->m_forceFeedback1D->computeForce(articulations, resForces);

            // resForces:             
            //articulations[0] = dofV[Dof::YAW];
            //articulations[1] = -dofV[Dof::PITCH];
            //articulations[2] = dofV[Dof::ROT];
            //articulations[3] = dofV[Dof::Z];
            _driver->setManual_PWM(float(-resForces[2][0]), float(-resForces[1][0]), float(resForces[3][0]), float(-resForces[0][0]));

            if (cptLoop == 50)
            {
                std::cout << "resForces: " << resForces << std::endl;
                cptLoop = 0;
            }

            cptLoop++;
        }
        else
            _driver->releaseForce();

        ctime_t endTime = CTime::getRefTime();
        ctime_t duration = endTime - startTime;

        // If loop is quicker than the target loop speed. Wait here.
        while (duration < targetTicksPerLoop)
        {
            endTime = CTime::getRefTime();
            duration = endTime - startTime;
        }

        
    }
}


void HapticAvatar_ArticulatedDeviceEmulator::CopyData(std::atomic<bool>& terminate, void * p_this)
{
    HapticAvatar_ArticulatedDeviceEmulator* _deviceCtrl = static_cast<HapticAvatar_ArticulatedDeviceEmulator*>(p_this);

    // Use computer tick for timer
    ctime_t targetSpeedLoop = 1 / 2; // Target loop speed: 0.5ms
    ctime_t refTicksPerMs = CTime::getRefTicksPerSec() / 1000;
    ctime_t targetTicksPerLoop = targetSpeedLoop * refTicksPerMs;
    double speedTimerMs = 1000 / double(CTime::getRefTicksPerSec());

    ctime_t lastTime = CTime::getRefTime();
    std::cout << "CopyData refTicksPerMs: " << refTicksPerMs << " targetTicksPerLoop: " << targetTicksPerLoop << std::endl;
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
    }
}



void HapticAvatar_ArticulatedDeviceEmulator::updatePositionImpl()
{
    //std::cout << "updatePositionImpl" << std::endl;

    // get info from simuData
    sofa::helper::fixed_array<float, 4> dofV = m_simuData.anglesAndLength;

    VecArticulation & articulations = *d_articulations.beginEdit();
    //std::cout << "YAW: " << dofV[Dof::YAW] << " | PITCH: " << dofV[Dof::PITCH] << " | ROT: " << dofV[Dof::ROT] << " | Z: " << dofV[Dof::Z] << std::endl;
    articulations[0] = dofV[Dof::YAW];
    articulations[1] = -dofV[Dof::PITCH];
    articulations[2] = dofV[Dof::ROT];
    articulations[3] = dofV[Dof::Z];

    float _OpeningAngle = m_simuData.jawOpening * 60.0f * 0.01f;
    articulations[4] = _OpeningAngle;
    articulations[5] = -_OpeningAngle;

    d_articulations.endEdit();
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe1(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[0] += value;
    d_articulations.endEdit();
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe2(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[1] += value;
    d_articulations.endEdit();
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe3(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[2] += value;
    d_articulations.endEdit();
}

void HapticAvatar_ArticulatedDeviceEmulator::moveTranslationAxe1(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[3] += value;
    d_articulations.endEdit();
}


void HapticAvatar_ArticulatedDeviceEmulator::openJaws(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[4] += value;
    articulations[5] -= value;
    d_articulations.endEdit();
}


void HapticAvatar_ArticulatedDeviceEmulator::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (sofa::core::objectmodel::KeypressedEvent* ke = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        if (ke->getKey() == '1')
            moveRotationAxe1(Articulation(0.01));
        else if (ke->getKey() == '3')
            moveRotationAxe1(Articulation(-0.01));
        else if (ke->getKey() == '4')
            moveRotationAxe2(Articulation(0.01));
        else if (ke->getKey() == '6')
            moveRotationAxe2(Articulation(-0.01));
        else if (ke->getKey() == '7')
            moveRotationAxe3(Articulation(0.01));
        else if (ke->getKey() == '9')
            moveRotationAxe3(Articulation(-0.01));
        else if (ke->getKey() == '8')
            moveTranslationAxe1(Articulation(1.0));
        else if (ke->getKey() == '2')
            moveTranslationAxe1(Articulation(-1.0));
        else if (ke->getKey() == '+')
            openJaws(Articulation(0.1));
        else if (ke->getKey() == '-')
            openJaws(Articulation(-0.1));
    }


    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        m_simulationStarted = true;
        updatePositionImpl();
    }
    
}


} // namespace sofa::HapticAvatar