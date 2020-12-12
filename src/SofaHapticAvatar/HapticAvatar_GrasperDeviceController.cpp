/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_GrasperDeviceController.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

int HapticAvatar_GrasperDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_GrasperDeviceController >()
    ;


//constructeur
HapticAvatar_GrasperDeviceController::HapticAvatar_GrasperDeviceController()
    : HapticAvatar_ArticulatedDeviceController()
    , d_MaxOpeningAngle(initData(&d_MaxOpeningAngle, SReal(60.0f), "MaxOpeningAngle", "Max jaws opening angle"))
    , l_iboxCtrl(initLink("iboxController", "link to IBoxController"))
    , m_iboxCtrl(nullptr)
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
}



//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_GrasperDeviceController::initImpl()
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


}


bool HapticAvatar_GrasperDeviceController::createHapticThreads()
{   
    m_terminate = false;
    haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);

    return true;
}


void HapticAvatar_GrasperDeviceController::Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver)
{ 
    std::cout << "Haptics thread" << std::endl;

    HapticAvatar_GrasperDeviceController* _deviceCtrl = static_cast<HapticAvatar_GrasperDeviceController*>(p_this);
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

    VecArticDeriv resForces;
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
            const HapticAvatar_GrasperDeviceController::VecArticulation& articulations = _deviceCtrl->d_articulations.getValue();
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

    // ensure no force
    _driver->releaseForce();
    std::cout << "Haptics thread END!!" << std::endl;
}


void HapticAvatar_GrasperDeviceController::CopyData(std::atomic<bool>& terminate, void * p_this)
{
    HapticAvatar_GrasperDeviceController* _deviceCtrl = static_cast<HapticAvatar_GrasperDeviceController*>(p_this);
    
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
    }
}

void HapticAvatar_GrasperDeviceController::updatePositionImpl()
{
    if (!m_HA_driver)
        return;

    //std::cout << "updatePositionImpl" << std::endl;

    // get info from simuData
    sofa::helper::fixed_array<float, 4> dofV = m_simuData.anglesAndLength;

    VecArticulation & articulations = *d_articulations.beginEdit();
    //std::cout << "YAW: " << dofV[Dof::YAW] << " | PITCH: " << dofV[Dof::PITCH] << " | ROT: " << dofV[Dof::ROT] << " | Z: " << dofV[Dof::Z] << std::endl;
    articulations[0] = dofV[Dof::YAW];
    articulations[1] = -dofV[Dof::PITCH];
    articulations[2] = dofV[Dof::ROT];
    articulations[3] = dofV[Dof::Z];

    float _OpeningAngle = m_simuData.jawOpening * d_MaxOpeningAngle.getValue() * 0.01f;
    articulations[4] = _OpeningAngle;
    articulations[5] = -_OpeningAngle;

    d_articulations.endEdit();
}


void HapticAvatar_GrasperDeviceController::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        m_simulationStarted = true;
        updatePositionImpl();
    }
}

} // namespace sofa::HapticAvatar
