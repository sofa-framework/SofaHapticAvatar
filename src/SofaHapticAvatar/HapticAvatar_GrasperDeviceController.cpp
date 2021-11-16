/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_GrasperDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/Node.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

int HapticAvatar_GrasperDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_GrasperDeviceController >()
    ;


//constructeur
HapticAvatar_GrasperDeviceController::HapticAvatar_GrasperDeviceController()
    : HapticAvatar_ArticulatedDeviceController()
    , d_useIBox(initData(&d_useIBox, bool(true), "useIBox", "Set to true if this device is linked to an ibox"))
    , d_MaxOpeningAngle(initData(&d_MaxOpeningAngle, SReal(60.0f), "MaxOpeningAngle", "Max jaws opening angle"))
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

    m_nbArticulations = 6;
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations.resize(m_nbArticulations);
    for (auto i = 0; i < m_nbArticulations; ++i)
        articulations[i] = 0;

    m_toolPositionCopy = articulations;
    m_resForces.resize(m_nbArticulations);
}



//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_GrasperDeviceController::initImpl()
{
    // Retrieve ForceFeedback component pointer
    if (l_forceFeedback.empty())
    {
        simulation::Node* context = dynamic_cast<simulation::Node*>(this->getContext()); // access to current node
        m_forceFeedback = context->get<LCPForceFeedback>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);
    }
    else
    {
        m_forceFeedback = l_forceFeedback.get();
    }

    m_HA_driver->setDeadBandPWMWidth(100, 0, 0, 0);
    if (m_forceFeedback == nullptr)
    {
        msg_warning() << "ForceFeedback not found";
    }
}


bool HapticAvatar_GrasperDeviceController::createHapticThreads()
{   
    std::cout << "HapticAvatar_GrasperDeviceController::createHapticThreads()" << std::endl;

    auto threadMgr = HapticAvatar_HapticThreadManager::getInstance();
    threadMgr->registerDevice(this);

    m_terminate = false;
    copy_thread = std::thread(&HapticAvatar_GrasperDeviceController::CopyData, this, std::ref(this->m_terminate), this);
    return true;
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

    // get info from simuData
    sofa::type::fixed_array<float, 4> dofV = m_simuData.anglesAndLength;

    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    //std::cout << "YAW: " << dofV[Dof::YAW] << " | PITCH: " << dofV[Dof::PITCH] << " | ROT: " << dofV[Dof::ROT] << " | Z: " << dofV[Dof::Z] << std::endl;
    articulations[0] = dofV[Dof::YAW];
    articulations[1] = -dofV[Dof::PITCH];
    articulations[2] = dofV[Dof::ROT];
    articulations[3] = dofV[Dof::Z];

    float _OpeningAngle = m_simuData.jawOpening * d_MaxOpeningAngle.getValue() * 0.01f;
    articulations[4] = _OpeningAngle;
    articulations[5] = -_OpeningAngle;
    
    // copy into an non Data variable for haptic thread acces.
    m_toolPositionCopy = articulations;
}


void HapticAvatar_GrasperDeviceController::haptic_updateArticulations(HapticAvatar_IBoxController* _IBoxCtrl)
{
    // Update info from device hardware
    // update angles and length
    m_hapticData.anglesAndLength = m_HA_driver->getAnglesAndLength();

    // Get tool Id // TODO check if this is needed at each haptic thread?
    m_hapticData.toolId = m_HA_driver->getToolID();

    if (d_useIBox.getValue() && _IBoxCtrl != nullptr)
    {
        m_hapticData.jawOpening = _IBoxCtrl->getJawOpeningAngle(m_hapticData.toolId);
    }
}


void HapticAvatar_GrasperDeviceController::haptic_updateForceFeedback(HapticAvatar_IBoxController* _IBoxCtrl)
{
    if (m_forceFeedback == nullptr)
        return;

    m_forceFeedback->computeForce(m_toolPositionCopy, m_resForces);

    m_HA_driver->setMotorForceAndTorques(-m_resForces[2][0], m_resForces[1][0], m_resForces[3][0], -m_resForces[0][0]);
    if (d_useIBox.getValue() && _IBoxCtrl != nullptr)
    {
        float jaw_momentum_arm = 25.0f * sin(0.38f + m_hapticData.jawOpening);
        float handleForce = (m_resForces[4][0] - m_resForces[5][0]) / jaw_momentum_arm; // in Newtons

        _IBoxCtrl->setHandleForce(m_hapticData.toolId, handleForce * 3);
    }
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
