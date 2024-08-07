/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_GrasperDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_HapticThreadManager.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/core/visual/VisualParams.h>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <fstream>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

const int HapticAvatar_GrasperDeviceControllerClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_GrasperDeviceController >()
    ;


//constructeur
HapticAvatar_GrasperDeviceController::HapticAvatar_GrasperDeviceController()
    : HapticAvatar_ArticulatedDeviceController()
    , d_useIBox(initData(&d_useIBox, bool(true), "useIBox", "Set to true if this device is linked to an ibox"))
    , d_MaxOpeningAngle(initData(&d_MaxOpeningAngle, 60.0f, "MaxOpeningAngle", "Max jaws opening angle"))
{
    m_toolRot.identity();

    m_nbArticulations = 6;
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations.resize(m_nbArticulations);
    for (unsigned int i = 0; i < m_nbArticulations; ++i)
        articulations[i] = 0;

    m_toolPositionCopy = articulations;
    m_resForces.resize(m_nbArticulations);
}


bool HapticAvatar_GrasperDeviceController::createHapticThreads()
{   
    msg_info() << "HapticAvatar_GrasperDeviceController::createHapticThreads()";

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

    m_HA_driver->setMotorForceAndTorques(-float(m_resForces[2][0]), float(m_resForces[1][0]), float(m_resForces[3][0]), -float(m_resForces[0][0]));
    if (d_useIBox.getValue() && _IBoxCtrl != nullptr)
    {
        float jaw_momentum_arm = 25.0f * sin(0.38f + m_hapticData.jawOpening);
        float handleForce = float(m_resForces[4][0] - m_resForces[5][0]) / jaw_momentum_arm; // in Newtons

        _IBoxCtrl->setHandleForce(m_hapticData.toolId, handleForce * 3);
    }
}

} // namespace sofa::HapticAvatar
