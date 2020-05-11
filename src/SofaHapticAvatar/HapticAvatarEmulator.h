/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/config.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/Quater.h>

#include <SofaUserInteraction/Controller.h>

#include <SofaHapticAvatar/HapticAvatarDriver.h>
#include <SofaHapticAvatar/HapticAvatarPortalManager.h>
#include <SofaHapticAvatar/HapticAvatarIBoxController.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

#include <SofaHaptics/ForceFeedback.h>

#include <atomic>

namespace sofa::component::controller
{


/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatarEmulator : public Controller
{

public:
    SOFA_CLASS(HapticAvatarEmulator, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    HapticAvatarEmulator();
	virtual ~HapticAvatarEmulator();

    virtual void init() override;
    virtual void bwdInit() override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    void updatePosition();
    void handleEvent(core::objectmodel::Event *) override;


    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;
    sofa::helper::vector<float> m_times;

    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);
    static void CopyData(std::atomic<bool>& terminate, void * p_this);

    std::atomic<bool> m_terminate;
    int m_portId;
    SingleLink<HapticAvatarEmulator, HapticAvatarPortalManager, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_portalMgr;
    SingleLink<HapticAvatarEmulator, HapticAvatarIBoxController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_iboxCtrl;
    sofa::component::controller::ForceFeedback::SPtr m_forceFeedback;

public:
    struct DeviceData
    {
        sofa::helper::fixed_array<float, 4> anglesAndLength;
        sofa::helper::fixed_array<float, 4> motorValues;
        sofa::helper::fixed_array<float, 3> collisionForces;
        float jawOpening;
    };

    DeviceData m_hapticData;
    DeviceData m_simuData;

private:
    void clearDevice();

private:
    HapticAvatarDriver * m_HA_driver;
    HapticAvatarPortalManager * m_portalMgr;
    HapticAvatarIBoxController * m_iboxCtrl;
    bool m_deviceReady;

    sofa::simulation::TaskScheduler* m_taskScheduler;
    sofa::simulation::CpuTask::Status m_simStepStatus;
    std::mutex lockPosition;

    std::thread haptic_thread;
    std::thread copy_thread;

};

} // namespace sofa::component::controller
