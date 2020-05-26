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

using namespace sofa::defaulttype;
using namespace sofa::simulation;


class HapticAvatarDeviceController;

class SOFA_HAPTICAVATAR_API HapticEmulatorTask : public CpuTask
{
public:
    HapticEmulatorTask(HapticAvatarDeviceController* ptr, CpuTask::Status* pStatus);

    virtual ~HapticEmulatorTask() {}

    virtual MemoryAlloc run() override final;

private:
    HapticAvatarDeviceController * m_controller;
};


// Set class to store Jaws Data information instead of struct so in the future could have a hiearchy of different tools.
class SOFA_HAPTICAVATAR_API HapticAvatarJaws
{
public:
    HapticAvatarJaws();


public:
    float m_MaxOpeningAngle;
    float m_jawLength;
    float m_jaw1Radius;
    float m_jaw2Radius;
    float m_shaftRadius;
};



/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatarDeviceController : public Controller
{

public:
    SOFA_CLASS(HapticAvatarDeviceController, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    HapticAvatarDeviceController();

	virtual ~HapticAvatarDeviceController();

    virtual void init() override;
    virtual void bwdInit() override;
    virtual void reinit() override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    void updatePosition();
    void handleEvent(core::objectmodel::Event *) override;

    void updateAnglesAndLength(sofa::helper::fixed_array<float, 4> values);

    //Data<Vec3d> d_positionBase; ///< Position of the interface base in the scene world coordinates
    //Data<Quat> d_orientationBase; ///< Orientation of the interface base in the scene world coordinates
    //Data<Quat> d_orientationTool; ///< Orientation of the tool    
    Data<SReal> d_scale; ///< Default scale applied to the Phantom Coordinates
    Data< Coord > d_posDevice; ///< position of the base of the part of the device    

    /// values returned by tool: Rot angle, Pitch angle, z Length, Yaw Angle
    Data<sofa::helper::fixed_array<float, 4> > d_toolValues;
    Data<sofa::helper::fixed_array<float, 4> > d_motorOutput;
    Data<sofa::helper::fixed_array<float, 3> > d_collisionForce;
    Data<float> d_jawTorq;
    Data<float> d_jawOpening;

    Data<bool> d_drawDeviceAxis;
    Data<bool> d_drawDebugForce;
    Data<bool> d_dumpThreadInfo;

    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;
    Data<int> d_fontSize;
    Data<Coord> d_jawUp;
    Data<Coord> d_jawDown;

    Data<VecCoord> d_testPosition;
    Data<SReal> m_forceScale;

    sofa::helper::vector<Vector3> m_debugForces;

    sofa::helper::vector<float> m_times;

    Coord m_debugRootPosition;
    sofa::defaulttype::Mat3x3f m_toolRot;
    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    static void CopyData(std::atomic<bool>& terminate, void * p_this);

    std::atomic<bool> m_terminate;
    int m_portId;
    SingleLink<HapticAvatarDeviceController, HapticAvatarPortalManager, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_portalMgr;
    SingleLink<HapticAvatarDeviceController, HapticAvatarIBoxController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_iboxCtrl;
    sofa::component::controller::ForceFeedback::SPtr m_forceFeedback;
    bool m_simulationStarted; ///< Boolean to warn scheduler when SOFA has started the simulation (changed by AnimateBeginEvent)
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

protected:
    void clearDevice();

protected:
    HapticAvatarDriver * m_HA_driver;
    HapticAvatarPortalManager * m_portalMgr;
    HapticAvatarIBoxController * m_iboxCtrl;
    bool m_deviceReady;

    sofa::simulation::TaskScheduler* m_taskScheduler;
    sofa::simulation::CpuTask::Status m_simStepStatus;
    std::mutex lockPosition;

    std::thread haptic_thread;
    std::thread copy_thread;

    HapticAvatarJaws m_jawsData;
};

} // namespace sofa::component::controller
