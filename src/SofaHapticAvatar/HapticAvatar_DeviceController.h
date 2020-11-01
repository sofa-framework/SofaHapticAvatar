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

#include <SofaHapticAvatar/HapticAvatar_Driver.h>
#include <SofaHapticAvatar/HapticAvatar_PortalManager.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

#include <SofaHaptics/ForceFeedback.h>
#include <SofaHaptics/LCPForceFeedback.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>

#include <atomic>

namespace sofa::component::controller
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;

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


struct SOFA_HAPTICAVATAR_API HapticContact
{
    Vector3 m_toolPosition;
    Vector3 m_objectPosition;
    Vector3 m_normal;
    Vector3 m_force;
    SReal distance;
    int tool; //0 = shaft, 1 = upjaw, 2 = downJaw
};


/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_DeviceController : public Controller
{

public:
    SOFA_CLASS(HapticAvatar_DeviceController, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef RigidTypes::VecDeriv VecDeriv;
    typedef SolidTypes<double>::Transform Transform;
    typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3Types> LCPForceFeedback;
    typedef helper::vector<core::collision::DetectionOutput> ContactVector;

    HapticAvatar_DeviceController();

	virtual ~HapticAvatar_DeviceController();

    virtual void init() override;
    virtual void bwdInit() override;
    virtual void reinit() override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    void updatePosition();
    void handleEvent(core::objectmodel::Event *) override;

    void updateAnglesAndLength(sofa::helper::fixed_array<float, 4> values);

    void retrieveCollisions();

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
    Data<bool> d_newMethod;

    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;
    Data<int> d_fontSize;

    Data<VecCoord> d_toolPosition;
    
    Data<SReal> m_forceScale;
    bool m_firstStep;
    SReal m_distance;


    Vec3 m_toolDir;
    Vec3 m_pitchDir;
    Vec3 m_h;
    Vec3 m_hTM;

    sofa::helper::vector<Vector3> m_debugForces;

    sofa::helper::vector<float> m_times;

    Coord m_debugRootPosition;
    sofa::defaulttype::Mat3x3f m_toolRot;
    sofa::defaulttype::Mat3x3f m_toolRotInv;
    sofa::defaulttype::Mat3x3f m_PortalRot;
    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    static void CopyData(std::atomic<bool>& terminate, void * p_this);

    std::atomic<bool> m_terminate;
    int m_portId;
    SingleLink<HapticAvatar_DeviceController, HapticAvatar_PortalManager, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_portalMgr;
    SingleLink<HapticAvatar_DeviceController, HapticAvatar_IBoxController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_iboxCtrl;
    LCPForceFeedback::SPtr m_forceFeedback;
    bool m_simulationStarted; ///< Boolean to warn scheduler when SOFA has started the simulation (changed by AnimateBeginEvent)
public:
    struct DeviceData
    {
        sofa::helper::fixed_array<float, 4> anglesAndLength;
        sofa::helper::fixed_array<float, 4> motorValues;
        sofa::helper::fixed_array<float, 3> collisionForces;
        VecDeriv hapticForces;
        float jawOpening;
    };

    DeviceData m_hapticData;
    DeviceData m_simuData;

protected:
    void clearDevice();

protected:
    HapticAvatar_Driver * m_HA_driver;
    HapticAvatar_PortalManager * m_portalMgr;
    HapticAvatar_IBoxController * m_iboxCtrl;
    bool m_deviceReady;

    sofa::simulation::CpuTask::Status m_simStepStatus;
    std::mutex lockPosition;

    std::thread haptic_thread;
    std::thread copy_thread;

    HapticAvatarJaws m_jawsData;

    // Pointer to the scene detection Method component (Narrow phase only)
    sofa::core::collision::NarrowPhaseDetection* m_detectionNP;

    // Pointer to the scene intersection Method component
    sofa::core::collision::Intersection* m_intersectionMethod;

    std::vector<HapticContact> contactsSimu, contactsHaptic;
};

} // namespace sofa::component::controller
