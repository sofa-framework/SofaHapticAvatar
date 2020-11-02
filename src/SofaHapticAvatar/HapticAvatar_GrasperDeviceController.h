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

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>

#include <SofaHapticAvatar/HapticAvatar_Driver.h>
#include <SofaHapticAvatar/HapticAvatar_PortalManager.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

#include <SofaHaptics/ForceFeedback.h>
#include <SofaHaptics/LCPForceFeedback.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>

#include <atomic>

namespace sofa::HapticAvatar
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
class SOFA_HAPTICAVATAR_API HapticAvatar_GrasperDeviceController : public HapticAvatar_BaseDeviceController
{

public:
    SOFA_CLASS(HapticAvatar_GrasperDeviceController, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef RigidTypes::VecDeriv VecDeriv;
    typedef SolidTypes<double>::Transform Transform;
    typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3Types> LCPForceFeedback;
    typedef helper::vector<core::collision::DetectionOutput> ContactVector;

    HapticAvatar_GrasperDeviceController();
        
    void handleEvent(core::objectmodel::Event *) override;

    void retrieveCollisions();

    
    Data<float> d_jawTorq;

    Data<bool> d_newMethod;

    SReal m_distance;

    Vec3 m_toolDir;
    Vec3 m_pitchDir;
    Vec3 m_h;
    Vec3 m_hTM;

    sofa::helper::vector<float> m_times;

    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    static void CopyData(std::atomic<bool>& terminate, void * p_this);

    SingleLink<HapticAvatar_GrasperDeviceController, HapticAvatar_IBoxController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_iboxCtrl;

protected:
    void initImpl() override;

    void updatePositionImpl() override;

    bool createHapticThreads() override;

    void drawImpl(const sofa::core::visual::VisualParams*) override;

protected:
    HapticAvatar_IBoxController * m_iboxCtrl;

    HapticAvatarJaws m_jawsData;

    // Pointer to the scene detection Method component (Narrow phase only)
    sofa::core::collision::NarrowPhaseDetection* m_detectionNP;

    // Pointer to the scene intersection Method component
    sofa::core::collision::Intersection* m_intersectionMethod;

    std::vector<HapticContact> contactsSimu, contactsHaptic;
};

} // namespace sofa::HapticAvatar
