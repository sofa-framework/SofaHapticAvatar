/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>
#include <sofa/core/collision/NarrowPhaseDetection.h>

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
    typedef helper::vector<core::collision::DetectionOutput> ContactVector;

    /// Default constructor
    HapticAvatar_GrasperDeviceController();

    /// handleEvent component method to catch collision info
    void handleEvent(core::objectmodel::Event *) override;
    
    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    /// Thread methods to cpy data from m_hapticData to m_simuData
    static void CopyData(std::atomic<bool>& terminate, void * p_this);

protected:
    /// Internal method to init specific collision components
    void initImpl() override;

    /// override method to create the different threads
    bool createHapticThreads() override;

    /// override method to update specific tool position
    void updatePositionImpl() override;

    /// Internal method to retrieve the collision information per simulation step
    void retrieveCollisions();

    /// Internal method to draw specific informations
    void drawImpl(const sofa::core::visual::VisualParams*) override {}

public:
    /// Parameter to choose old/new method
    Data<bool> d_newMethod;

    /// link to the IBox controller component 
    SingleLink<HapticAvatar_GrasperDeviceController, HapticAvatar_IBoxController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_iboxCtrl;

    /// collision distance value, need public to be accessed by haptic thread
    SReal m_distance;

    /// list of contact detected by the collision
    std::vector<HapticContact> contactsSimu, contactsHaptic;

protected:
    /// Pointer to the IBoxController component
    HapticAvatar_IBoxController * m_iboxCtrl;

    /// Jaws specific informations
    HapticAvatarJaws m_jawsData;

    // Pointer to the scene detection Method component (Narrow phase only)
    sofa::core::collision::NarrowPhaseDetection* m_detectionNP;

    // Pointer to the scene intersection Method component
    sofa::core::collision::Intersection* m_intersectionMethod;
};

} // namespace sofa::HapticAvatar
