/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/config.h>
#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>

namespace sofa::HapticAvatar
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;
using namespace sofa::component::controller;

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
class SOFA_HAPTICAVATAR_API HapticAvatar_TipDeviceController : public HapticAvatar_BaseDeviceController
{

public:
    SOFA_CLASS(HapticAvatar_TipDeviceController, HapticAvatar_BaseDeviceController);

    HapticAvatar_TipDeviceController();

    //virtual ~HapticAvatar_TipDeviceController()

    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    static void CopyData(std::atomic<bool>& terminate, void * p_this);

protected:
    void createHapticThreads() override;
};

} // namespace sofa::HapticAvatar
