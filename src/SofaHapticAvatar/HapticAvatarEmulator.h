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
#include <SofaHapticAvatar/HapticAvatarDeviceController.h>

#include <atomic>

namespace sofa::component::controller
{
    using namespace sofa::defaulttype;

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatarEmulator : public HapticAvatarDeviceController
{

public:
    SOFA_CLASS(HapticAvatarEmulator, HapticAvatarDeviceController);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    HapticAvatarEmulator();
    virtual ~HapticAvatarEmulator() {}

    virtual void bwdInit() override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    void handleEvent(core::objectmodel::Event *) override;

    /// General Haptic thread methods
    static void HapticsEmulated(std::atomic<bool>& terminate, void * p_this, void * p_driver);
    
    Data<SReal> m_floorHeight;
    Data<SReal> m_damping;
    Data<int> m_testMode;

    Vector3 m_targetPosition;
    sofa::helper::fixed_array<float, 4> m_roughForce;
    SReal m_roughIntensity;

    bool m_activeTest;
};

} // namespace sofa::component::controller
