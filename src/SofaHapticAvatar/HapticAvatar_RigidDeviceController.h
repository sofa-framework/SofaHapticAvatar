/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

namespace sofa::HapticAvatar
{

// Set class to store Jaws Data information instead of struct so in the future could have a hiearchy of different tools.
class SOFA_HAPTICAVATAR_API HapticRigidAvatarJaws
{
public:
    HapticRigidAvatarJaws();

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
class SOFA_HAPTICAVATAR_API HapticAvatar_RigidDeviceController : public HapticAvatar_BaseDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_RigidDeviceController, HapticAvatar_BaseDeviceController);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef RigidTypes::VecDeriv VecDeriv;
    typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3Types> LCPForceFeedback;

    typedef SolidTypes<double>::Transform Transform;

    /// default constructor
    HapticAvatar_RigidDeviceController();

    /// default destructor
	virtual ~HapticAvatar_RigidDeviceController();

protected:
   
    /// Internal method to draw specific informations
    void drawImpl(const sofa::core::visual::VisualParams* vparams) override;

public:
    /// link to the IBox controller component 
    SingleLink<HapticAvatar_RigidDeviceController, HapticAvatar_IBoxController, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_iboxCtrl;

    /// output data position of the tool
    Data<VecCoord> d_toolPosition;

    /// Pointer to the ForceFeedback component
    LCPForceFeedback::SPtr m_forceFeedback;

protected:
    /// Pointer to the IBoxController component
    HapticAvatar_IBoxController * m_iboxCtrl;

    /// Jaws specific informations
    HapticRigidAvatarJaws m_jawsData;

};

} // namespace sofa::HapticAvatar
