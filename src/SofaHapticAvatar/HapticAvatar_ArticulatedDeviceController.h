/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>

namespace sofa::HapticAvatar
{

using namespace sofa::defaulttype;

class HapticAvatar_IBoxController;

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_ArticulatedDeviceController : public HapticAvatar_BaseDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_ArticulatedDeviceController, HapticAvatar_BaseDeviceController);

    using Coord = Vec1Types::Coord;
    using VecCoord = Vec1Types::VecCoord;
    using VecDeriv = Vec1Types::VecDeriv;
    using LCPForceFeedback = sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Vec1dTypes>;
    using ArticulationSize = unsigned int;

    /// Default constructor
    HapticAvatar_ArticulatedDeviceController();

    virtual void haptic_updateArticulations(HapticAvatar_IBoxController* _iBox) { SOFA_UNUSED(_iBox); }

    virtual void haptic_updateForceFeedback(HapticAvatar_IBoxController* _iBox) { SOFA_UNUSED(_iBox); }

public:
    /// output data position of the tool
    Data<VecCoord> d_toolPosition;

    /// Pointer to the ForceFeedback component
    LCPForceFeedback::SPtr m_forceFeedback;

    // link to the forceFeedBack component, if not set will search through graph and take first one encountered
    SingleLink<HapticAvatar_ArticulatedDeviceController, LCPForceFeedback, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_forceFeedback;

protected:
    VecCoord m_toolPositionCopy;
    VecDeriv m_resForces;
    ArticulationSize m_nbArticulations;
};

} // namespace sofa::HapticAvatar
