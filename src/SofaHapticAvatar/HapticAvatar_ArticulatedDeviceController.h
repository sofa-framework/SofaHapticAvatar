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

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_ArticulatedDeviceController : public HapticAvatar_BaseDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_ArticulatedDeviceController, HapticAvatar_BaseDeviceController);

    typedef Vec1Types::Coord Articulation;
    typedef Vec1Types::VecCoord VecArticulation;
    typedef Vec1Types::VecDeriv VecArticDeriv;
    typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Vec1dTypes> LCPForceFeedback1D;


    /// Default constructor
    HapticAvatar_ArticulatedDeviceController();

    ///// handleEvent component method to catch collision info
    //void handleEvent(core::objectmodel::Event *) override;
    

public:
    Data<VecArticulation> d_articulations;

    /// Pointer to the ForceFeedback component
    LCPForceFeedback1D::SPtr m_forceFeedback1D;


protected:
    /// Internal method to init specific collision components
    void initImpl() override;

    ///// override method to create the different threads
    //bool createHapticThreads() override;

    ///// override method to update specific tool position
    //void updatePositionImpl() override;

    ///// Internal method to draw specific informations
    //void drawImpl(const sofa::core::visual::VisualParams*) override {}
};

} // namespace sofa::HapticAvatar
