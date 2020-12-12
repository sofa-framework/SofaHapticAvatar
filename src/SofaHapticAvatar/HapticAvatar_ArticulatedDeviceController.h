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
    typedef Vec1Types::Coord Coord;
    typedef Vec1Types::VecCoord VecCoord;
    typedef Vec1Types::VecDeriv VecDeriv;
    typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Vec1dTypes> LCPForceFeedback;


    /// Default constructor
    HapticAvatar_ArticulatedDeviceController();

    ///// handleEvent component method to catch collision info
    //void handleEvent(core::objectmodel::Event *) override;
    

public:
    /// output data position of the tool
    Data<VecCoord> d_toolPosition;

    /// Pointer to the ForceFeedback component
    LCPForceFeedback::SPtr m_forceFeedback;


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
