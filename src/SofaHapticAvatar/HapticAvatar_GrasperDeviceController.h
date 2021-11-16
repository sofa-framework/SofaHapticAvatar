/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

namespace sofa::HapticAvatar
{

using namespace sofa::defaulttype;

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_GrasperDeviceController : public HapticAvatar_ArticulatedDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_GrasperDeviceController, HapticAvatar_ArticulatedDeviceController);

    /// Default constructor
    HapticAvatar_GrasperDeviceController();

    /// handleEvent component method to catch collision info
    void handleEvent(core::objectmodel::Event *) override;
    
    /// Thread methods to cpy data from m_hapticData to m_simuData
    void CopyData(std::atomic<bool>& terminate, void * p_this);


    void haptic_updateArticulations(HapticAvatar_IBoxController* _IBoxCtrl) override;

    void haptic_updateForceFeedback(HapticAvatar_IBoxController* _IBoxCtrl) override;


protected:
    /// Internal method to init specific collision components
    void initImpl() override;

    /// override method to create the different threads
    bool createHapticThreads() override;

    /// override method to update specific tool position
    void updatePositionImpl() override;

    /// Internal method to draw specific informations
    void drawImpl(const sofa::core::visual::VisualParams*) override {}

public:
    Data<bool> d_useIBox;
    /// Max opening angle of the Jaws
    Data<SReal> d_MaxOpeningAngle;
};

} // namespace sofa::HapticAvatar
