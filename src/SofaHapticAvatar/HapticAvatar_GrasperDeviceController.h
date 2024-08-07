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

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_GrasperDeviceController : public HapticAvatar_ArticulatedDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_GrasperDeviceController, HapticAvatar_ArticulatedDeviceController);

    /// Default constructor
    HapticAvatar_GrasperDeviceController();

    virtual ~HapticAvatar_GrasperDeviceController() {}
  
    /// Thread methods to cpy data from m_hapticData to m_simuData
    void CopyData(std::atomic<bool>& terminate, void * p_this);


    void haptic_updateArticulations(HapticAvatar_IBoxController* _IBoxCtrl) override;

    void haptic_updateForceFeedback(HapticAvatar_IBoxController* _IBoxCtrl) override;


protected:

    /// override method to create the different threads
    bool createHapticThreads() override;

    /// override method to update specific tool position
    void updatePositionImpl() override;

public:
    Data<bool> d_useIBox;
    /// Max opening angle of the Jaws
    Data<float> d_MaxOpeningAngle;
};

} // namespace sofa::HapticAvatar
