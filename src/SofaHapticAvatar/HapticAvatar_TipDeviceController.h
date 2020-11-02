/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>
#include <atomic>

namespace sofa::HapticAvatar
{

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
    bool createHapticThreads() override;

    void updatePositionImpl() override;
};

} // namespace sofa::HapticAvatar
