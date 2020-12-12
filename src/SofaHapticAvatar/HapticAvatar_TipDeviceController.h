/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_RigidDeviceController.h>
#include <atomic>

namespace sofa::HapticAvatar
{

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_TipDeviceController : public HapticAvatar_RigidDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_TipDeviceController, HapticAvatar_RigidDeviceController);

    /// Default constructor
    HapticAvatar_TipDeviceController();

    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    /// Thread methods to cpy data from m_hapticData to m_simuData
    static void CopyData(std::atomic<bool>& terminate, void * p_this);

protected:
    /// override method to create the different threads
    bool createHapticThreads() override;

    /// override method to update specific tool position
    void updatePositionImpl() override;
};

}
