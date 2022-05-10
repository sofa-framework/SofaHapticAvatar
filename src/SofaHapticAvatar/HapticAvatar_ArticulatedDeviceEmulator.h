/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

#include <atomic>

namespace sofa::HapticAvatar
{

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_ArticulatedDeviceEmulator : public HapticAvatar_ArticulatedDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_ArticulatedDeviceEmulator, HapticAvatar_ArticulatedDeviceController);

    /// Default constructor
    HapticAvatar_ArticulatedDeviceEmulator();

    /// Method to handle various event like keyboard or omni.
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    void moveRotationAxe1(Coord value);
    void moveRotationAxe2(Coord value);
    void moveRotationAxe3(Coord value);
    void moveTranslationAxe1(Coord value);
    void openJaws(Coord value);

protected:    
    /// Internal method to init specific collision components
    void initDevice() override;

    /// override method to create the different threads
    bool createHapticThreads() override;

    /// override method to update specific tool position
    void updatePositionImpl() override;

};

}
