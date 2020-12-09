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
class SOFA_HAPTICAVATAR_API HapticAvatar_ArticulatedDeviceEmulator : public HapticAvatar_BaseDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_ArticulatedDeviceEmulator, HapticAvatar_BaseDeviceController);

    typedef Vec1Types::Coord Articulation;
    typedef Vec1Types::VecCoord VecArticulation;

    /// Default constructor
    HapticAvatar_ArticulatedDeviceEmulator();


    /// Method to handle various event like keyboard or omni.
    void handleEvent(sofa::core::objectmodel::Event* event) override;

    void moveRotationAxe1(Articulation value);
    void moveRotationAxe2(Articulation value);
    void moveRotationAxe3(Articulation value);
    void moveTranslationAxe1(Articulation value);
    void openJaws(Articulation value);

    Data<VecArticulation> d_articulations;

protected:
    /// override method to create the different threads
    bool createHapticThreads() override;

    /// override method to update specific tool position
    void updatePositionImpl() override;
};

}
