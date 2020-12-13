/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_RigidDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

namespace sofa::HapticAvatar
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;


/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_DirectGrasperDeviceController : public HapticAvatar_RigidDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_DirectGrasperDeviceController, HapticAvatar_RigidDeviceController);

    /// Default constructor
    HapticAvatar_DirectGrasperDeviceController();

    /// handleEvent component method to catch collision info
    void handleEvent(core::objectmodel::Event *) override;
    
    /// General Haptic thread methods
    static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    /// Thread methods to cpy data from m_hapticData to m_simuData
    static void CopyData(std::atomic<bool>& terminate, void * p_this);

protected:
    /// Internal method to init specific collision components
    void initImpl() override;

    /// override method to create the different threads
    bool createHapticThreads() override;

    /// override method to update specific tool position
    void updatePositionImpl() override;

    /// Internal method to draw specific informations
    void drawImpl(const sofa::core::visual::VisualParams*) override {}

};

} // namespace sofa::HapticAvatar
