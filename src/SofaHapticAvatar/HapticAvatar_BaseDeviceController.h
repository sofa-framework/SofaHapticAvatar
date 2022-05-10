/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/config.h>
#include <sofa/type/Vec.h>
#include <SofaHapticAvatar/HapticAvatar_DriverBase.h>
#include <SofaUserInteraction/Controller.h>

namespace sofa::HapticAvatar
{
using namespace sofa::defaulttype;
using namespace sofa::simulation;
using namespace sofa::component::controller;

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_BaseDeviceController : public Controller
{
public:
    SOFA_CLASS(HapticAvatar_BaseDeviceController, Controller);

    using Coord = Vec1Types::Coord;
    using VecCoord = Vec1Types::VecCoord;
    using VecDeriv = Vec1Types::VecDeriv;

    /// default constructor
    HapticAvatar_BaseDeviceController();

    /// Component API 
    ///{
    void init() override;
    void handleEvent(core::objectmodel::Event *) override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;
    ///}

    virtual HapticAvatar_DriverBase* getBaseDriver() = 0;
    
protected:
    /// Internal method to init specific info. Called by init
    virtual void initDevice() = 0;
    
    /// Main method to clear the device
    virtual void clearDevice() {};

    /// Main method from the SOFA simulation call at each simulation step begin.
    virtual void simulation_updateData() = 0;

    
    /// Internal method to bo overriden by child class to draw specific information. Called by @sa draw
    virtual void drawImpl(const sofa::core::visual::VisualParams* vparams) { SOFA_UNUSED(vparams); }

    /// Internal method to bo overriden by child class to draw debug information. Called by @sa draw if d_drawDebug is true
    virtual void drawDebug(const sofa::core::visual::VisualParams* vparams) { SOFA_UNUSED(vparams); }

public:
    /// Name of the port for this device
    Data<std::string> d_portName; 
    /// Data to store Information received by HW device
    Data<std::string> d_hapticIdentity;

    /// Data parameter to draw debug information
    Data<bool> d_drawDebug;    

protected:
    /// Internal parameter to know if device is ready or not.
    bool m_deviceReady = false;
};

} // namespace sofa::HapticAvatar
