/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/config.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/Quater.h>

#include <SofaUserInteraction/Controller.h>

#include <SofaHapticAvatar/HapticAvatar_Driver.h>
#include <SofaHapticAvatar/HapticAvatar_PortalManager.h>
#include <SofaHapticAvatar/HapticAvatar_IBoxController.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

#include <SofaHaptics/LCPForceFeedback.h>
#include <SofaHaptics/ForceFeedback.h>

#include <atomic>

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
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef RigidTypes::VecDeriv VecDeriv;
    typedef SolidTypes<double>::Transform Transform;
    typedef sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Rigid3Types> LCPForceFeedback;

    /// default constructor
    HapticAvatar_BaseDeviceController();

    /// default destructor
	virtual ~HapticAvatar_BaseDeviceController();

    /// Component API 
    ///{
    void init() override;
    void bwdInit() override;    
    void handleEvent(core::objectmodel::Event *) override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;
    ///}

protected:
    /// Main method to start haptic threads
    virtual bool createHapticThreads() = 0;
    
    /// Main method to clear the device
    void clearDevice();

    /// Main method to update the tool device from haptic information. 
    /// Will call @sa updatePortalAnglesAndLength and @sa updatePositionImpl
    void updatePosition();

    /// Method to propage
    void updatePortalAnglesAndLength(sofa::helper::fixed_array<float, 4> values);

    /// Internal method to bo overriden by child class to propagate specific position. Called by @sa updatePosition
    virtual void updatePositionImpl() {};

    /// Internal method to bo overriden by child class to draw specific information. Called by @sa draw
    virtual void drawImpl(const sofa::core::visual::VisualParams*) {};

    /// Internal method to bo overriden by child class to draw debug information. Called by @sa draw if d_drawDebug is true
    virtual void drawDebug(const sofa::core::visual::VisualParams* vparams);


public:
    /// Name of the port for this device
    Data<std::string> d_portName; 
    /// Data to store Information received by HW device
    Data<std::string> d_hapticIdentity;

    /// Default scale applied to the Phantom Coordinates
    Data<SReal> d_scale; 
    /// Scale to apply to the forcefeedback. //TODO: check if this is still needed
    Data<SReal> m_forceScale;

    /// position of the base of the device
    Data< Coord > d_posDevice;
    /// output data position of the tool
    Data<VecCoord> d_toolPosition;

    /// Parameter to dump thread info. //TODO: check if this should not be removed...
    Data<bool> d_dumpThreadInfo;
    
    /// Data parameter to draw dof axis
    Data<bool> d_drawDeviceAxis;
    /// Data parameter to draw debug information
    Data<bool> d_drawDebug;
    /// Data parameter to draw output logs
    Data<bool> d_drawLogOutputs;

    
    /// Link to the portalManager component
    SingleLink<HapticAvatar_BaseDeviceController, HapticAvatar_PortalManager, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_portalMgr;

public: 
    /// Data public for haptic thread

    /// Structure used to transfer data fromt he haptic thread to the simulation thread.
    struct DeviceData
    {
        sofa::helper::fixed_array<float, 4> anglesAndLength;
        sofa::helper::fixed_array<float, 4> motorValues;
        sofa::helper::fixed_array<float, 3> collisionForces;
        VecDeriv hapticForces;
        float jawOpening;
    };

    /// Data belonging to the haptic thread only
    DeviceData m_hapticData;
    /// Data used in the copy thread to copy @sa m_hapticData into this data that can be used by simulation thread.
    DeviceData m_simuData;

    /// Pointer to the ForceFeedback component
    LCPForceFeedback::SPtr m_forceFeedback;

    /// Boolean to warn scheduler when SOFA has started the simulation (changed by AnimateBeginEvent)
    bool m_simulationStarted; 
    
    /// Bool to notify thread to stop work
    std::atomic<bool> m_terminate;


protected:
    /// Pointer to the internal Driver for device API communication
    HapticAvatar_Driver * m_HA_driver;
    /// Pointer to the portal manager to get information from the current portal
    HapticAvatar_PortalManager * m_portalMgr;
    
    /// values returned by tool: Rot angle, Pitch angle, z Length, Yaw Angle
    DeviceData m_debugData;

    /// Parameter to know if device is ready or not.
    bool m_deviceReady;
    /// Id of the port returned by portalManager
    int m_portId;
    
    std::thread haptic_thread;
    std::thread copy_thread;


    sofa::defaulttype::Mat3x3f m_toolRot;
    sofa::defaulttype::Mat3x3f m_toolRotInv;
    sofa::defaulttype::Mat3x3f m_PortalRot;
};

} // namespace sofa::HapticAvatar
