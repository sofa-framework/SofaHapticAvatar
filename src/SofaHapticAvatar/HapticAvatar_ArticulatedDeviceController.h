/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_DriverPort.h>
#include <SofaHapticAvatar/HapticAvatar_PortalManager.h>
#include <SofaHaptics/LCPForceFeedback.h>


namespace sofa::HapticAvatar
{

using namespace sofa::defaulttype;

class HapticAvatar_IBoxController;

/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_ArticulatedDeviceController : public HapticAvatar_BaseDeviceController
{
public:
    SOFA_CLASS(HapticAvatar_ArticulatedDeviceController, HapticAvatar_BaseDeviceController);

    using LCPForceFeedback = sofa::component::controller::LCPForceFeedback<sofa::defaulttype::Vec1dTypes>;
    using ArticulationSize = unsigned int;

    /// Default constructor
    HapticAvatar_ArticulatedDeviceController();

    ~HapticAvatar_ArticulatedDeviceController();

    /// SOFA api method called after all components have been init
    void bwdInit() override;
    
    /// Method to be overriden by device specialisation to update the articulations from haptic information
    virtual void haptic_updateArticulations(HapticAvatar_IBoxController* _iBox) { SOFA_UNUSED(_iBox); }

    /// Method to be overriden by device specialisation to compute and update the force feedback given the articulations
    virtual void haptic_updateForceFeedback(HapticAvatar_IBoxController* _iBox) { SOFA_UNUSED(_iBox); }


    HapticAvatar_DriverBase* getBaseDriver() override { return m_HA_driver; }

protected:
    /// HapticAvatar_BaseDeviceController api override
    ///{
    void initDevice() override;
    void clearDevice() override;
    void simulation_updateData() override;
    ///}

    /// Main method to start haptic threads. To be overriden by device specialization
    virtual bool createHapticThreads() = 0;

    /// Main method to update the tool device from haptic information. 
    /// Will call @sa updatePortalAnglesAndLength and @sa updatePositionImpl
    virtual void updatePosition();

    /// Internal method to bo overriden by child class to propagate specific position. Called by @sa updatePosition
    virtual void updatePositionImpl() = 0;


public:
    /// output data position of the tool
    Data<VecCoord> d_toolPosition;

    /// Bool to notify thread to stop work
    std::atomic<bool> m_terminate = true;

    /// Pointer to the ForceFeedback component
    LCPForceFeedback::SPtr m_forceFeedback;

    // link to the forceFeedBack component, if not set will search through graph and take first one encountered
    SingleLink<HapticAvatar_ArticulatedDeviceController, LCPForceFeedback, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_forceFeedback;
    
    /// Link to the portalManager component
    SingleLink<HapticAvatar_ArticulatedDeviceController, HapticAvatar_PortalManager, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_portalMgr;


protected:
    /// Pointer to the internal Driver for device API communication
    HapticAvatar_DriverPort* m_HA_driver = nullptr;

    /// Pointer to the portal manager to get information from the current portal
    HapticAvatar_PortalManager* m_portalMgr = nullptr;


    /// Data public for haptic thread
    /// Structure used to transfer data fromt he haptic thread to the simulation thread.
    struct DeviceData
    {
        sofa::type::fixed_array<float, 4> anglesAndLength;
        sofa::type::fixed_array<float, 4> motorValues;
        int toolId;
        float jawOpening;
    };

    /// Data belonging to the haptic thread only
    DeviceData m_hapticData;
    /// Data used in the copy thread to copy @sa m_hapticData into this data that can be used by simulation thread.
    DeviceData m_simuData;
    /// values returned by tool: Rot angle, Pitch angle, z Length, Yaw Angle
    DeviceData m_debugData;


    VecCoord m_toolPositionCopy;
    VecDeriv m_resForces;
    ArticulationSize m_nbArticulations;
    /// Id of the port returned by portalManager
    int m_portId = -1;

    sofa::type::Mat3x3f m_toolRot;
    sofa::type::Mat3x3f m_toolRotInv;
    sofa::type::Mat3x3f m_PortalRot;
    sofa::type::Mat4x4f m_instrumentMtx;

    std::thread copy_thread;
};

} // namespace sofa::HapticAvatar
