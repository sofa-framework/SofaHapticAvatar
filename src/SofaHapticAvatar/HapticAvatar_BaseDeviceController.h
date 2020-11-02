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

    HapticAvatar_BaseDeviceController();

	virtual ~HapticAvatar_BaseDeviceController();

    virtual void init() override;
    virtual void bwdInit() override;    
    void handleEvent(core::objectmodel::Event *) override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;


    void updatePosition();
    

    void updateAnglesAndLength(sofa::helper::fixed_array<float, 4> values);

    Data<SReal> d_scale; ///< Default scale applied to the Phantom Coordinates
    Data< Coord > d_posDevice; ///< position of the base of the part of the device    


    Data<bool> d_logOutputs;
    /// values returned by tool: Rot angle, Pitch angle, z Length, Yaw Angle
    Data<sofa::helper::fixed_array<float, 4> > d_info_toolValues;
    Data<sofa::helper::fixed_array<float, 4> > d_info_motorOutput;
    Data<sofa::helper::fixed_array<float, 3> > d_info_collisionForce;
    Data<float> d_info_jawOpening;

    Data<bool> d_dumpThreadInfo;
    Data<bool> d_drawDeviceAxis;
    Data<bool> d_drawDebugForce;
    

    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;
    Data<int> d_fontSize;

    Data<VecCoord> d_toolPosition;
    
    Data<SReal> m_forceScale;
    bool m_firstStep;

    sofa::helper::vector<Vector3> m_debugForces;

    sofa::helper::vector<float> m_times;

    Coord m_debugRootPosition;
    sofa::defaulttype::Mat3x3f m_toolRot;
    sofa::defaulttype::Mat3x3f m_toolRotInv;
    sofa::defaulttype::Mat3x3f m_PortalRot;
    
    /// General Haptic thread methods
    //static void Haptics(std::atomic<bool>& terminate, void * p_this, void * p_driver);

    //static void CopyData(std::atomic<bool>& terminate, void * p_this);

    std::atomic<bool> m_terminate;
    int m_portId;
    SingleLink<HapticAvatar_BaseDeviceController, HapticAvatar_PortalManager, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK> l_portalMgr;

    LCPForceFeedback::SPtr m_forceFeedback;
    bool m_simulationStarted; ///< Boolean to warn scheduler when SOFA has started the simulation (changed by AnimateBeginEvent)
public:
    struct DeviceData
    {
        sofa::helper::fixed_array<float, 4> anglesAndLength;
        sofa::helper::fixed_array<float, 4> motorValues;
        sofa::helper::fixed_array<float, 3> collisionForces;
        VecDeriv hapticForces;
        float jawOpening;
    };

    DeviceData m_hapticData;
    DeviceData m_simuData;

protected:
    void clearDevice();

    virtual void createHapticThreads() = 0;

protected:
    HapticAvatar_Driver * m_HA_driver;
    HapticAvatar_PortalManager * m_portalMgr;
    bool m_deviceReady;
    
    std::mutex lockPosition;

    std::thread haptic_thread;
    std::thread copy_thread;

};

} // namespace sofa::HapticAvatar
