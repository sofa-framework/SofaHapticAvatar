/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#ifndef SOFA_HAPTICAVATAR_DEVICECONTROLLER_H
#define SOFA_HAPTICAVATAR_DEVICECONTROLLER_H

#include <SofaHapticAvatar/config.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/Quater.h>

#include <SofaUserInteraction/Controller.h>

#include <SofaHapticAvatar/HapticAvatarAPI.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

namespace sofa 
{

namespace component 
{

namespace controller 
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;


class HapticAvatarDeviceController;

class SOFA_HAPTICAVATAR_API HapticEmulatorTask : public CpuTask
{
public:
    HapticEmulatorTask(HapticAvatarDeviceController* ptr, CpuTask::Status* pStatus);

    virtual ~HapticEmulatorTask() {}

    virtual MemoryAlloc run() override final;

private:
    HapticAvatarDeviceController * m_driver;
};


/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatarDeviceController : public Controller
{

public:
    SOFA_CLASS(HapticAvatarDeviceController, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    HapticAvatarDeviceController();

	virtual ~HapticAvatarDeviceController();

    virtual void init() override;
    virtual void bwdInit() override;
    virtual void reinit() override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;

    void updatePosition();
    void handleEvent(core::objectmodel::Event *) override;

    Data<Vec3d> d_positionBase; ///< Position of the interface base in the scene world coordinates
    Data<Quat> d_orientationBase; ///< Orientation of the interface base in the scene world coordinates
    Data<Quat> d_orientationTool; ///< Orientation of the tool

    Data<SReal> d_scale; ///< Default scale applied to the Phantom Coordinates
    Data<SReal> d_forceScale; ///< Default forceScale applied to the force feedback. 
    Data< Coord > d_posDevice; ///< position of the base of the part of the device    

    Data<bool> d_drawDevice;
    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;

private:
    void clearDevice();

private:
    HapticAvatarAPI * m_HA_API;
    sofa::simulation::TaskScheduler* m_taskScheduler;
    std::mutex lockPosition;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_HAPTICAVATAR_DEVICECONTROLLER_H
