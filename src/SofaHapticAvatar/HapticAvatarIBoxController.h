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

#include <SofaHapticAvatar/HapticAvatarDriver.h>
#include <SofaHapticAvatar/HapticAvatarPortalManager.h>

#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>

#include <SofaHaptics/ForceFeedback.h>

#include <atomic>

namespace sofa::component::controller
{

using namespace sofa::defaulttype;
using namespace sofa::simulation;


/**
* Haptic Avatar IBox controller
*/
class SOFA_HAPTICAVATAR_API HapticAvatarIBoxController : public Controller
{

public:
    SOFA_CLASS(HapticAvatarIBoxController, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    HapticAvatarIBoxController();

	virtual ~HapticAvatarIBoxController();

    virtual void init() override;
    virtual void bwdInit() override;
    virtual void reinit() override;

    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;

    float getJawOpeningAngle();

private:
    void clearDevice();

private:
    HapticAvatarDriver * m_HA_driver;
    bool m_deviceReady;
};

} // namespace sofa::component::controller
