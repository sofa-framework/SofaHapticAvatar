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
class SOFA_HAPTICAVATAR_API HapticAvatar_IBoxController : public Controller
{

public:
    SOFA_CLASS(HapticAvatar_IBoxController, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    HapticAvatar_IBoxController();

	virtual ~HapticAvatar_IBoxController();

    virtual void init() override;
    virtual void bwdInit() override;
    virtual void reinit() override;

    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;

    float getJawOpeningAngle();

private:
    void clearDevice();

private:
    HapticAvatar_Driver * m_HA_driver;
    bool m_deviceReady;
};

} // namespace sofa::component::controller
