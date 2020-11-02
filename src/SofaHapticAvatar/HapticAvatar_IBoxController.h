/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/config.h>
#include <SofaHapticAvatar/HapticAvatar_Driver.h>

#include <SofaUserInteraction/Controller.h>

namespace sofa::HapticAvatar
{

using namespace sofa::simulation;
using namespace sofa::defaulttype;
using namespace sofa::component::controller;


/**
* Haptic Avatar IBox controller
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_IBoxController : public Controller
{

public:
    SOFA_CLASS(HapticAvatar_IBoxController, Controller);

    HapticAvatar_IBoxController();

	virtual ~HapticAvatar_IBoxController();

    virtual void init() override;

    Data<std::string> d_portName;
    Data<std::string> d_hapticIdentity;

    float getJawOpeningAngle();

private:
    void clearDevice();

private:
    HapticAvatar_Driver * m_HA_driver;
    bool m_deviceReady;
};

} // namespace sofa::HapticAvatar
