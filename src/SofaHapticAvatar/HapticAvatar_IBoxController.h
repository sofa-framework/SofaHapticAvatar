/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/config.h>
#include <SofaHapticAvatar/HapticAvatar_DriverIbox.h>
#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>

#include <SofaUserInteraction/Controller.h>

namespace sofa::HapticAvatar
{

using namespace sofa::simulation;
using namespace sofa::defaulttype;
using namespace sofa::component::controller;


/**
* Haptic Avatar IBox controller
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_IBoxController : public HapticAvatar_BaseDeviceController
{

public:
    SOFA_CLASS(HapticAvatar_IBoxController, HapticAvatar_BaseDeviceController);

    HapticAvatar_IBoxController();
    ~HapticAvatar_IBoxController() override;

    float getJawOpeningAngle(int toolId);

    void setHandleForce(int toolId, float force);

	void setLoopGain(int chan, float loopGainP, float loopGainD);

    void update();

    HapticAvatar_DriverBase* getBaseDriver() override { return m_HA_driver; }

protected:
    void initDevice() override;
    void clearDevice() override;
    void simulation_updateData() override {}

private:
    HapticAvatar_DriverIbox * m_HA_driver = nullptr;
};

} // namespace sofa::HapticAvatar
