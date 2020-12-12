/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceController.h>

#include <sofa/core/ObjectFactory.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>
#include <sofa/simulation/CollisionEndEvent.h>
#include <sofa/core/collision/DetectionOutput.h>

#include <sofa/core/visual/VisualParams.h>

namespace sofa::HapticAvatar
{

using namespace sofa::helper::system::thread;

//constructeur
HapticAvatar_ArticulatedDeviceController::HapticAvatar_ArticulatedDeviceController()
    : HapticAvatar_BaseDeviceController()
    , d_articulations(initData(&d_articulations, "articulations", "Output data position of the tool"))
    , m_forceFeedback1D(nullptr)
{
    this->f_listening.setValue(true);
}



//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_ArticulatedDeviceController::initImpl()
{
    simulation::Node *context = dynamic_cast<simulation::Node *>(this->getContext()); // access to current node
    m_forceFeedback1D = context->get<LCPForceFeedback1D>(this->getTags(), sofa::core::objectmodel::BaseContext::SearchRoot);

    if (m_forceFeedback1D == nullptr)
    {
        msg_warning() << "ForceFeedback not found";
    }


}



} // namespace sofa::HapticAvatar
