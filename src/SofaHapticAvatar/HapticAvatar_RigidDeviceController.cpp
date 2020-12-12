/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_RigidDeviceController.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <iomanip> 

namespace sofa::HapticAvatar
{

//constructeur
HapticAvatar_RigidDeviceController::HapticAvatar_RigidDeviceController()
    
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

    HapticAvatar_RigidDeviceController::VecCoord & toolPosition = *d_toolPosition.beginEdit();
    toolPosition.resize(8);
    d_toolPosition.endEdit();    
}


HapticAvatar_RigidDeviceController::~HapticAvatar_RigidDeviceController()
{

}



} // namespace sofa::HapticAvatar