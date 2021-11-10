/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceEmulator.h>

#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>


#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <chrono>

namespace sofa::HapticAvatar
{

using namespace HapticAvatar;
using namespace sofa::helper::system::thread;

int HapticAvatar_ArticulatedDeviceEmulatorClass = core::RegisterObject("Driver allowing interfacing with Haptic Avatar device.")
    .add< HapticAvatar_ArticulatedDeviceEmulator >()
    ;


//constructeur
HapticAvatar_ArticulatedDeviceEmulator::HapticAvatar_ArticulatedDeviceEmulator()
    : HapticAvatar_ArticulatedDeviceController()
{
    this->f_listening.setValue(true);
    
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations.resize(6);
    articulations[0] = 0;
    articulations[1] = 0;
    articulations[2] = 0;
    articulations[3] = 0;

    articulations[4] = 0;
    articulations[5] = 0;

    //articulations[0] = dofV[Dof::YAW];
    //articulations[1] = -dofV[Dof::PITCH];
    //articulations[2] = dofV[Dof::ROT];
    //articulations[3] = dofV[Dof::Z];
    //articulations[4] = morsUP;
    //articulations[5] = morsDown;

}


void HapticAvatar_ArticulatedDeviceEmulator::initImpl()
{
    
}

bool HapticAvatar_ArticulatedDeviceEmulator::createHapticThreads()
{
    m_terminate = true;
    return true;
}


void HapticAvatar_ArticulatedDeviceEmulator::updatePositionImpl()
{
    //std::cout << "updatePositionImpl" << std::endl;

    // get info from simuData
    sofa::type::fixed_array<float, 4> dofV = m_simuData.anglesAndLength;

    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    //std::cout << "YAW: " << dofV[Dof::YAW] << " | PITCH: " << dofV[Dof::PITCH] << " | ROT: " << dofV[Dof::ROT] << " | Z: " << dofV[Dof::Z] << std::endl;
    articulations[0] = dofV[Dof::YAW];
    articulations[1] = -dofV[Dof::PITCH];
    articulations[2] = dofV[Dof::ROT];
    articulations[3] = dofV[Dof::Z];

    float _OpeningAngle = m_simuData.jawOpening * 60.0f * 0.01f;
    articulations[4] = _OpeningAngle;
    articulations[5] = -_OpeningAngle;
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe1(Coord value)
{
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations[0] += value;
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe2(Coord value)
{
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations[1] += value;
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe3(Coord value)
{
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations[2] += value;
}

void HapticAvatar_ArticulatedDeviceEmulator::moveTranslationAxe1(Coord value)
{
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations[3] += value;
}


void HapticAvatar_ArticulatedDeviceEmulator::openJaws(Coord value)
{
    sofa::helper::WriteOnlyAccessor < Data<VecCoord> > articulations = d_toolPosition;
    articulations[4] += value;
    articulations[5] -= value;
}


void HapticAvatar_ArticulatedDeviceEmulator::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (sofa::core::objectmodel::KeypressedEvent* ke = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        if (ke->getKey() == '1')
            moveRotationAxe1(Coord(0.01));
        else if (ke->getKey() == '3')
            moveRotationAxe1(Coord(-0.01));
        else if (ke->getKey() == '4')
            moveRotationAxe2(Coord(0.01));
        else if (ke->getKey() == '6')
            moveRotationAxe2(Coord(-0.01));
        else if (ke->getKey() == '7')
            moveRotationAxe3(Coord(0.01));
        else if (ke->getKey() == '9')
            moveRotationAxe3(Coord(-0.01));
        else if (ke->getKey() == '8')
            moveTranslationAxe1(Coord(1.0));
        else if (ke->getKey() == '2')
            moveTranslationAxe1(Coord(-1.0));
        else if (ke->getKey() == '+')
            openJaws(Coord(0.1));
        else if (ke->getKey() == '-')
            openJaws(Coord(-0.1));
    }


    //if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    //{
    //    m_simulationStarted = true;
    //    updatePositionImpl();
    //}
    
}


} // namespace sofa::HapticAvatar
