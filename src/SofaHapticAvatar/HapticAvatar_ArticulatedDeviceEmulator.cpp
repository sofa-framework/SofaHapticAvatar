/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_ArticulatedDeviceEmulator.h>


#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/core/objectmodel/KeyreleasedEvent.h>


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
    : HapticAvatar_BaseDeviceController()
    , d_articulations(initData(&d_articulations, "articulations", "Output data position of the tool"))
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);

    m_toolRot.identity();

    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations.resize(6);
    articulations[0] = 0;
    articulations[1] = 0;
    articulations[2] = 0;
    articulations[3] = 0;

    articulations[4] = 0;
    articulations[5] = 0;
    d_articulations.endEdit();
}


bool HapticAvatar_ArticulatedDeviceEmulator::createHapticThreads()
{
    m_terminate = false;
    /*haptic_thread = std::thread(Haptics, std::ref(this->m_terminate), this, m_HA_driver);
    copy_thread = std::thread(CopyData, std::ref(this->m_terminate), this);
    m_hapticData.hapticForces.resize(5);
    m_simuData.hapticForces.resize(5);*/

    return true;
}



void HapticAvatar_ArticulatedDeviceEmulator::updatePositionImpl()
{
    
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe1(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[0] += value;
    d_articulations.endEdit();
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe2(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[1] += value;
    d_articulations.endEdit();
}

void HapticAvatar_ArticulatedDeviceEmulator::moveRotationAxe3(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[2] += value;
    d_articulations.endEdit();
}

void HapticAvatar_ArticulatedDeviceEmulator::moveTranslationAxe1(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[3] += value;
    d_articulations.endEdit();
}


void HapticAvatar_ArticulatedDeviceEmulator::openJaws(Articulation value)
{
    VecArticulation & articulations = *d_articulations.beginEdit();
    articulations[4] += value;
    articulations[5] -= value;
    d_articulations.endEdit();
}


void HapticAvatar_ArticulatedDeviceEmulator::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (sofa::core::objectmodel::KeypressedEvent* ke = dynamic_cast<sofa::core::objectmodel::KeypressedEvent*>(event))
    {
        if (ke->getKey() == '1')
            moveRotationAxe1(Articulation(0.01));
        else if (ke->getKey() == '3')
            moveRotationAxe1(Articulation(-0.01));
        else if (ke->getKey() == '4')
            moveRotationAxe2(Articulation(0.01));
        else if (ke->getKey() == '6')
            moveRotationAxe2(Articulation(-0.01));
        else if (ke->getKey() == '7')
            moveRotationAxe3(Articulation(0.01));
        else if (ke->getKey() == '9')
            moveRotationAxe3(Articulation(-0.01));
        else if (ke->getKey() == '8')
            moveTranslationAxe1(Articulation(1.0));
        else if (ke->getKey() == '2')
            moveTranslationAxe1(Articulation(-1.0));
        else if (ke->getKey() == '+')
            openJaws(Articulation(0.1));
        else if (ke->getKey() == '-')
            openJaws(Articulation(-0.1));
    }

    
}


} // namespace sofa::HapticAvatar