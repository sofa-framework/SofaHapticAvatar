/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/

#include <SofaHapticAvatar/HapticAvatar_BaseDeviceController.h>
#include <SofaHapticAvatar/HapticAvatar_HapticThreadManager.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/simulation/AnimateEndEvent.h>

#include <sofa/core/visual/VisualParams.h>
#include <iomanip> 

namespace sofa::HapticAvatar
{

//constructeur
HapticAvatar_BaseDeviceController::HapticAvatar_BaseDeviceController()
    : d_portName(initData(&d_portName, std::string("//./COM3"), "portName", "Name of the port used by this device"))
    , d_hapticIdentity(initData(&d_hapticIdentity, "hapticIdentity", "Data to store Information received by HW device"))
    , d_drawDebug(initData(&d_drawDebug, false, "drawDebugForce", "Parameter to draw debug information"))
{
    this->f_listening.setValue(true);
    
    d_hapticIdentity.setReadOnly(true);
}


HapticAvatar_BaseDeviceController::~HapticAvatar_BaseDeviceController()
{
    clearImpl();
    HapticAvatar_HapticThreadManager::kill();
}

void HapticAvatar_BaseDeviceController::clearImpl()
{
    clearDevice();
}

//executed once at the start of Sofa, initialization of all variables excepts haptics-related ones
void HapticAvatar_BaseDeviceController::init()
{
    msg_info() << "HapticAvatar_BaseDeviceController::init()";
    initDevice();
}


void HapticAvatar_BaseDeviceController::draw(const sofa::core::visual::VisualParams* vparams)
{
    if (!m_deviceReady)
        return;

    // draw internal specific info
    drawImpl(vparams);

    // If true draw debug information
    if (d_drawDebug.getValue())
    {
        drawDebug(vparams);
    }
}


//void HapticAvatar_BaseDeviceController::drawDebug(const sofa::core::visual::VisualParams* vparams)
//{
//    if (d_drawLogOutputs.getValue())
//    {
//        int newLine = 12;
//        int fontS = 12;
//        const sofa::type::fixed_array<float, 4>& dofV = m_debugData.anglesAndLength;
//        const sofa::type::fixed_array<float, 4>& motV = m_debugData.motorValues;
//        sofa::type::RGBAColor color(0.0, 1.0, 0.0, 1.0);
//
//        std::string title = "       Yaw   Pitch   Rot   Z";
//        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, title.c_str());
//        newLine += fontS * 2;
//
//        std::stringstream ss;
//        ss << std::fixed << std::setprecision(2) << "Value  "
//            << dofV[Dof::YAW] << "  "
//            << dofV[Dof::PITCH] << "  "
//            << dofV[Dof::ROT] << "  "
//            << dofV[Dof::Z];
//
//        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
//        newLine += fontS * 2;
//
//        ss.str(std::string());
//        ss << std::fixed << std::setprecision(2) << "Motor  "
//            << motV[Dof::YAW] << "  "
//            << motV[Dof::PITCH] << "  "
//            << motV[Dof::ROT] << "  "
//            << motV[Dof::Z];
//
//        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
//        newLine += fontS * 4;
//
//
//
//        std::string title2 = "           XForce  YForce  Zforce  JTorq";
//        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, title2.c_str());
//        newLine += fontS * 2;
//
//        ss.str(std::string());
//        ss << std::fixed << std::setprecision(2) << "Jaws opening  "
//            << m_debugData.jawOpening << "    ";
//
//        vparams->drawTool()->writeOverlayText(8, newLine, fontS, color, ss.str().c_str());
//        newLine += fontS * 2;
//    }
//}


void HapticAvatar_BaseDeviceController::handleEvent(core::objectmodel::Event *event)
{
    if (!m_deviceReady)
        return;

    if (dynamic_cast<sofa::simulation::AnimateBeginEvent *>(event))
    {
        HapticAvatar_HapticThreadManager::getInstance()->setSimulationStarted();
        simulation_updateData();
    }
}

} // namespace sofa::HapticAvatar