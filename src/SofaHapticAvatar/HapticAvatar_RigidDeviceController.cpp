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

HapticRigidAvatarJaws::HapticRigidAvatarJaws()
    : m_MaxOpeningAngle(60.0f)
    , m_jawLength(20.0f)
    , m_jaw1Radius(1.5f)
    , m_jaw2Radius(1.5f)
    , m_shaftRadius(2.5f)
{

}


//constructeur
HapticAvatar_RigidDeviceController::HapticAvatar_RigidDeviceController()
    : HapticAvatar_BaseDeviceController()
    , l_iboxCtrl(initLink("iboxController", "link to IBoxController"))
    , d_toolPosition(initData(&d_toolPosition, "toolPosition", "Output data position of the tool"))
    , m_forceFeedback(nullptr)
    , m_iboxCtrl(nullptr)
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


void HapticAvatar_RigidDeviceController::drawImpl(const sofa::core::visual::VisualParams* vparams)
{
    // draw tool rigid dof arrows
    if (d_drawDeviceAxis.getValue())
    {
        const VecCoord & toolPosition = d_toolPosition.getValue();
        float glRadius = float(d_scale.getValue());

        for (unsigned int i = 0; i < toolPosition.size(); ++i)
        {
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(20, 0, 0)*d_scale.getValue()), glRadius, sofa::type::RGBAColor(1, 0, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 20, 0)*d_scale.getValue()), glRadius, sofa::type::RGBAColor(0, 1, 0, 1));
            vparams->drawTool()->drawArrow(toolPosition[i].getCenter(), toolPosition[i].getCenter() + toolPosition[i].getOrientation().rotate(Vector3(0, 0, 20)*d_scale.getValue()), glRadius, sofa::type::RGBAColor(0, 0, 1, 1));
        }
    }

    //if (d_drawDebug.getValue())
    //{
        //const VecCoord & toolPosition = d_toolPosition.getValue();
        //const VecDeriv& force = m_debugData.hapticForces;

        //Vec3 dirTotal, angTotal;
        //for (int i = 0; i < force.size(); i++)
        //{
        //    Vec3 dir = force[i].getLinear();
        //    Vec3 ang = force[i].getAngular();
        //    //vparams->drawTool()->drawLine(toolPosition[i].getCenter(), toolPosition[i].getCenter() + dir * 50, defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0));
        //    //vparams->drawTool()->drawLine(toolPosition[i].getCenter(), toolPosition[i].getCenter() + ang * 50, defaulttype::Vec4f(0.0f, 1.0f, 0.0f, 1.0));

        //    dirTotal += dir;
        //    angTotal += ang;
        //}

    //    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + dirTotal * 50, defaulttype::Vec4f(1.0f, 1.0f, 1.0f, 1.0));
    //    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + angTotal * 50, defaulttype::Vec4f(1.0f, 0.0f, 0.0f, 1.0));
    //    vparams->drawTool()->drawLine(toolPosition[3].getCenter(), toolPosition[3].getCenter() + m_toolRotInv * dirTotal * 50, defaulttype::Vec4f(0.0f, 0.0f, 1.0f, 1.0));
    //}
}


} // namespace sofa::HapticAvatar