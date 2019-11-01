/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#ifndef SOFA_HAPTICAVATAR_HAPTICAVATARDRIVER_H
#define SOFA_HAPTICAVATAR_HAPTICAVATARDRIVER_H

#include <SofaHapticAvatar/config.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/Quater.h>

#include <SofaUserInteraction/Controller.h>

namespace sofa 
{

namespace component 
{

namespace controller 
{

using namespace sofa::defaulttype;


/**
* Haptic Avatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatarDriver : public Controller
{

public:
    SOFA_CLASS(HapticAvatarDriver, Controller);
    typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
    typedef SolidTypes<double>::Transform Transform;

    HapticAvatarDriver();

	virtual ~HapticAvatarDriver();

    virtual void init() override;
    virtual void bwdInit() override;
    virtual void reinit() override;
    virtual void draw(const sofa::core::visual::VisualParams* vparams) override;
    void handleEvent(core::objectmodel::Event *) override;

    Data<Vec3d> d_positionBase; ///< Position of the interface base in the scene world coordinates
    Data<Quat> d_orientationBase; ///< Orientation of the interface base in the scene world coordinates
    Data<Quat> d_orientationTool; ///< Orientation of the tool

    Data<SReal> d_scale; ///< Default scale applied to the Phantom Coordinates
    Data<SReal> d_forceScale; ///< Default forceScale applied to the force feedback. 
    Data< Coord > d_posDevice; ///< position of the base of the part of the device    

    Data<bool> d_drawDevice;

private:
    void clearDevice();
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_HAPTICAVATAR_HAPTICAVATARDRIVER_H
