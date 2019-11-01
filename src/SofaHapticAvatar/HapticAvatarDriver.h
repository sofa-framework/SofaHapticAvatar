/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#ifndef SOFA_HAPTICAVATAR_HAPTICAVATARDRIVER_H
#define SOFA_HAPTICAVATAR_HAPTICAVATARDRIVER_H

//Geomagic include
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

    Data<bool> d_drawDevice;
private:
    void clearDevice();
};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_HAPTICAVATAR_HAPTICAVATARDRIVER_H
