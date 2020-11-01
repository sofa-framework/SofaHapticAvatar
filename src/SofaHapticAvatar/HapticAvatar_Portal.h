/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2019 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <SofaHapticAvatar/config.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <string>

namespace sofa::component::controller
{

/**
* HapticAvatar_Portal 
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_Portal
{
public:
    typedef sofa::defaulttype::RigidTypes::Coord Coord;
    typedef sofa::defaulttype::Vec3f Vec3f;
    typedef sofa::defaulttype::Vec4f Vec4f;

    HapticAvatar_Portal(int id, int rail, float railPos, float flipAngle, float tiltAngle, std::string comPort);

    virtual ~HapticAvatar_Portal() {}


    void portalSetup();

    void updatePostion(float yawAngle, float pitchAngle);

    void printInfo();

    const Coord& getPortalPosition();
    const std::string& getPortalCom() {return m_comPort;}
  
    const sofa::defaulttype::Mat4x4f& getPortalTransform() { return m_portalMtx; }
private:
    int m_id; ///< 
    int m_rail; ///< rail number, middle rail has number 0
    float m_railPos; ///< rail position, mm from centrum in the rail
    float m_flipAngle; ///< Angles in degrees that the haptic device is flipped 
    float m_tiltAngle; ///< Angles in degrees that the haptic device is tilted
    std::string m_comPort; ///< COM port assigned to the device in the portal

    float m_yawAngle;
    float m_pitchAngle;

    bool m_hasMoved;
    Vec3f m_rootPosition;
    sofa::defaulttype::Quat m_rootOrientation;
    Coord m_portalPosition;

    sofa::defaulttype::Mat4x4f m_portalMtx;
};

} // namespace sofa::component::controller
