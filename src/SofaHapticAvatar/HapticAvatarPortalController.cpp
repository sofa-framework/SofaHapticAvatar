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

#include <SofaHapticAvatar/HapticAvatarDefines.h>
#include <SofaHapticAvatar/HapticAvatarPortalController.h>
#include <sofa/helper/Quater.h>
#include <iostream>
#include <algorithm>
#include <stdio.h>
#include <stdlib.h>

namespace sofa
{

namespace component
{

namespace controller
{

HapticAvatarPortalController::HapticAvatarPortalController(int id, int rail, float railPos, float flipAngle, float tiltAngle, std::string comPort)
    : m_id (id)
    , m_rail(rail)
    , m_railPos(railPos)
    , m_flipAngle(flipAngle)
    , m_tiltAngle(tiltAngle)
    , m_comPort(comPort)
{
    
}


void HapticAvatarPortalController::portalSetup()
{
    m_portalPosition[0] = m_railPos;
    float railDistance = float(RAIL_DISTANCE);
    m_portalPosition[2] = m_rail * railDistance;
    if (fabs(m_rail) >= 2)
    {
        m_portalPosition[1] = 174.23f;  // this is the outer rails
    }
    else
    {
        m_portalPosition[1] = 194.23f; //mm
    }

    sofa::defaulttype::Quat orientation = orientation.fromEuler(0.0f, m_flipAngle*EULER_TO_RAD, m_tiltAngle*EULER_TO_RAD);
    if (m_flipAngle == 180) // TODO: remove this hack. FIX problem in fromEuler sign in SOFA for extrem angles.
        orientation[0] *= -1;

    //std::cout << "m_flipAngle: " << m_flipAngle << std::endl;
    //std::cout << "m_tiltAngle: " << m_tiltAngle << std::endl;
    //std::cout << "orientation: " << orientation << std::endl;
    m_portalPosition.getOrientation() = orientation;    
}

void HapticAvatarPortalController::updatePostion(float yawAngle, float pitchAngle)
{
    m_yawAngle = yawAngle;
    m_pitchAngle = pitchAngle;
}


void HapticAvatarPortalController::printInfo()
{
    std::cout << "## HapticAvatarPortalController Number: " << m_id << std::endl;
    std::cout << "# Settings: ComPort: " << m_comPort << " | Rail Number: " << m_rail << std::endl;
    std::cout << "# values: RailPos: " << m_railPos << " | FlipAngle: " << m_flipAngle << " | TiltAngle: " << m_tiltAngle << std::endl;
    std::cout << "# Position: " << m_portalPosition << std::endl;
    std::cout << "##################################" << std::endl;
}

} // namespace controller

} // namespace component

} // namespace sofa
