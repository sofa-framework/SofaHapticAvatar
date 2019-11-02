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
#include <SofaHapticAvatar/PortalController.h>
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

PortalController::PortalController(int id, int rail, float railPos, float flipAngle, float tiltAngle, std::string comPort)
    : m_id (id)
    , m_rail(rail)
    , m_railPos(railPos)
    , m_flipAngle(flipAngle)
    , m_tiltAngle(tiltAngle)
    , m_comPort(comPort)
{
    
}


void PortalController::printInfo()
{
    std::cout << "## PortalController Number: " << m_id << std::endl;
    std::cout << "# Settings: ComPort: " << m_comPort << " | Rail Number: " << m_rail << std::endl;
    std::cout << "# values: RailPos: " << m_railPos << " | FlipAngle: " << m_flipAngle << " | TiltAngle: " << m_tiltAngle << std::endl;
    std::cout << "##################################" << std::endl;
}

} // namespace controller

} // namespace component

} // namespace sofa
