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
    , m_yawAngle(0.0f)
    , m_pitchAngle(0.0f)
    , m_hasMoved(false)
{
    m_portalMtx.identity();
}


void HapticAvatarPortalController::portalSetup()
{
    m_rootPosition[0] = m_railPos;
    float railDistance = float(RAIL_DISTANCE);
    m_rootPosition[2] = m_rail * railDistance;
    if (fabs(m_rail) >= 2)
    {
        m_rootPosition[1] = 174.23f;  // this is the outer rails
    }
    else
    {
        m_rootPosition[1] = 194.23f; //mm
    }

    m_rootOrientation = sofa::defaulttype::Quat::fromEuler(0.0f, m_flipAngle*EULER_TO_RAD, m_tiltAngle*EULER_TO_RAD);
    if (m_flipAngle == 180) // TODO: remove this hack. FIX problem in fromEuler sign in SOFA for extrem angles.
        m_rootOrientation[0] *= -1;

    //std::cout << "m_flipAngle: " << m_flipAngle << std::endl;
    //std::cout << "m_tiltAngle: " << m_tiltAngle << std::endl;
    //std::cout << "orientation: " << orientation << std::endl;
    m_portalPosition.getCenter() = m_rootPosition;
    m_portalPosition.getOrientation() = m_rootOrientation;
}

void HapticAvatarPortalController::updatePostion(float yawAngle, float pitchAngle)
{
   // std::cout << "yawAngle: " << yawAngle << std::endl;
    //std::cout << "pitchAngle: " << pitchAngle << std::endl;

    m_yawAngle = yawAngle;
    m_pitchAngle = pitchAngle;
    m_hasMoved = true;
}


sofa::defaulttype::Mat4x4f MatFromTranslation(sofa::defaulttype::Vec3f trans)
{
    sofa::defaulttype::Mat4x4f mat;
    
    mat.identity();
    for (unsigned int i = 0; i < 3; ++i)
        mat[i][0] = trans[i];
    std::cout << "MatFromTranslation: " << mat << std::endl;

    return mat;
}

sofa::defaulttype::Mat4x4f MatFromRotation(sofa::defaulttype::Quat rot)
{
    sofa::defaulttype::Mat4x4f mat;
    mat[3][3] = 1;
    sofa::defaulttype::Mat3x3f rotM;
    rot.toMatrix(rotM);
    
    for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++)
            mat[i][j] = rotM[i][j];
    std::cout << "MatFromRotation: " << mat << std::endl;
    return mat;
}

const HapticAvatarPortalController::Coord& HapticAvatarPortalController::getPortalPosition()
{
    if (m_hasMoved == false)
        return m_portalPosition;
    
    // else compute new position
    // position = root position x init flip rotation
    // position = position * yaw rotation
    // position = position * pitch rotation
    // potion = position + gear translation
    //MatFromTranslation(m_rootPosition);

    sofa::defaulttype::Mat4x4f T_gear = sofa::defaulttype::Mat4x4f::transformTranslation(Vec3f(8.8f, 0.0f, 0.0f));    
    sofa::defaulttype::Mat4x4f T_portal = sofa::defaulttype::Mat4x4f::transformTranslation(m_rootPosition);
    
    sofa::defaulttype::Mat4x4f R_tiltflip = sofa::defaulttype::Mat4x4f::transformRotation(m_rootOrientation);
    sofa::defaulttype::Quat pitchRot = sofa::defaulttype::Quat::fromEuler(0.0f, 0.0f, -m_pitchAngle);
    sofa::defaulttype::Quat yawRot = sofa::defaulttype::Quat::fromEuler(m_yawAngle, 0.0f, 0.0f);
    
    sofa::defaulttype::Mat4x4f R_yaw = sofa::defaulttype::Mat4x4f::transformRotation(pitchRot);
    sofa::defaulttype::Mat4x4f R_pitch = sofa::defaulttype::Mat4x4f::transformRotation(yawRot);
    
    m_portalMtx = T_portal * R_tiltflip * R_yaw * R_pitch * T_gear;
    
    bool debug = false;
    if (debug)
    {
        std::cout << "------ portalId: " << m_id << " ------" << std::endl;
        std::cout << "T_gear: " << T_gear << std::endl;
        std::cout << "T_portal: " << T_portal << std::endl;
        std::cout << "R_tiltflip: " << R_tiltflip << std::endl;
        std::cout << "R_yaw: " << R_yaw << std::endl;
        std::cout << "R_pitch: " << R_pitch << std::endl;
        std::cout << "m_portalMtx: " << m_portalMtx << std::endl;
        std::cout << "---------------------------------" << std::endl;
    }

    sofa::defaulttype::Mat3x3f rotM;
    for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++)
            rotM[i][j] = m_portalMtx[i][j];

    sofa::defaulttype::Quat orien;
    orien.fromMatrix(rotM);
    m_portalPosition.getCenter() = Vec3f(m_portalMtx[0][3], m_portalMtx[1][3], m_portalMtx[2][3]);
    m_portalPosition.getOrientation() = orien;
    //m_portalPosition.getOrientation() = fullRot;
    
    
    //Vec3f position = m_rootOrientation.rotate(m_rootPosition)

    //sofa::defaulttype::Quat fullRot = m_rootOrientation *pitchRot * yawRot;

    //Vec3f T_gear = Vec3f(8.8f, 0.0f, 0.0f);
    //Vec3f translation = m_rootPosition + T_gear;
    

    
    
    /*rot.toMatrix(rotM);

    for (unsigned int i = 0; i < 3; i++)
        for (unsigned int j = 0; j < 3; j++)
            mat[i][j] = rotM[i][j];*/



    //m_portalPosition.getCenter() = translation;
    //m_portalPosition.getOrientation() = fullRot;

    m_hasMoved = false;
    return m_portalPosition;
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
