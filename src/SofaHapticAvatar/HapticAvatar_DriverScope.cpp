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

#include <SofaHapticAvatar/HapticAvatar_DriverScope.h>
#include <sofa/helper/logging/Messaging.h>

namespace sofa::HapticAvatar
{

using namespace HapticAvatar;


///////////////////////////////////////////////////////////////
/////      Methods for specific device communication      /////
///////////////////////////////////////////////////////////////

HapticAvatar_DriverScope::HapticAvatar_DriverScope(const std::string& portName)
    : HapticAvatar_DriverBase(portName)
{
    setupNumReturnVals();  // needs to be implemented in each device driver
    setupCmdLists();   // needs to be implemented in each device driver

    device_type = 3;

}

void HapticAvatar_DriverScope::setupNumReturnVals()
{
    num_return_vals[(int)CmdScope::RESET] = 1;
    num_return_vals[(int)CmdScope::GET_DEVICE_TYPE] = 1;
    num_return_vals[(int)CmdScope::GET_BUTTON_STATES] = 3;
    num_return_vals[(int)CmdScope::GET_ZOOM_LEVEL] = 1;
    num_return_vals[(int)CmdScope::GET_CAMERA_ANGLE] = 1;
    num_return_vals[(int)CmdScope::GET_CRC_POLY] = 1;
    num_return_vals[(int)CmdScope::GET_CURRENT_DELTA_T] = 1;


    // Set all scale factor to 1.0 to begin with ...
    for (int i = 0; i < (int)CmdScope::ALWAYS_LAST; i++)
        scale_factor[i] = 1.0f;

    // ... and now change those who are not 1.0
    scale_factor[(int)CmdScope::GET_CAMERA_ANGLE] = 10000.0f;
    scale_factor[(int)CmdScope::GET_CURRENT_DELTA_T] = 10000.0f;

    device_num_cmds = (int)CmdScope::ALWAYS_LAST;
}

void HapticAvatar_DriverScope::setupCmdLists()
{
    // Setup the data you want to subscribe to from the port device here. Subscription commands can only be of type GET_... without input arguments.
    // It is good practice to use prime numbers to avoid that all commands are sent at once.
    subscribeTo((int)CmdScope::GET_CAMERA_ANGLE, 1);
    subscribeTo((int)CmdScope::GET_BUTTON_STATES, 1);
    subscribeTo((int)CmdScope::GET_ZOOM_LEVEL, 1);
    subscribeTo((int)CmdScope::GET_CURRENT_DELTA_T, 17);
}


bool HapticAvatar_DriverScope::getButtonPressed(int button)
{
    if (button >= 0 && button < 3) {
        return (getInt((int)CmdScope::GET_BUTTON_STATES, button) != 0);
    }
    else {
        return false;
    }
}

int HapticAvatar_DriverScope::getZoomLevel()
{
    return getInt((int)CmdScope::GET_ZOOM_LEVEL);
}

float HapticAvatar_DriverScope::getCameraAngle()
{
    return getFloat((int)CmdScope::GET_CAMERA_ANGLE);
}


float  HapticAvatar_DriverScope::getCurrentDeltaT()
{
    return getFloat((int)CmdScope::GET_CURRENT_DELTA_T);
}

int HapticAvatar_DriverScope::getSerialNumber()
{
    return getInt((int)CmdScope::GET_SERIAL_NUM);
}



} // namespace sofa::HapticAvatar
