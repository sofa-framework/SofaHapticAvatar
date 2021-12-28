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
#include <SofaHapticAvatar/HapticAvatar_DriverBase.h>
#include <sofa/type/Vec.h>
#include <string>

namespace sofa::HapticAvatar
{

    class SOFA_HAPTICAVATAR_API HapticAvatar_DriverScope : public HapticAvatar_DriverBase
    {
    public:
        HapticAvatar_DriverScope(const std::string& portName);

        // Functions that are typically used at initialization or shutdown
        // ---------------------------------------------------------------
        
        /** Get the unique serial number of the device
        * @returns {int} the serial number, typically a 7-digit number
        */
        int getSerialNumber() override;


        // Functions that are used in the visual loop
        // ------------------------------------------------------------------
      
        /** Get the camera head angle relative the scope shaft.
        * @returns {float} the absolute angle in radians between the camera head and the scope shaft
        */
        float getCameraAngle();


        /** Get the states of one of the three buttons on the camera.
        * @param {button} which button. 0=button closest to the shaft, 1=middle button, 2 = outermost button
        * @returns {bool} where false = not pressed, true=pressed
        */        
        bool getButtonPressed(int button);

        /** Get the current zoom level 
        * @returns {int} limited to -10 to +10. It is up to the user to decide which direction is zoom-in and -out.
        */      
        int getZoomLevel();


        // Functions that are typically used only for statistics, diagnostics and debugging
        // --------------------------------------------------------------------------------

        /** Get the loop time inside the device. Typically around 0.05ms.
        * @returns {float}, the loop time in ms.
        */
        float getCurrentDeltaT();

    protected:
        /// Internal method to setup how many return values each command is expecting and how to scale outgoing and incoming data.
        void setupNumReturnVals() override;
        /// Internal method to setup which data from the device to subscribe to, and how often.
        void setupCmdLists() override;

    private:

        // This enum is a list of all commands. The same list exists in the device.
        enum CmdScope
        {
            RESET = 0,
            GET_DEVICE_TYPE,
            GET_BUTTON_STATES,
            GET_ZOOM_LEVEL,
            GET_CAMERA_ANGLE,
            GET_CRC_POLY,
            GET_CURRENT_DELTA_T,
            GET_SERIAL_NUM,
            ALWAYS_LAST
        };
    };
} // namespace sofa::HapticAvatar
