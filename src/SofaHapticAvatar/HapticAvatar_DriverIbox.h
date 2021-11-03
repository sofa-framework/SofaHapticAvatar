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
#include <sofa/defaulttype/Vec.h>
#include <string>


namespace sofa::HapticAvatar
{

#ifndef IBOX_NUM_CHANNELS
#define IBOX_NUM_CHANNELS 6
#endif

    class SOFA_HAPTICAVATAR_API HapticAvatar_DriverIbox : public HapticAvatar_DriverBase
    {
    public:
        HapticAvatar_DriverIbox(const std::string& portName);


        /**  TODO check if returning a Vec4 is the best to do
        */
        float getOpeningValue(int toolId);
        void setForce(int toolId, float force);
        int getStatus();
        int getCalibrationStatus(int toolId);
        float getBatteryVoltage();
        float getBoardTemp();
        int getLastPWM(int toolId);
        void setForceFeedbackEnable(bool on);
        float getCurrentDeltaT();
        void setLoopGain(int chan, float loopGainP, float loopGainD);
        float getSensedForce(int toolId);
        void setZeroForce(int toolId);
        float getPosVoltage(int toolId);
        int getSerialNumber() override;
        float getChargingCurrent();


        //void setHandleForces(float upperJawForce, float lowerJawForce);

        virtual void printStatus();


        float getPartTemperature(int part);



    protected:
        int convertToolIdToChannel(int toolId);

        /*
        /// Internal method to get the enum id for reset command. To be overwritten by child
        int getResetCommandId() override { return CmdIBox::RESET_IBOX; }

        /// Internal method to get the enum id for identity command. To be overwritten by child
        int getIdentityCommandId() override { return CmdIBox::GET_IBOX_DEVICE_TYPE; }

        /// Internal method to get the enum id for toolID command. To be overwritten by child
        int getToolIDCommandId() override { return CmdIBox::GET_IBOX_HANDLE_IDS; }

        /// Internal method to get the enum id for status command. To be overwritten by child
        int getStatusCommandId() override { return CmdIBox::GET_IBOX_STATUS; }
        */

        void setupNumReturnVals();
        void setupCmdLists();

    private:
        enum CmdIBox
        {
            RESET = 0,
            GET_DEVICE_TYPE,
            GET_OPENING_VALUES,
            GET_HANDLE_IDS,		// deprecated
            GET_PEDAL_STATES,		// also available in GET_STATUS
            SET_ALL_FORCES,
            SET_CHAN_FORCE,
            GET_STATUS,
            GET_CALIBRATION_STATUS, // deprecated
            GET_MOTOR_BOARD_STATUS, // also available in GET_STATUS
            GET_BATTERY_VOLTAGE,
            GET_BOARD_TEMP,
            SET_MANUAL_PWM,
            SET_POWER_ON_MANUAL,
            SET_FAN_ON_MANUAL,
            GET_LAST_PWM,
            SET_FF_ENABLE,
            GET_CURRENT_DELTA_T,
            SET_LOOP_GAIN,
            GET_OPTO_FORCES,
            SET_ZERO_FORCE,
            GET_POS_VOLTAGES,
            GET_HANDLE_IDS_REAL,	// deprecated
            GET_BUILD_DATE,
            GET_SERIAL_NUM,
            SET_CHARGE_ENABLE,
            SET_HANDLE_LED,
            GET_OPTO_VOLTAGES,
            SET_TO_CALIBRATE,
            GET_PART_TEMPERATURES,
            SET_MAX_USB_CHARGE_CURRENT,
            GET_USB_CHARGING_CURRENT,
            GET_CONNECTION_STATES,
            GET_HANDLES_ACTIVITY,
            SET_FORCE_OFFSET,     // use this for making a handle open itself as if it was a spring
            ALWAYS_LAST
        };

        enum IboxThermalSimPart {
            MotorWinding0 = 0, MotorWinding1, MotorWinding2, MotorWinding3,
            MotorHousing0, MotorHousing1, MotorHousing2, MotorHousing3,
            HeatSink, CoolingAir, Ambient, NumParts
        };
    };
}