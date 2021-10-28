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

    enum Dof
    {
        ROT = 0,
        PITCH,
        Z,
        YAW
    };


    class SOFA_HAPTICAVATAR_API HapticAvatar_DriverPort : public HapticAvatar_DriverBase
    {
    public:
        HapticAvatar_DriverPort(const std::string& portName);


        /** get the angles of the port (Yaw + Pitch) and the insertion length and rotation of the tool.
        * Command: GET_ANGLES_AND_LENGTH
        * @returns {vec4f} {Tool rotation angle, Pitch angle, Z length, Yaw angle}
        */
        sofa::type::fixed_array<float, 4> getAnglesAndLength();

        /** Get the torque on the jaws around the jaw rotation pin.
        * Command: GET_TOOL_JAW_TORQUE
        * @returns {float} The sum of the torque on the jaws
        */
        // Later. float getJawTorque();
        int getToolID();
        bool getToolInserted();
        float getBoardTemp();
        float getBatteryVoltage();
        sofa::type::fixed_array<int, 4> getRawEncoderValues(); // in counts
        sofa::type::fixed_array<float, 4> getEncoderScalingValues(); // radians/counts
        bool getYawPitchCalibrated();
        int getSerialNumber() override;
        /* Later
        void setJawOpeningAngle();
        float getJawTorque();
        void setToolData();
        sofa::type::fixed_array<float, 3> getToolTipVelocity();
        void setForceFeedbackEnable();
        float getIDTipLength();
        */


        /** Get information about the last pwm sent out to the motors.
        * Command: GET_LAST_PWM
        * @returns {vec4f}  {Rot motor, Pitch motor, Z motor, Yaw motor} PWM values [-2040 .. 2040]
        */
        sofa::type::fixed_array<float, 4> getLastPWM();

        /** To get conversion factors from raw pwm values to torques or forces.
        * Command: GET_MOTOR_SCALING_VALUES
        * @returns {vec4f} {Rot motor, Pitch motor, Z motor, Yaw motor} scaling factor (in Nmm/pwm-value)
        */
        sofa::type::fixed_array<float, 4> getMotorScalingValues();




        /** Set the force and torque output per motor.
        * Command: SET_MOTOR_FORCE_AND_TORQUES
        * @param {vec4f} values: specify what to reset. See doc.
        */
        void setMotorForceAndTorques(float rot, float pitch, float z, float yaw);
        void setDeadBandPWMWidth(float rot, float pitch, float z, float yaw);

        /** Set a force vector on the tool tip plus the tool rotation torque. This is an alternative way to output force (compared to SET_MOTOR_FORCE_AND_TORQUES)
        * Command: SET_TIP_FORCE_AND_ROT_TORQUE
        * @param {vec3f} force: specify what to reset. See doc.
        * @param {float} RotTorque: specify what to reset. See doc.
        */
        //void setTipForceAndRotTorque(sofa::type::fixed_array<float, 4> values);

        /** Will decompose a force vector in device coordinate system to compute torque and force to be sent to apply to the device. Using @sa writeRoughForce.
        * @param {vec3f} force: force vector to apply to the device in its coordinate space.
        */
        // void setManualForceVector(sofa::type::Vector3 force, bool useManualPWM = false);


        //void setTipForceVector(sofa::type::Vector3 force);

        // Will send 0 torque and force values to the device. Using SET_MANUAL_PWM.
        void releaseForce();

        // Will call SET_MANUAL_PWM: Set the force output manually per motor with raw PWM-values. 
        void setManualPWM(float rotTorque, float pitchTorque, float zforce, float yawTorque);

        //void setManual_Force_and_Torques(float rotTorque, float pitchTorque, float zforce, float yawTorque);

    protected:
        /// Internal method to get the enum id for reset command. To be overwritten by child
        //int getResetCommandId() override { return CmdTool::RESET; }

        /// Internal method to get the enum id for identity command. To be overwritten by child
        //int getDeviceTypeCommandId() override { return CmdTool::GET_DEVICE_TYPE; }

        /// Internal method to get the enum id for status command. To be overwritten by child
        //int getSerialNumberCommandId() override { return CmdTool::GET_SERIAL_NUM; }

        void setupNumReturnVals();
        void setupCmdLists();

    private:
        enum CmdPort
        {
            RESET = 0,
            GET_DEVICE_TYPE,
            GET_ANGLES_AND_LENGTH,
            GET_TOOL_ID,
            GET_CURRENT_DELTA_T,
            GET_STATUS,
            SET_MOTOR_FORCE_AND_TORQUES,
            SET_TIP_FORCE_AND_ROT_TORQUE,
            SET_YAW_PITCH_ZERO_ANG,
            SET_LED_BLINK_MODE,
            SET_COLLISION_OBJECT,
            SET_COLLISION_OBJECT_ACTIVE,
            SET_COLLISION_OBJECT_P0,
            SET_COLLISION_OBJECT_V0,
            SET_COLLISION_OBJECT_N,
            SET_COLLISION_OBJECT_Q,
            SET_COLLISION_OBJECT_R,
            SET_COLLISION_OBJECT_S,
            SET_COLLISION_OBJECT_T,
            SET_COLLISION_OBJECT_STIFFNESS,
            SET_COLLISION_OBJECT_DAMPING,
            SET_COLLISION_OBJECT_FRICTION,
            GET_LAST_COLLISION_FORCE,
            GET_LAST_PWM,
            GET_LAST_COLLISION_DATA,
            SET_TOOL_JAW_OPENING_ANGLE,
            GET_TOOL_JAW_TORQUE,
            SET_TOOL_DATA,
            GET_TOOL_INSERTED,
            GET_TOOL_TIP_VELOCITY,
            GET_TOOL_TIP_POSITION,
            GET_TOOL_DIRECTION,
            GET_RAW_ENCODER_VALUES,
            GET_ENCODER_SCALING_VALUES,
            GET_MOTOR_SCALING_VALUES,
            SET_MANUAL_PWM,
            GET_BOARD_TEMP,
            GET_BATTERY_VOLTAGE,
            GET_CALIBRATION_STATUS,
            GET_AMPLIFIERS_STATUS,
            GET_HALL_STATES,
            SET_POWER_ON_MANUAL,
            SET_FAN_ON_MANUAL,
            SET_FF_ENABLE,
            GET_SERIAL_NUM,
            GET_BUILD_DATE,
            SET_CHARGE_ENABLE,
            GET_TIP_LENGTH,
            GET_PART_TEMPERATURES,
            SET_MAX_USB_CHARGE_CURRENT,
            GET_USB_CHARGING_CURRENT,
            SET_DEADBAND_PWM_WIDTH,
            ALWAYS_LAST
        };

    };
} // namespace sofa::HapticAvatar
