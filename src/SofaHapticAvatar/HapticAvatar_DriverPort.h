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

#define MAX_NUM_PRIMITIVES 100

    class SOFA_HAPTICAVATAR_API HapticAvatar_DriverPort : public HapticAvatar_DriverBase
    {
    public:
        HapticAvatar_DriverPort(const std::string& portName);

        // Functions that are typically used at initialization or shutdown
        // ---------------------------------------------------------------
        
        /** Get the unique serial number of the device
        * @returns {int} the serial number, typically a 7-digit number
        */
        int getSerialNumber() override;

        /** Set the dead band width. This can be use used to prevent oscillations, typically only used for the rotation dof.
        * @param {rot} is the rot deadband (pwm counts) typically 100 is a good value.
        * @param {rot} is the pitch deadband (pwm counts) typically 0 is a good value.
        * @param {rot} is the z deadband (pwm counts) typically 0 is a good value.
        * @param {rot} is the yaw deadband (pwm counts) typically 0 is a good value.
        */
        void setDeadBandPWMWidth(float rot, float pitch, float z, float yaw);
 
        /** Information about the yaw/pitch calibration
        * @returns {bool} true if both yaw and pitch is calibrated, else false.
        */        
        bool getYawPitchCalibrated();

        /** Get the scaling values from torque or force to PWM output. This information is only needed if setManualPWM() is going to be used.
        * @returns {vec4f} the scaling values for rot (Nmm/pwm-counts), pitch (Nmm/pwm-counts), z (N/pwm-unit) and yaw (Nmm/pwm-counts) respectively.
        */
        sofa::type::fixed_array<float, 4> getMotorScalingValues();

        /** Get the scaling values from encoder count to angles or length. This information is only needed if getRawEncoderValues() is going to be used.
        * @returns {vec4f} the scaling values for rot (counts per radian), pitch (counts per radian), z (counts per mm) and yaw (counts per radian) respectively.
        */
        sofa::type::fixed_array<float, 4> getEncoderScalingValues(); // radians/counts

        /** Enable or disable force feedback, i.e. power output to the motors. 
        * @param {rot} true=enable ff, false=disable ff.
        */
        void setForceFeedbackEnable(bool on);


        // Functions that are used in the haptic loop
        // ------------------------------------------------------------------
      
        /** Get the angles of the port (Yaw + Pitch + Rot) and the insertion length (Z).
        * @returns {vec4f} {Tool rotation (twist) angle, Pitch angle, Z length, Yaw angle}
        */
        sofa::type::fixed_array<float, 4> getAnglesAndLength();

        /** Get the ID of the inserted tool, i.e. the simulated medical instrument (if any).
        * @returns {int} where -1=no tool inserted, 0=tool inserted but not identified, 1,2,3 ... is an identified tool
        */        
        int getToolID();

        /** Ask if a tool is inserted or not, or more specifically, if the opto sensor is occluded. 
        * It is preferred to use getToolID().
        * @returns {bool} where false = not inserted, true = inserted
        */      
        bool getToolInserted();

        /** Set the force and torque output per motor (this is the preferred way).
        * @param {rot} is the rot torque (Nmm), i.e. the twist torque of the shaft
        * @param {pitch} is the pitch torque (Nmm), i.e. back-and-forth torque
        * @param {z} is the insertion force (N), i.e. the twist torque of the shaft
        * @param {yaw} is the yaw torque (Nmm), i.e. the side-to-side torque
        */
        void setMotorForceAndTorques(float rot, float pitch, float z, float yaw);

        /** This is an alternative (non-preferred) way to set the force and torque output per motor.
        * Use the getMotorScalingValues() to get the scaling factors Nmm/pwm or N/pwm initially.
        * @param {rot} is the rot torque (pwm-units, -2000 to +2000), i.e. the twist torque of the shaft
        * @param {pitch} is the pitch torque (pwm-units, -2000 to +2000), i.e. back-and-forth torque
        * @param {z} is the insertion force (pwm-units, -2000 to +2000), i.e. the twist torque of the shaft
        * @param {yaw} is the yaw torque (pwm-units, -2000 to +2000), i.e. the side-to-side torque
        */
        void setManualPWM(int rot, int pitch, int z, int yaw);


        // Functions for collision primitives
        // ------------------------------------------------------------------

        void setInstrumentData(float shaft_diameter, float jaw1_diameter, float jaw2_diameter, float jaw_length);
        void setJawOpeningAngle(float ang);
        float getJawTorque();
        int addSphere(sofa::type::fixed_array<float, 3> pos, float radius, float stiffness, float damping, float friction);
        int addCapsule(sofa::type::fixed_array<float, 3> pos, sofa::type::fixed_array<float, 3> ori, float radius, float length, float stiffness, float damping, float friction);
        int addTorus(sofa::type::fixed_array<float, 3> pos, sofa::type::fixed_array<float, 3> ori, float major_radius, float minor_radius, float stiffness, float damping, float friction);
        void deletePrimitive(int index);
        void deleteAllPrimitives();
        void setActive(int index, bool active);
        void updatePosition(int index, sofa::type::fixed_array<float, 3> new_pos);
        void updateOrientation(int index, sofa::type::fixed_array<float, 3> new_ori);
        void updateRadius1(int index, float radius);
        void updateRadius2(int index, float radius);
        void updateLength(int index, float length);
        void updateStiffness(int index, float stiffness);
        void updateDamping(int index, float damping);
        void updateFriction(int index, float friction);




 




        // Functions that are typically used only for statistics, diagnostics and debugging
        // --------------------------------------------------------------------------------

        /** Get the loop time inside the device. Typically around 0.05ms.
        * @returns {float}, the loop time in ms.
        */
        float getCurrentDeltaT();

        /** Get a status word from the device. It is the most compact way to get information about amplifiers, calibration, battery status etc.
        * @returns {int} which needs to be bitwise AND:ed to extract different information, see manual.
        */
        int getStatus();

        /** Get the board temperature from the device. This is only necessary for diagnostics. 
        * @returns {float} the amplifier board temperature in degrees Celcius.
        */
        float getBoardTemp();

        /** Get the device battery voltage. This is only necessary for diagnostics.
        * @returns {float} the voltage. Fully charged = 16.8V, battery exhausted below 12.0V.
        */
        float getBatteryVoltage();

        /** Get the USB draw set charging current (the batterys are charged using CC-CV). This is only necessary for diagnostics.
        * @returns {float} the charging set current (Amps).
        */
        float getChargingCurrent();

        /** Get the raw encoder values. This is an alternative (non-preferred) way to get angles and length. 
        * The raw encoder values can be used in conjunction with the getEncoderScalingValues() to convert to mm and radians.
        * @returns {vec4i} the encoder values in Rot, Pitch, Z and Yaw respectively.
        */
        sofa::type::fixed_array<int, 4> getRawEncoderValues(); // in counts

        /** Get the PWM output to the motors. 
        * @returns {vec4i} the PWM output in Rot, Pitch, Z and Yaw respectively.
        */
        sofa::type::fixed_array<float, 4> getLastPWM();

        /** Get a simulated part (structure) temperature, e.g. a motor winding temperature. The parts are listed in the enums PortThermalSimPart 
        * Can be used for supervision. The device will take necessary actions to reduce the outputs if the temperatures are too high.
        * @returns {float} the temperature in degrees Celcius.
        */        
        float getPartTemperature(int part);

        // Will send 0 torque and force values to the device.
        void releaseForce();

        // Will output to cout a resport of the status of the device, such as temperatures, battery voltage etc.
        void printStatus() override;

    protected:
        /// Internal method to setup how many return values each command is expecting and how to scale outgoing and incoming data.
        void setupNumReturnVals() override;
        /// Internal method to setup which data from the device to subscribe to, and how often.
        void setupCmdLists() override;

        int reserveNextPrimitiveIndex();
        void appendPrimitive(int index, int type, int active,
            sofa::type::fixed_array<float, 3> p0,
            sofa::type::fixed_array<float, 3> v0,
            sofa::type::fixed_array<float, 3> n,
            float q, float r, float s, float t, float stiffness, float friction, float damping);
        void updatePrimitiveProperty(int index, int prop, float value);
        void updatePrimitiveProperty(int index, int prop, sofa::type::fixed_array<float, 3> vec);
    private:

        bool primitive_index_used[MAX_NUM_PRIMITIVES];
        // This enum is a list of all commands. The same list exists in the device.
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

        // Enums for the thermal simulation in the device. The motor winding temperatures are the most interesting.
        enum PortThermalSimPart {
            RMotorWinding = 0, PMotorWinding, ZMotorWinding, YMotorWinding,
            HeatSink, RStruct, PStruct, ZStruct, YStruct, Cover, CoolingAir, Ambient, NumParts
        };

        enum CoPropEnum {
            CO_PROP_P0,
            CO_PROP_V0,
            CO_PROP_N,
            CO_PROP_Q,
            CO_PROP_R,
            CO_PROP_S,
            CO_PROP_T,
            CO_PROP_STIFFNESS,
            CO_PROP_FRICTION,
            CO_PROP_DAMPING,
        };

        enum CoType {
            CO_NONE= 0,
            CO_PLANE,
            CO_SPHERE,
            CO_SPRING,
            CO_STICKY_PLANE,
            CO_CYLINDER,
            CO_TORUS
        };
    };
} // namespace sofa::HapticAvatar
