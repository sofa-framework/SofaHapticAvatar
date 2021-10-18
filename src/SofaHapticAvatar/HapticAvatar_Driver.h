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
#include <SofaHapticAvatar/HapticAvatar_Defines.h>
#include <sofa/defaulttype/Vec.h>
#include <string>

namespace sofa::HapticAvatar
{

/**
* HapticAvatar driver
*/
class SOFA_HAPTICAVATAR_API HapticAvatar_BaseDriver
{
public:
    HapticAvatar_BaseDriver(const std::string& portName);

    virtual ~HapticAvatar_BaseDriver();

    bool IsConnected() { return m_connected; }


    /** Reset encoders, motor outputs, calibration flags, collision objects. 
    * Command: RESET
    * @param {int} mode: specify what to reset. See doc. 
    */
    int resetDevice(int mode = 15);

    /** Ask for the identity and build version of the firmware in the device.
    * Command: GET_IDENTITY
    */
    std::string getIdentity();

    /** To get an identification number of which tool is inserted (if any). The identification number is related to the length of the pin at the tip of the tool. 
    * Command: GET_TOOL_ID
    */
    int getToolID();

    /** To get the device status for calibration, amplifiers, hall effect sensors, battery, fan, power and board temperature. This is mainly for diagnostics and error detection. 
    * Command: GET_STATUS
    */
    int getDeviceStatus();



    /** Generic method which will format the command and send it to the device using HapticAvatar::Cmd and list of arguments given as input. Result will be stored in input char* result if not null.
    * @param {HapticAvatar::Cmd} command: the command enum to be sent.
    * @param {string} arguments: already formatted list of arguments to be sent with the command.
    * @param {char *} result: if not null, response will be asked to device and stored in this char*.
    * @returns {bool} true if command success otherwise false before getting result.
    */
    bool sendCommandToDevice(int commandId, const std::string& arguments, char *result);

    
    /// Will convert a char* array response into std::string while removing end of line and space at end.
    std::string convertSingleData(char *buffer, bool forceRemoveEoL = false);

protected:
    /// Internal method to connect to device
    void connectDevice();

    /** Internal method to really to get response from the device. Will be looping while calling @sa ReadDataImplLooping with a security of 10k loop.
    * @param {char *} buffer: array to store the response.
    * @param {bool} do_flush: to flush after getting response.
    */
    int getDataImpl(char *buffer, bool do_flush);
    
    /** Internal low level method to really do the job of getting a response from the device
    * @param {char *} buffer: array to store the response.
    * @param {uint} nbChar: size of the command array
    * @param {int *} queue: queue size to be read.
    * @param {bool} do_flush: to flush after getting response.
    */
    int ReadDataImpl(char *buffer, unsigned int nbChar, int *queue, bool do_flush);

    /** Internal low level method to really do the job of sending a command to the device.
    * @param {char *} buffer: full command as an array.
    * @param {uint} nbChar: size of the command array
    */
    bool WriteDataImpl(char *buffer, unsigned int nbChar);

    
    /// Internal method to get the enum id for reset command. To be overwritten by child
    virtual int getResetCommandId() = 0;

    /// Internal method to get the enum id for identity command. To be overwritten by child
    virtual int getIdentityCommandId() = 0;

    /// Internal method to get the enum id for toolID command. To be overwritten by child
    virtual int getToolIDCommandId() = 0;

    /// Internal method to get the enum id for status command. To be overwritten by child
    virtual int getStatusCommandId() = 0;

private:
    //Connection status
    bool m_connected;

    //Serial comm handler
    HANDLE m_hSerial;
    //Get various information about the connection
    COMSTAT m_status;
    //Keep track of last error
    DWORD m_errors;

    // String name of the port (ex: COM3)
    std::string m_portName;
};



class SOFA_HAPTICAVATAR_API HapticAvatar_Driver : public HapticAvatar_BaseDriver
{
public:
    HapticAvatar_Driver(const std::string& portName);


    /** get the angles of the port (Yaw + Pitch) and the insertion length and rotation of the tool.
    * Command: GET_ANGLES_AND_LENGTH
    * @returns {vec4f} {Tool rotation angle, Pitch angle, Pitch angle, Yaw angle}
    */
    sofa::type::fixed_array<float, 4> getAngles_AndLength();

    /** Get the torque on the jaws around the jaw rotation pin.
    * Command: GET_TOOL_JAW_TORQUE
    * @returns {float} The sum of the torque on the jaws
    */
    float getJawTorque();

    /** Get the opening angle.
    * Command: SET_TOOL_JAW_OPENING_ANGLE
    * @returns {float} The opening angle: 0.0 means fully closed. 1.0 means fully opened
    */
    float getJawOpeningAngle();



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

    /** Get the most recent collision force sum. This is only related to collisions against primitives.
    * Command: GET_LAST_COLLISION_FORCE
    * @returns {vec3f} A vector (Fx, Fy, Fz) in the Device LCS
    */
    sofa::type::fixed_array<float, 3> getLastCollisionForce();



    /** Set the force and torque output per motor.
    * Command: SET_MOTOR_FORCE_AND_TORQUES
    * @param {vec4f} values: specify what to reset. See doc.
    */
    void setMotorForce_AndTorques(sofa::type::fixed_array<float, 4> values);

    /** Set a force vector on the tool tip plus the tool rotation torque. This is an alternative way to output force (compared to SET_MOTOR_FORCE_AND_TORQUES)
    * Command: SET_TIP_FORCE_AND_ROT_TORQUE
    * @param {vec3f} force: specify what to reset. See doc.
    * @param {float} RotTorque: specify what to reset. See doc.
    */
    void setTipForce_AndRotTorque(sofa::type::Vector3 force, float RotTorque);
    
    /** Will decompose a force vector in device coordinate system to compute torque and force to be sent to apply to the device. Using @sa writeRoughForce.
    * @param {vec3f} force: force vector to apply to the device in its coordinate space.
    */
    void setManualForceVector(sofa::type::Vector3 force, bool useManualPWM = false);


    void setTipForceVector(sofa::type::Vector3 force);

    // Will send 0 torque and force values to the device. Using SET_MANUAL_PWM.
    void releaseForce();

    // Will call SET_MANUAL_PWM: Set the force output manually per motor with raw PWM-values. 
    void setManual_PWM(float rotTorque, float pitchTorque, float zforce, float yawTorque);

    void setManual_Force_and_Torques(float rotTorque, float pitchTorque, float zforce, float yawTorque);

protected:
    /// Internal method to get the enum id for reset command. To be overwritten by child
    int getResetCommandId() override { return CmdTool::RESET; }

    /// Internal method to get the enum id for identity command. To be overwritten by child
    int getIdentityCommandId() override { return CmdTool::GET_IDENTITY; }

    /// Internal method to get the enum id for toolID command. To be overwritten by child
    int getToolIDCommandId() override { return CmdTool::GET_TOOL_ID; }

    /// Internal method to get the enum id for status command. To be overwritten by child
    int getStatusCommandId() override { return CmdTool::GET_STATUS; }
};




class SOFA_HAPTICAVATAR_API HapticAvatar_IboxDriver : public HapticAvatar_BaseDriver
{
public:
    HapticAvatar_IboxDriver(const std::string& portName);


    /**  TODO check if returning a Vec4 is the best to do
    */
    sofa::type::fixed_array<float, 4> getOpeningValues();

    void setHandleForces(float upperJawForce, float lowerJawForce);

	void setLoopGain(int loopGain);


protected:
    /// Internal method to get the enum id for reset command. To be overwritten by child
    int getResetCommandId() override { return CmdIBox::RESET_IBOX; }

    /// Internal method to get the enum id for identity command. To be overwritten by child
    int getIdentityCommandId() override { return CmdIBox::GET_IBOX_DEVICE_TYPE; }

    /// Internal method to get the enum id for toolID command. To be overwritten by child
    int getToolIDCommandId() override { return CmdIBox::GET_IBOX_HANDLE_IDS; }

    /// Internal method to get the enum id for status command. To be overwritten by child
    int getStatusCommandId() override { return CmdIBox::GET_IBOX_STATUS; }
};


} // namespace sofa::HapticAvatar
