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

#include <SofaHapticAvatar/HapticAvatar_DriverPort.h>
#include <sofa/helper/logging/Messaging.h>

namespace sofa::HapticAvatar
{

using namespace HapticAvatar;


///////////////////////////////////////////////////////////////
/////      Methods for specific device communication      /////
///////////////////////////////////////////////////////////////

HapticAvatar_DriverPort::HapticAvatar_DriverPort(const std::string& portName)
    : HapticAvatar_DriverBase(portName)
{
    setupNumReturnVals();  // needs to be implemented in each device driver
    setupCmdLists();   // needs to be implemented in each device driver

    device_type = 1;

}

void HapticAvatar_DriverPort::setupNumReturnVals()
{
    // num_return_vals = new int[CmdPort::ALWAYS_LAST];

    num_return_vals[(int)CmdPort::RESET] = 1;
    num_return_vals[(int)CmdPort::GET_DEVICE_TYPE] = 1;
    num_return_vals[(int)CmdPort::GET_ANGLES_AND_LENGTH] = 4;
    num_return_vals[(int)CmdPort::GET_TOOL_ID] = 1;
    num_return_vals[(int)CmdPort::GET_CURRENT_DELTA_T] = 1;
    num_return_vals[(int)CmdPort::GET_STATUS] = 1;
    num_return_vals[(int)CmdPort::SET_MOTOR_FORCE_AND_TORQUES] = 0;
    num_return_vals[(int)CmdPort::SET_TIP_FORCE_AND_ROT_TORQUE] = 0;
    num_return_vals[(int)CmdPort::SET_YAW_PITCH_ZERO_ANG] = 0;
    num_return_vals[(int)CmdPort::SET_LED_BLINK_MODE] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_ACTIVE] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_P0] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_V0] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_N] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_Q] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_R] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_S] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_T] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_STIFFNESS] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_DAMPING] = 0;
    num_return_vals[(int)CmdPort::SET_COLLISION_OBJECT_FRICTION] = 0;
    num_return_vals[(int)CmdPort::GET_LAST_COLLISION_FORCE] = 3;
    num_return_vals[(int)CmdPort::GET_LAST_PWM] = 4;
    num_return_vals[(int)CmdPort::GET_LAST_COLLISION_DATA] = 1;
    num_return_vals[(int)CmdPort::SET_TOOL_JAW_OPENING_ANGLE] = 0;
    num_return_vals[(int)CmdPort::GET_TOOL_JAW_TORQUE] = 1;
    num_return_vals[(int)CmdPort::SET_TOOL_DATA] = 0;
    num_return_vals[(int)CmdPort::GET_TOOL_INSERTED] = 1;
    num_return_vals[(int)CmdPort::GET_TOOL_TIP_VELOCITY] = 3;
    num_return_vals[(int)CmdPort::GET_TOOL_TIP_POSITION] = 3;
    num_return_vals[(int)CmdPort::GET_TOOL_DIRECTION] = 3;
    num_return_vals[(int)CmdPort::GET_RAW_ENCODER_VALUES] = 4;
    num_return_vals[(int)CmdPort::GET_ENCODER_SCALING_VALUES] = 4;
    num_return_vals[(int)CmdPort::GET_MOTOR_SCALING_VALUES] = 4;
    num_return_vals[(int)CmdPort::SET_MANUAL_PWM] = 0;
    num_return_vals[(int)CmdPort::GET_BOARD_TEMP] = 1;
    num_return_vals[(int)CmdPort::GET_BATTERY_VOLTAGE] = 1;
    num_return_vals[(int)CmdPort::GET_CALIBRATION_STATUS] = 3;
    num_return_vals[(int)CmdPort::GET_AMPLIFIERS_STATUS] = 4;
    num_return_vals[(int)CmdPort::GET_HALL_STATES] = 2;
    num_return_vals[(int)CmdPort::SET_POWER_ON_MANUAL] = 0;
    num_return_vals[(int)CmdPort::SET_FAN_ON_MANUAL] = 0;
    num_return_vals[(int)CmdPort::SET_FF_ENABLE] = 0;
    num_return_vals[(int)CmdPort::GET_SERIAL_NUM] = 1;
    num_return_vals[(int)CmdPort::GET_BUILD_DATE] = 1;
    num_return_vals[(int)CmdPort::SET_CHARGE_ENABLE] = 0;
    num_return_vals[(int)CmdPort::GET_TIP_LENGTH] = 1;
    num_return_vals[(int)CmdPort::GET_PART_TEMPERATURES] = 12;
    num_return_vals[(int)CmdPort::SET_MAX_USB_CHARGE_CURRENT] = 0;
    num_return_vals[(int)CmdPort::GET_USB_CHARGING_CURRENT] = 1;
    num_return_vals[(int)CmdPort::SET_DEADBAND_PWM_WIDTH] = 1;

    // Set all scale factor to 1.0 to begin with ...
    for (int i = 0; i < (int)CmdPort::ALWAYS_LAST; i++)
        scale_factor[i] = 1.0f;

    // ... and now change those who are not 1.0
    scale_factor[(int)CmdPort::GET_ANGLES_AND_LENGTH] = 10000.0f;
    scale_factor[(int)CmdPort::GET_CURRENT_DELTA_T] = 10000.0f;
    scale_factor[(int)CmdPort::SET_MOTOR_FORCE_AND_TORQUES] = 10000.0f;
    scale_factor[(int)CmdPort::SET_TIP_FORCE_AND_ROT_TORQUE] = 10000.0f;
    scale_factor[(int)CmdPort::SET_YAW_PITCH_ZERO_ANG] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_P0] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_V0] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_N] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_Q] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_R] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_S] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_T] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_STIFFNESS] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_DAMPING] = 10000.0f;
    scale_factor[(int)CmdPort::SET_COLLISION_OBJECT_FRICTION] = 10000.0f;
    scale_factor[(int)CmdPort::GET_LAST_COLLISION_FORCE] = 10000.0f;
    scale_factor[(int)CmdPort::GET_LAST_COLLISION_DATA] = 10000.0f;
    scale_factor[(int)CmdPort::SET_TOOL_JAW_OPENING_ANGLE] = 10000.0f;
    scale_factor[(int)CmdPort::GET_TOOL_JAW_TORQUE] = 10000.0f;
    scale_factor[(int)CmdPort::SET_TOOL_DATA] = 10000.0f;
    scale_factor[(int)CmdPort::GET_TOOL_TIP_VELOCITY] = 10000.0f;
    scale_factor[(int)CmdPort::GET_TOOL_TIP_POSITION] = 10000.0f;
    scale_factor[(int)CmdPort::GET_TOOL_DIRECTION] = 10000.0f;
    scale_factor[(int)CmdPort::GET_ENCODER_SCALING_VALUES] = 10000.0f;
    scale_factor[(int)CmdPort::GET_MOTOR_SCALING_VALUES] = 10000.0f;
    scale_factor[(int)CmdPort::GET_BOARD_TEMP] = 10000.0f;
    scale_factor[(int)CmdPort::GET_BATTERY_VOLTAGE] = 10000.0f;
    scale_factor[(int)CmdPort::GET_AMPLIFIERS_STATUS] = 10000.0f;
    scale_factor[(int)CmdPort::GET_TIP_LENGTH] = 10000.0f;
    scale_factor[(int)CmdPort::GET_PART_TEMPERATURES] = 10.0f;
    scale_factor[(int)CmdPort::GET_USB_CHARGING_CURRENT] = 10000.0f;

    device_num_cmds = (int)CmdPort::ALWAYS_LAST;
}

void HapticAvatar_DriverPort::setupCmdLists()
{
    // Setup the data you want to subscribe to from the port device here. Subscription commands can only be of type GET_... without input arguments.
    // It is good practice to use prime numbers to avoid that all commands are sent at once.
    subscribeTo((int)CmdPort::GET_ANGLES_AND_LENGTH, 1);
    subscribeTo((int)CmdPort::GET_TOOL_INSERTED, 11);
    subscribeTo((int)CmdPort::GET_TOOL_ID, 13);
    subscribeTo((int)CmdPort::GET_CURRENT_DELTA_T, 17);
    subscribeTo((int)CmdPort::GET_LAST_PWM, 19);
    subscribeTo((int)CmdPort::GET_BOARD_TEMP, 10007);
    subscribeTo((int)CmdPort::GET_BATTERY_VOLTAGE, 10009);
    subscribeTo((int)CmdPort::GET_STATUS, 1009);
    subscribeTo((int)CmdPort::GET_CALIBRATION_STATUS, 1013);
    subscribeTo((int)CmdPort::GET_USB_CHARGING_CURRENT, 10037);
    subscribeTo((int)CmdPort::GET_PART_TEMPERATURES, 10039);
}


sofa::type::fixed_array<float, 4> HapticAvatar_DriverPort::getAnglesAndLength()
{
    return getFloat4((int)CmdPort::GET_ANGLES_AND_LENGTH);
}

sofa::type::fixed_array<float, 4> HapticAvatar_DriverPort::getLastPWM()
{
    return getFloat4((int)CmdPort::GET_LAST_PWM);
}

int HapticAvatar_DriverPort::getToolID()
{
    return getInt((int)CmdPort::GET_TOOL_ID);
}

bool HapticAvatar_DriverPort::getToolInserted()
{
    return (bool) getInt((int)CmdPort::GET_TOOL_INSERTED);
}

float HapticAvatar_DriverPort::getBoardTemp() 
{ 
    return getFloat((int)CmdPort::GET_BOARD_TEMP); 
}

float HapticAvatar_DriverPort::getBatteryVoltage()
{
    return getFloat((int)CmdPort::GET_BATTERY_VOLTAGE);
}

sofa::type::fixed_array<int, 4> HapticAvatar_DriverPort::getRawEncoderValues()
{
    return getInt4((int)CmdPort::GET_RAW_ENCODER_VALUES);
}

sofa::type::fixed_array<float, 4> HapticAvatar_DriverPort::getEncoderScalingValues()
{
    return getFloat4((int)CmdPort::GET_ENCODER_SCALING_VALUES);
}


bool HapticAvatar_DriverPort::getYawPitchCalibrated()
{
    return ((bool)result_table[(int)CmdPort::GET_CALIBRATION_STATUS][0]) && ((bool)result_table[(int)CmdPort::GET_CALIBRATION_STATUS][1]);
}

sofa::type::fixed_array<float, 4> HapticAvatar_DriverPort::getMotorScalingValues()
{
    return getFloat4((int)CmdPort::GET_MOTOR_SCALING_VALUES);
}

int HapticAvatar_DriverPort::getStatus()
{
    return getInt((int)CmdPort::GET_STATUS);
}

void HapticAvatar_DriverPort::setMotorForceAndTorques(float rot, float pitch, float z, float yaw)
{
    appendFloat((int)CmdPort::SET_MOTOR_FORCE_AND_TORQUES, rot/1000, pitch/1000, z, yaw/1000);
}

//void HapticAvatar_DriverPort::setTipForceAndRotTorque(int rot, int pitch, int z, int yaw)
//{
//    appendFloat4((int)CmdPort::SET_TIP_FORCE_AND_ROT_TORQUE, values);
//}

void HapticAvatar_DriverPort::setDeadBandPWMWidth(float rot, float pitch, float z, float yaw)
{
    appendFloat((int)CmdPort::SET_DEADBAND_PWM_WIDTH, rot, pitch, z, yaw);
}

void HapticAvatar_DriverPort::setForceFeedbackEnable(bool on)
{
    appendInt((int)CmdPort::SET_FF_ENABLE, (int)on);
}

float  HapticAvatar_DriverPort::getCurrentDeltaT()
{
    return getFloat((int)CmdPort::GET_CURRENT_DELTA_T);
}

int cptF = 0;


void HapticAvatar_DriverPort::releaseForce()
{
    setManualPWM(0, 0, 0, 0);
    //sendCommandToDevice(CmdPort::SET_MANUAL_PWM, "0 0 0 0", nullptr);
}


void HapticAvatar_DriverPort::setManualPWM(int rot, int pitch, int z, int yaw)
{
    appendFloat((int)CmdPort::SET_MANUAL_PWM, rot, pitch, z, yaw);
}


int HapticAvatar_DriverPort::getSerialNumber()
{
    return getInt((int)CmdPort::GET_SERIAL_NUM);
}
float HapticAvatar_DriverPort::getPartTemperature(int part)
{
    return getFloat((int)CmdPort::GET_PART_TEMPERATURES, part);
}

float HapticAvatar_DriverPort::getChargingCurrent()
{
    return getFloat((int)CmdPort::GET_USB_CHARGING_CURRENT);
}

void HapticAvatar_DriverPort::printStatus()
{
    std::cout << "Status for Port device S/N " << getSerialNumber() << " at " << getPortName() << std::endl;
    std::cout << "--------------------------------------------" << std::endl;

    std::cout << "  Amplifier board temperature " << getBoardTemp() << "°C" << std::endl;
    std::cout << "  Battery voltage " << getBatteryVoltage() << "V" << std::endl;
    std::cout << "  Charging current " << getChargingCurrent() << "A  (USB draw)" << std::endl;
    unsigned int status = (unsigned int) getStatus();
    std::cout << "  Yaw calibrated: ";
    if ((status & 0x08) > 0)
        std::cout << "Yes     ";
    else
        std::cout << "No      ";
    std::cout << "Pitch calibrated: ";
    if ((status & 0x02) > 0)
        std::cout << "Yes ";
    else
        std::cout << "No  ";
    std::cout << std::endl;
    if ((status & 0x80) > 0)
        std::cout << "  Yaw Amplifier temperature warning!" << std::endl;
    if ((status & 0x20) > 0)
        std::cout << "  Pitch Amplifier temperature warning!" << std::endl;
    if ((status & 0x10) > 0)
        std::cout << "  Z or Rot Amplifier temperature warning!" << std::endl;
    if ((status & 0x800) > 0)
        std::cout << "  Yaw Amplifier undervoltage warning!" << std::endl;
    if ((status & 0x200) > 0)
        std::cout << "  Pitch Amplifier undervoltage warning!" << std::endl;
    if ((status & 0x100) > 0)
        std::cout << "  Z or Rot Amplifier undervoltage warning!" << std::endl;

    float temp = getPartTemperature((int)PortThermalSimPart::YMotorWinding);
    std::cout << "  Yaw motor winding temperature " << temp << " °C";
    if (temp > 135)
        std::cout << " HIGH, output reduced!";
    std::cout << std::endl;
    temp = getPartTemperature((int)PortThermalSimPart::PMotorWinding);
    std::cout << "  Pitch motor winding temperature " << temp << " °C";
    if (temp > 135)
        std::cout << " HIGH, output reduced!";
    std::cout << std::endl;
    temp = getPartTemperature((int)PortThermalSimPart::ZMotorWinding);
    std::cout << "  Z motor winding temperature " << temp << " °C";
    if (temp > 105)
        std::cout << " HIGH, output reduced!";
    std::cout << std::endl;
    temp = getPartTemperature((int)PortThermalSimPart::RMotorWinding);
    std::cout << "  Rot motor winding temperature " << temp << " °C";
    if (temp > 85)
        std::cout << " HIGH, output reduced!";
    std::cout << std::endl << std::endl;

}


void HapticAvatar_DriverPort::setInstrumentData(float shaft_diameter, float jaw1_diameter, float jaw2_diameter, float jaw_length)
{
    appendFloat((int)CmdPort::SET_TOOL_DATA, shaft_diameter, jaw1_diameter, jaw2_diameter, jaw_length);
}

void HapticAvatar_DriverPort::setJawOpeningAngle(float ang)
{
    appendFloat((int)CmdPort::SET_TOOL_JAW_OPENING_ANGLE, ang);
}
float HapticAvatar_DriverPort::getJawTorque()
{
    return getFloat((int)CmdPort::GET_TOOL_JAW_TORQUE);
}

int HapticAvatar_DriverPort::reserveNextPrimitiveIndex()
{
    int retval = -1;
    for (int i = 0; i < MAX_NUM_PRIMITIVES; i++)
    {
        if (!primitive_index_used[i]) {
            primitive_index_used[i] = true;
            retval = i;
            break;
        }
    }
    return retval;
}
int HapticAvatar_DriverPort::addSphere(sofa::type::fixed_array<float, 3> pos, float radius, float stiffness, float damping, float friction)
{
    int index = reserveNextPrimitiveIndex();
    if (index >= 0) {
        sofa::type::fixed_array<float, 3> v0 = { 1, 0, 0 };
        sofa::type::fixed_array<float, 3> n = { 0, 0, 1 };
        appendPrimitive(index, (int)CoType::CO_SPHERE, true, pos, v0, n, 0, radius, 0, 0, stiffness, friction, damping);
    }
    return index;
}

int HapticAvatar_DriverPort::addCapsule(sofa::type::fixed_array<float, 3> pos, sofa::type::fixed_array<float, 3> ori, float radius, float length, float stiffness, float damping, float friction)
{
    int index = reserveNextPrimitiveIndex();
    if (index >= 0) {
        sofa::type::fixed_array<float, 3> n = { 0, 0, 1 };
        appendPrimitive(index, (int)CoType::CO_CYLINDER, true, pos, ori, n, 0, radius, length, 0, stiffness, friction, damping);
    }
    return index;
}

int HapticAvatar_DriverPort::addTorus(sofa::type::fixed_array<float, 3> pos, sofa::type::fixed_array<float, 3> ori, float major_radius, float minor_radius, float stiffness, float damping, float friction)
{
    int index = reserveNextPrimitiveIndex();
    if (index >= 0) {
        sofa::type::fixed_array<float, 3> n = { 0, 0, 1 };
        appendPrimitive(index, (int)CoType::CO_CYLINDER, true, pos, ori, n, 0, major_radius, minor_radius, 0, stiffness, friction, damping);
    }
    return index;
}

void HapticAvatar_DriverPort::deletePrimitive(int index)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendInt((CmdPort::SET_COLLISION_OBJECT_ACTIVE), index, 0);
        primitive_index_used[index] = false;
    }
}
void HapticAvatar_DriverPort::deleteAllPrimitives()
{
    for (int i = 0; i < MAX_NUM_PRIMITIVES; i++) {
        deletePrimitive(i);
    }
}

void HapticAvatar_DriverPort::setActive(int index, bool active)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendInt((CmdPort::SET_COLLISION_OBJECT_ACTIVE), index, (int) active);
    }
}
void HapticAvatar_DriverPort::updatePosition(int index, sofa::type::fixed_array<float, 3> new_pos)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_P0), index, new_pos);
    }
}
void HapticAvatar_DriverPort::updateOrientation(int index, sofa::type::fixed_array<float, 3> new_ori)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_V0), index, new_ori);
    }
}
void HapticAvatar_DriverPort::updateRadius1(int index, float radius)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_R), index, radius);
    }
}
void HapticAvatar_DriverPort::updateRadius2(int index, float radius)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_Q), index, radius);
    }
}
void HapticAvatar_DriverPort::updateLength(int index, float length)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_T), index, length);
    }
}
void HapticAvatar_DriverPort::updateStiffness(int index, float stiffness)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_STIFFNESS), index, stiffness);
    }
}
void HapticAvatar_DriverPort::updateDamping(int index, float damping)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_DAMPING), index, damping);
    }
}
void HapticAvatar_DriverPort::updateFriction(int index, float friction)
{
    if (index >= 0 && index < MAX_NUM_PRIMITIVES) {
        appendIntFloat((CmdPort::SET_COLLISION_OBJECT_FRICTION), index, friction);
    }
}

void HapticAvatar_DriverPort::appendPrimitive(int index, int type, int active,
    sofa::type::fixed_array<float, 3> p0,
    sofa::type::fixed_array<float, 3> v0,
    sofa::type::fixed_array<float, 3> n,
    float q, float r, float s, float t, float stiffness, float friction, float damping)
{
    int cmd = (int)CmdPort::SET_COLLISION_OBJECT;
    std::string arguments;
    arguments = std::to_string(index) + " " +
        std::to_string(type) + " " +
        std::to_string(active) + " " +
        std::to_string(int(p0[0] * scale_factor[cmd])) + " " +
        std::to_string(int(p0[1] * scale_factor[cmd])) + " " +
        std::to_string(int(p0[2] * scale_factor[cmd])) + " " +
        std::to_string(int(v0[0] * scale_factor[cmd])) + " " +
        std::to_string(int(v0[1] * scale_factor[cmd])) + " " +
        std::to_string(int(v0[2] * scale_factor[cmd])) + " " +
        std::to_string(int(n[0] * scale_factor[cmd])) + " " +
        std::to_string(int(n[1] * scale_factor[cmd])) + " " +
        std::to_string(int(n[2] * scale_factor[cmd])) + " " +
        std::to_string(int(q * scale_factor[cmd])) + " " +
        std::to_string(int(r * scale_factor[cmd])) + " " +
        std::to_string(int(s * scale_factor[cmd])) + " " +
        std::to_string(int(t * scale_factor[cmd])) + " " +
        std::to_string(int(stiffness * scale_factor[cmd])) + " " +
        std::to_string(int(friction * scale_factor[cmd])) + " " +
        std::to_string(int(damping * scale_factor[cmd])) + " ";

    appendCmd((int)CmdPort::SET_COLLISION_OBJECT, arguments.c_str());
}

} // namespace sofa::HapticAvatar
