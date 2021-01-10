/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#pragma once

namespace sofa::HapticAvatar
{

// device communication
enum CmdTool
{
    RESET = 0,
    GET_IDENTITY,
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
    SET_FF_ENABLE
};


enum CmdIBox 
{
    RESET_IBOX = 0,
    GET_IBOX_DEVICE_TYPE,
    GET_IBOX_OPENING_VALUES,
    GET_IBOX_HANDLE_IDS,
    GET_IBOX_PEDAL_STATES,
    SET_IBOX_ALL_FORCES,
    SET_IBOX_CHAN_FORCE,
    GET_IBOX_STATUS,
    GET_IBOX_CALIBRATION_STATUS,
    GET_IBOX_MOTOR_BOARD_STATUS,
    GET_IBOX_BATTERY_VOLTAGE,
    GET_IBOX_BOARD_TEMP,
    SET_IBOX_MANUAL_PWM,
    SET_IBOX_POWER_ON_MANUAL,
    SET_IBOX_FAN_ON_MANUAL,
    GET_IBOX_LAST_PWM,
    SET_IBOX_FF_ENABLE,
    GET_IBOX_CURRENT_DELTA_T,
    SET_IBOX_LOOP_GAIN,
    GET_IBOX_OPTO_FORCE,
    SET_IBOX_ZERO_FORCE,
    GET_IBOX_POS_VOLTAGE,
    GET_IBOX_HANDLE_IDS_REAL,
    GET_IBOX_BUILD_DATE,
    GET_IBOX_SERIAL_NUM,
    SET_IBOX_CHARGE_ENABLE,
    ALWAYS_LAST
};


enum Dof 
{ 
    ROT = 0, 
    PITCH, 
    Z, 
    YAW 
};


#define OUTGOING_DATA_LEN 64
#define INCOMING_DATA_LEN 512
#define NBJOINT 6
#define ARDUINO_WAIT_TIME 2000

#define RAIL_DISTANCE 50
#define EULER_TO_RAD 0.0174533

} // namespace sofa::HapticAvatar
