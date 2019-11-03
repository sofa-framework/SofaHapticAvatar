/******************************************************************************
* License version                                                             *
*                                                                             *
* Authors:                                                                    *
* Contact information:                                                        *
******************************************************************************/
#ifndef SOFA_HAPTICAVATAR_HAPTICAVATARDEFINES_H
#define SOFA_HAPTICAVATAR_HAPTICAVATARDEFINES_H


namespace HapticAvatar
{

// device communication
enum Cmd
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



#define OUTGOING_DATA_LEN 64
#define INCOMING_DATA_LEN 512
#define NBJOINT 6
#define ARDUINO_WAIT_TIME 2000

#define RAIL_DISTANCE 50
#define EULER_TO_RAD 0.0174533




} // namespace HapticAvatar

#endif // SOFA_HAPTICAVATAR_HAPTICAVATARDEFINES_H
