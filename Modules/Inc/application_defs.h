#ifndef APPLICATION_DEFS_H_
#define APPLICATION_DEFS_H_

#include "robot.h"
#include "sensors_common.h"
#include "stdint.h"

#define ROBOT_APP_ID    0x10u
#define PARAM_APP_ID    0x11u
#define MEMORY_APP_ID   0x12u
#define SENSOR_APP_ID   0x13u
#define HK_APP_ID       0x14u
#define LOGIC_APP_ID    0x20u
#define GENERAL_APP_ID    0x19u

typedef enum{
    UNDEF = 0,
    START_ROB,
    STOP_ROB,
    RUN_FOR_TIME,
    RUN_FOR_DIST,
    RUN_TO_POINT,
    ROTATE,
    WAIT_TIME,
    MANUAL_START,
    MANUAL_STOP,
    AUTOPATH_START,
    AUTOPATH_STOP,
    SAFE_RETURN,
    RESET_POS
} robcommand_type_t;

typedef enum{
    INIT,
    CALIBRATE,
    SET_MODE,
    GET_ROLL,
    GET_PITCH,
    GET_YAW,
    GET_HEADING,
    GET_TEMP,
    DEINIT
}sens_command_type_t;

typedef enum{
    IF_EQUAL_THEN,
    IF_NOT_EQUAL_THEN,
    IF_LESS_THEN,
    IF_BIGGER_THEN,
    VAR_ASSIGN_VALUE,
    LOOP,
    VAR_ARITH_ADD,
    VAR_ARITH_SUB
}logic_command_type;

#define MAX_CUSTOM_VARS (10)

typedef enum{
    EMPTY_OP,
    POS_X,
    POS_Y,
    POS_Z,
    ROLL,
    PITCH,
    YAW,
    ROB_MODE,
    TEMP,
    LEFT_WHEEL_SPEED,
    RIGHT_WHEEL_SPEED,
    VAR0,
    VAR1,
    VAR2,
    VAR3,
    VAR4,
    VAR5,
    VAR6,
    VAR7,
    VAR8,
    VAR9,
}logic_operand_type;

typedef enum{
    LA_STOP_ROB,
    LA_RESET_ROB_POS,
    LA_RETURN_TO_PREV_POS,
    LA_RETURN_TO_POS_ZERO,
    LA_RESET_CMD_QUEUE,
    LA_RESET_TRACE_QUEUE,
    LA_REINIT_IMU,
    LA_STOP_IMU,
    LA_RECALIBRATE_IMU,
    LA_START_MONITORINGS,
    LA_STOP_MONITORINGS,
    LA_SYSTEM_SHUTDOWN
}logic_action_type;

typedef enum{
    GA_START_MONITORINGS,
    GA_STOP_MONITORINGS,
    GA_RESET_CMD_QUEUE,
    GA_RESET_TRACE_QUEUE,
    GA_SET_PARAM,
}general_action_type;

typedef union{
    struct control_bits{
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t left : 1;
        uint8_t down : 1;
    } bit;
    uint8_t control_byte;
} manual_ctrl_command_t;

typedef struct{
    Mobile_Platform_t* robot;
    robcommand_type_t type;
    uint16_t time;
    int16_t speed;
    rob_coord_t point;
    int32_t distance;
    int16_t angle;
    manual_ctrl_command_t man_ctrl;
}robot_command_t;

typedef struct{
    sensors_id_t sensor;
    sens_command_type_t type;
    uint8_t mode;
    uint16_t calibration_steps;
}sensor_command_t;

typedef struct{
    logic_command_type type;
    logic_operand_type operand;
    int32_t value;
    logic_action_type action;
    Mobile_Platform_t* robot;
}logic_command_t;

typedef struct{
    general_action_type type;
    uint16_t param_id;
    int32_t value;
}general_command_t;

typedef union payload
{
    robot_command_t robcmd;
    sensor_command_t senscmd;
    logic_command_t logiccmd;
    general_command_t gencmd;
}payload_t;


#endif