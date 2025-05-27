#include "logic_executor.h"
#include "application_defs.h"
#include "robot.h"
#include "sensors_common.h"
#include "robot_app.h"
#include "monitoring.h"

#define LOGIC_COMPARE(status, op1, cmp, op2) \
                    if (cmp == "=="){status = (op1 == op2);} \
                    else if (cmp == ">"){status = (op1 > op2);} \
                    else if (cmp == "<"){status = (op1 < op2);} \
                    else if (cmp == "!="){status = (op1 != op2);} 


char* signs_map[4] = 
{ 
    [IF_EQUAL_THEN] = "==",
    [IF_NOT_EQUAL_THEN] = "!=",
    [IF_LESS_THEN] = "<",
    [IF_BIGGER_THEN] = ">",
};


static int32_t m_vars[MAX_CUSTOM_VARS];

static uint8_t var_index(logic_operand_type op){
    uint8_t index = 0;
    if (op >= VAR0 && op <= VAR9){
        index = (uint8_t)(op - VAR0);
    }

    return index;
}


static bool if_statement_handler(logic_payload_t* data, logic_command_type type){

    logic_operand_type op = data->logiccmd.operand;
    int32_t value = data->logiccmd.value;

    bool status = false;
    switch (op)
    {
    case POS_X:
    {
        rob_coord_t coord = Robot_GetCoord(data->logiccmd.robot);
        LOGIC_COMPARE(status, coord.x_pos, signs_map[type], value);
        break;
    }
    
    case POS_Y:
    {
        rob_coord_t coord = Robot_GetCoord(data->logiccmd.robot);
        LOGIC_COMPARE(status, coord.y_pos, signs_map[type], value);
        break;
    }
    
    case POS_Z:
    {
        rob_coord_t coord = Robot_GetCoord(data->logiccmd.robot);
        LOGIC_COMPARE(status, coord.z_pos, signs_map[type], value);
        break;
    }

    case ROLL:
    {
        euler_angles_t angle;
        Sensor_GetValue(IMU, &angle);
        LOGIC_COMPARE(status, (int32_t)angle.roll, signs_map[type], value);
        break;
    }
    case PITCH:
    {
        euler_angles_t angle;
        Sensor_GetValue(IMU, &angle);
        LOGIC_COMPARE(status, (int32_t)angle.pitch, signs_map[type], value);
        break;
    }
    case YAW:
    {
        euler_angles_t angle;
        Sensor_GetValue(IMU, &angle);
        LOGIC_COMPARE(status, (int32_t)angle.yaw, signs_map[type], value);
        break;
    }
    case LEFT_WHEEL_SPEED:
    {
        int32_t speed = Robot_GetWheelSpeed(data->logiccmd.robot, LEFT);
        LOGIC_COMPARE(status, speed, signs_map[type], value);
        break;
    }
    case RIGHT_WHEEL_SPEED:
    {
        int32_t speed = Robot_GetWheelSpeed(data->logiccmd.robot, RIGHT);
        LOGIC_COMPARE(status, speed, signs_map[type], value);
        break;
    }

    case VAR0:
    case VAR1:
    case VAR2:
    case VAR3:
    case VAR4:
    case VAR5:
    case VAR6:
    case VAR7:
    case VAR8:
    case VAR9:
    {
        const uint8_t index = var_index(op);
        LOGIC_COMPARE(status, m_vars[index], signs_map[type], value);
        break;
    }
    default:
        break;
    }

    return status;
}

bool LogicExecutor_Ready(logic_payload_t* data){

    logic_operand_type op = data->logiccmd.operand;

    bool ready = false;
    switch (op)
    {
    case POS_X:
    case POS_Y:
    case POS_Z:
        ready = (Robot_Status(data->logiccmd.robot) != ROB_OFF);
        break;
    case ROLL:
    case PITCH:
    case YAW:
        ready = (Sensor_GetState(IMU) == SENSOR_WORKING);
        break;
    default:
        ready = true;
        break;
    }

    return ready;
    
}

void LogicExecutor_Dispatch(logic_payload_t* data, status_notif_cb cb){

    logic_command_type type = data->logiccmd.type;

    bool execute_action = false;

    switch (type)
    {
    case IF_EQUAL_THEN:
    case IF_BIGGER_THEN:
    case IF_LESS_THEN:
    case IF_NOT_EQUAL_THEN:
        execute_action = if_statement_handler(data, type);
        break;

    case VAR_ASSIGN_VALUE:
    {
        const uint8_t index = var_index(data->logiccmd.operand);
        m_vars[index] = data->logiccmd.value;
        break;
    }
    case VAR_ARITH_ADD:
    {
        const uint8_t index = var_index(data->logiccmd.operand);
        m_vars[index] += data->logiccmd.value;
        break;
    }
    case VAR_ARITH_SUB:
    {
        const uint8_t index = var_index(data->logiccmd.operand);
        m_vars[index] -= data->logiccmd.value;
        break;
    }
    case LOOP:
        //TODO :(
        break;

    default:
        break;
    }

    if (!execute_action){
        cb(DONE_OK, 0);
        return;
    }

    logic_action_type action = data->logiccmd.action;
    switch (action)
    {
    case LA_STOP_ROB:
        Robot_Stop(data->logiccmd.robot);
        cb(DONE_OK, 0);
        break;

    case LA_RESET_ROB_POS:
    {
        robot_payload_t robdata;
        robdata.robcmd.robot = data->logiccmd.robot;
        robdata.robcmd.type = RESET_POS;
        Robot_Dispatch(&robdata, cb);
        break;
    }

    case LA_RETURN_TO_PREV_POS:
    {
        robot_payload_t robdata;
        robdata.robcmd.robot = data->logiccmd.robot;
        robdata.robcmd.type = SAFE_RETURN;
        Robot_Dispatch(&robdata, cb);
        break;
    }
    case LA_RETURN_TO_POS_ZERO:
    {
        robot_payload_t robdata;
        robdata.robcmd.robot = data->logiccmd.robot;
        robdata.robcmd.type = RUN_TO_POINT;
        robdata.robcmd.speed = 100;
        robdata.robcmd.point.x_pos = 0;
        robdata.robcmd.point.y_pos = 0;
        Robot_Dispatch(&robdata, cb);
        break;
    }

    case LA_RESET_CMD_QUEUE:
        Command_ResetQueue(NORMAL_SEVERITY);
        cb(DONE_OK, 0);
        break;

    case LA_RESET_TRACE_QUEUE:
        Trace_ResetQueue();
        cb(DONE_OK, 0);
        break;
    case LA_REINIT_IMU:
        Sensor_SetState(IMU, SENSOR_ENABLE);
        cb(DONE_OK, 0);
        break;
    case LA_STOP_IMU:
        Sensor_SetState(IMU, SENSOR_DISABLE);
        cb(DONE_OK, 0);
        break;
    case LA_RECALIBRATE_IMU:
    {
        sensor_payload_t data;
        data.senscmd.sensor = IMU;
        data.senscmd.type = CALIBRATE;
        data.senscmd.calibration_steps = 50;    //has to be something
        Sensor_Dispatch(&data, cb);
    }
        break;
    case LA_START_MONITORINGS:
        Monitoring_Setup(MON_ENABLED);
        cb(DONE_OK, 0);
        break;
    case LA_STOP_MONITORINGS:
        Monitoring_Setup(MON_DISABLED);
        cb(DONE_OK, 0);
        break;
    case LA_SYSTEM_SHUTDOWN:
        //TODO
        break;
    
    default:
        break;
    }


}
