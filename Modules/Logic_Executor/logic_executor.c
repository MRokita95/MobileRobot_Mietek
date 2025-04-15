#include "logic_executor.h"
#include "application_defs.h"
#include "robot.h"
#include "sensors_common.h"
#include "robot_app.h"

static int32_t m_vars[MAX_CUSTOM_VARS];

static uint8_t var_index(logic_operand_type op){
    uint8_t index = 0;
    if (op >= VAR0 && op <= VAR9){
        index = (uint8_t)(op - VAR0);
    }

    return index;
}


static bool if_eq_handler(logic_payload_t* data){

    logic_operand_type op = data->logiccmd.operand;
    int32_t value = data->logiccmd.value;

    bool equal = false;
    switch (op)
    {
    case POS_X:
    {
        rob_coord_t coord = Robot_GetCoord(data->logiccmd.robot);
        equal = (coord.x_pos == value);
        break;
    }
    
    case POS_Y:
    {
        rob_coord_t coord = Robot_GetCoord(data->logiccmd.robot);
        equal = (coord.y_pos == value);
        break;
    }
    
    case POS_Z:
    {
        rob_coord_t coord = Robot_GetCoord(data->logiccmd.robot);
        equal = (coord.z_pos == value);
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
        equal = m_vars[index] == value;
        break;
    }
    default:
        break;
    }

    return equal;
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
        ready = Sensor_GetState(IMU);
        break;
    default:
        ready = true;
        break;
    }

    return ready;
    
}

void LogicExecutor_Dispatch(logic_payload_t* data, status_update_cb cb){

    logic_command_type type = data->logiccmd.type;

    bool execute_action = false;

    switch (type)
    {
    case IF_EQUAL_THEN:
        execute_action = if_eq_handler(data);
        break;

    case VAR_ASSIGN_VALUE:
    {
        const uint8_t index = var_index(data->logiccmd.operand);
        m_vars[index] = data->logiccmd.value;
    }
    case LOOP:
        
        break;

    default:
        break;
    }

    if (!execute_action){
        cb(DONE_OK);
        return;
    }

    logic_action_type action = data->logiccmd.action;
    switch (action)
    {
    case LA_STOP_ROB:
        Robot_Stop(data->logiccmd.robot);
        break;

    case LA_RESET_ROB_POS:
        //TODO: implement
        break;

    case LA_RETURN_TO_PREV_POS:
        //TODO: implement
        break;
    case LA_RETURN_TO_POS_ZERO:
        ROBOT_MOVE_TO_POINT(100, 0, 0);
        break;

    case LA_RESET_CMD_QUEUE:
        //TODO
        break;

    case LA_RESET_TRACE_QUEUE:
        //TODO
        break;
    case LA_REINIT_IMU:
        //TODO
        break;
    case LA_STOP_IMU:
        //TODO
        break;
    case LA_RECALIBRATE_IMU:
        //TODO
        break;
    case LA_START_MONITORINGS:
        //TODO
        break;
    case LA_STOP_MONITORINGS:
        //TODO
        break;
    case LA_SYSTEM_SHUTDOWN:
        //TODO
        break;
    
    default:
        break;
    }

    cb(DONE_OK);

}