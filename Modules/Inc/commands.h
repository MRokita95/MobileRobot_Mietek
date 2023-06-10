#ifndef COMMANDS_H_
#define COMMANDS_H_

#include "robot.h"
#include <stdint.h>

#define MAX_COMMANDS_CNT 100u

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
    AUTOPATH_STOP
} command_type_t;

typedef enum{
    EMPTY = 0,
    IDLE,
    IN_PROGRESS,
    DONE_OK,
    ERR,
    TIMEOUT
} command_status_t;


typedef union{
    struct control_bits{
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t left : 1;
        uint8_t down : 1;
    } bit;
    uint8_t control_byte;
} manual_ctrl_command_t;


typedef struct {
    uint32_t id;
    command_type_t type;
    command_status_t status;
    uint32_t time;
    int16_t speed;
    rob_coord_t point;
    int32_t distance;
    int16_t angle;
    manual_ctrl_command_t man_ctrl;
} command_t;

typedef enum{
    BUFF_EMPTY = 0,
    BUFF_OK,
    BUFF_FULL,
    BUFF_NOK,
} command_buff_status_t;


command_buff_status_t command_add(command_t command);

command_buff_status_t command_get_next(command_t* command);

command_buff_status_t command_buff_status();

command_status_t command_actual_status(void);

void command_set_status(command_status_t status);



#endif
