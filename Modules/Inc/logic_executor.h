#ifndef LOGIC_EXECUTOR_H_
#define LOGIC_EXECUTOR_H_

#include "stdbool.h"
#include "stdint.h"
#include "commands_types.h"



typedef union payload logic_payload_t;


bool LogicExecutor_Ready(logic_payload_t* data);

void LogicExecutor_Dispatch(logic_payload_t* data, status_update_cb cb);

#endif