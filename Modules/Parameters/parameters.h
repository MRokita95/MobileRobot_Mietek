#ifndef PARAMETERS_H_
#define PARAMETERS_H_

#include "param_handle.h"
#include <stdbool.h>

#define ID_MAX     30u
#define STORED_PARAM_SIZE 256u

#define STORED_PARAMS_WORD 0x12345678

typedef struct{
    uint16_t id;
    bool settable;
    param_type_t type;
    uint16_t size;
    parameter_val_t value;
} param_t;

#endif