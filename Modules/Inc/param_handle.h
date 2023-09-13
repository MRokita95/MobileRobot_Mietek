#ifndef PARAM_HANDLE_H_
#define PARAM_HANDLE_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>

typedef union
{
    uint8_t u8_val;
    uint16_t u16_val;
    uint32_t u32_val;
    int8_t i8_val;
    int16_t i16_val;
    int32_t i32_val;
    float f_val;
    bool b_val;
}parameter_val_t;

typedef enum{
    param_uint8_t = 0,
    param_uint16_t,
    param_uint32_t,
    param_int8_t,
    param_int16_t,
    param_int32_t,
    param_float,
    param_bool,
} param_type_t;

typedef enum{
    PARAM_OK = 0,
    PARAM_FAILED,
    PARAM_NOT_IN_FLASH
} param_ret_status_t;



#define KP_ID      1u
#define KI_ID      2u
#define KD_ID      3u
#define ACCEL_SETP_ID      4u

#define PARAM_SETTABLE(NAME, TYPE, DEFAULT) \
    do{ \
        static TYPE NAME##_par_id = (TYPE)DEFAULT; \
        array[NAME] = DEFAULT; \
    }while(0)

void Param_Initialize(void);

param_ret_status_t Param_Set(uint16_t id, void* value);

param_ret_status_t Param_Get(uint16_t id, void* value);

param_ret_status_t Param_SaveToFlash(void);

param_ret_status_t Param_LoadFromFlash(void);

#endif