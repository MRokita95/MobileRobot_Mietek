#include <stdlib.h>
#include <string.h>
#include "parameters.h"
#include "flash_memory.h"
 

static param_t param_table[ID_MAX];
static uint16_t param_idx;


#define ADD_SETTABLE_PARAM(ID,DEFAULT,TYPE) \
    do{ \
        param_table[param_idx].id = ID; \
        param_table[param_idx].settable = true; \
        param_table[param_idx].type = param_##TYPE; \
        param_table[param_idx].size = sizeof(TYPE); \
        set_param_value(ID, &(TYPE){DEFAULT}); \
        param_idx++; \
    }while(0)

#define ADD_PARAM(ID,TYPE) \
    do{ \
        param_table[param_idx].id = ID; \
        param_table[param_idx].size = sizeof(TYPE); \
        param_table[param_idx].type = param_##TYPE; \
        param_idx++; \
    }while(0)


static uint16_t get_param_idx(uint16_t id){
    for (uint16_t idx = 0; idx < ID_MAX; idx++){
        if (param_table[idx].id == id){
            return idx;
        }
    }
    return ID_MAX;
}

static param_ret_status_t handle_param(uint16_t id, void* value, bool write){
    if (id > ID_MAX) {
        return PARAM_FAILED;
    }

    uint16_t idx = get_param_idx(id);

    if (idx == ID_MAX){
        return PARAM_FAILED;
    }

    void* src;
    void* dest;
    if (write){
        src = value;
        dest = &param_table[idx].value;
    } else {
        src = &param_table[idx].value;
        dest = value;
    }


    switch (param_table[idx].type)
    {
    case param_uint8_t:
        memcpy(dest, src, sizeof(uint8_t));
        break;

    case param_uint16_t:
        memcpy(dest, src, sizeof(uint16_t));
        break;

    case param_uint32_t:
        memcpy(dest, src, sizeof(uint32_t));
        break;

    case param_int8_t:
        memcpy(dest, src, sizeof(int8_t));
        break;

    case param_int16_t:
        memcpy(dest, src, sizeof(int16_t));
        break;

    case param_int32_t:
        memcpy(dest, src, sizeof(int32_t));
        break;

    case param_float:
        memcpy(dest, src, sizeof(float));
        break;

    case param_bool:
        memcpy(dest, src, sizeof(bool));
        break;
    
    default:
        return PARAM_FAILED;
        break;
    }

    return PARAM_OK;
}


/*
-------------------- PUBLIC FUNCTIONS -----------------------
 */

void Param_Initialize(){

    /* Set Parameters ID and default values */
    ADD_SETTABLE_PARAM(KP_ID, 0.06, float);
    ADD_SETTABLE_PARAM(KI_ID, 0.9, float);
    ADD_SETTABLE_PARAM(KD_ID, 0.4, float);

    /*  Check if parameters are store in FLASH.
        If stored - default values will be overwritten, 
        otherwise, default values remain */

    (void)Param_LoadFromFlash();


}


param_ret_status_t Param_Set(uint16_t id, void* value){
    return handle_param(id, value, true);
}

param_ret_status_t Param_Get(uint16_t id, void* value){
    return handle_param(id, value, false);
}

param_ret_status_t Param_SaveToFlash(void){

    uint8_t block[STORED_PARAM_SIZE];
    uint16_t bytes = 0;

    uint8_t store_sign[4];
    store_sign[0] = (STORED_PARAMS_WORD & 0xFF000000) >> 24;
    store_sign[1] = (STORED_PARAMS_WORD & 0x00FF0000) >> 16;
    store_sign[2] = (STORED_PARAMS_WORD & 0x0000FF00) >> 8;
    store_sign[3] = STORED_PARAMS_WORD & 0x000000FF;

    uint32_t status = Flash_Write(FLASH_START_PARAM_ADDR, &store_sign, 4u);

    for (uint16_t idx = 0; idx < ID_MAX; idx++){
        if (param_table[idx].id != 0u && param_table[idx].settable){
            memcpy(&block[bytes], &param_table[idx].value, param_table[idx].size);
            bytes += param_table[idx].size;
        }

        if (bytes >= STORED_PARAM_SIZE){
            return PARAM_FAILED;
        }
    }

    uint32_t param_start_addr = FLASH_START_PARAM_ADDR + 4u;
    status = Flash_Write(param_start_addr, block, bytes);

    if (status == 0){
        return PARAM_OK;
    } else {
        return PARAM_FAILED;
    }
}

param_ret_status_t Param_LoadFromFlash(void){
    uint8_t block[STORED_PARAM_SIZE];
    uint16_t bytes = 0u;

    uint8_t store_sign[4];
    Flash_Read(FLASH_START_PARAM_ADDR, store_sign, 4u);

    uint32_t store_word = 0;
    
    store_word |= store_sign[0];
    store_word = store_word << 8;
    store_word |= store_sign[1];
    store_word = store_word << 8;
    store_word |= store_sign[2];
    store_word = store_word << 8;
    store_word |= store_sign[3];

    if (store_word != STORED_PARAMS_WORD){
        return PARAM_NOT_IN_FLASH;
    }

    uint32_t param_start_addr = FLASH_START_PARAM_ADDR + 4u;

    /* first iteration for retreive number of bytes */
    for (uint16_t idx = 0; idx < ID_MAX; idx++){
        if (param_table[idx].id != 0u && param_table[idx].settable){

            bytes += param_table[idx].size;
        }

        if (bytes > STORED_PARAM_SIZE){
            return PARAM_FAILED;
        }
    }

    Flash_Read(param_start_addr, block, bytes);

    bytes = 0u; //reset bytes

    /* second iteration for assigning values */
    for (uint16_t idx = 0; idx < ID_MAX; idx++){
        if (param_table[idx].id != 0u && param_table[idx].settable){
            memcpy(&param_table[idx].value, block[bytes], param_table[idx].size);
            bytes += param_table[idx].size;
        }

        if (bytes > STORED_PARAM_SIZE){
            return PARAM_FAILED;
        }
    }

    return PARAM_OK;
}
