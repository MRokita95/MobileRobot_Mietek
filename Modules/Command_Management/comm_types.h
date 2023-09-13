#ifndef COMM_TYPES_H_
#define COMM_TYPES_H_

#include <stdint.h>
#include <stdlib.h>
#include "robot.h"

#define ROB_APP_ID  0x10u
#define MEM_APP_ID  0x11u
#define PAR_APP_ID  0x12u
#define SENS_APP_ID 0x13u
#define HK_APP_ID   0x14u
#define TR_DATA_APP_ID   0x15u


#define FLASH_LOAD_FNC_ID  0x01u
#define FLASH_SAVE_FNC_ID  0x02u

#define PARAM_SET_FNC_ID  0x01u
#define PARAM_GET_FNC_ID  0x02u


#define SENS_IMU_FNC_ID 0x01u

typedef struct {
    uint8_t roll;
    uint8_t pitch;
    uint8_t yaw;
} imu_data_response_t;


typedef struct {
    int16_t robot_status;
    int16_t robot_mode;
    int16_t actual_speed;
    int16_t setpoint_speed;
    int32_t actual_distance;
    int32_t setpoint_distance;
    rob_coord_t actual_pos;
    rob_coord_t setpoint_pos;
    struct rob_orient{
        int16_t roll;
        int16_t pitch;
        int16_t yaw;
    };
} rob_data_response_t;

#endif