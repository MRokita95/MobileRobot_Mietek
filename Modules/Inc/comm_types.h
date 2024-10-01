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
    robot_status_t current_state;
    robot_mode_t active_mode;
    int32_t speed_setpoint;
    int32_t right_wheel_speed;
    int32_t left_wheel_speed;
} __attribute__((packed)) robot_status_data_t;

#endif