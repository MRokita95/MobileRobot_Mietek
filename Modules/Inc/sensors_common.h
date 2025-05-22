#ifndef SENSORS_COMMON_H_
#define SENSORS_COMMON_H_

#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "stdbool.h"
#include "commands_types.h"

#define IMU_LOG_ACTIVE
#define IMU_LOG_FREQUENCY 1000u	// [ms]

typedef enum{
    SENSOR_ENABLE,
    SENSOR_DISABLE
}sensor_state_t;

typedef enum{
    SENSOR_DISABLED,
    SENSOR_UNCALIBRATED,
    SENSOR_WORKING
}sensor_status_t;

typedef union payload sensor_payload_t;

typedef enum{
	IMU = 0,
    SENSOR_MAX
}sensors_id_t;


#define DEBUG_PRINT 1
#ifdef DEBUG_PRINT
    extern UART_HandleTypeDef huart2;
    extern void send_uart(UART_HandleTypeDef* uart_instance, void const * argument);
    #define SENS_DEBUG(...) send_uart(&huart2, __VA_ARGS__)
#else
    #define SENS_DEBUG(...) {}
#endif

typedef struct {
	float roll;
	float pitch;
	float yaw;
} euler_angles_t;


void* Sensor_Init(sensors_id_t sensor);

void Sensor_Task();

void Sensor_SetState(sensors_id_t sensor, sensor_state_t state);

void Sensor_GetValue(sensors_id_t sensor, void* value);

sensor_status_t Sensor_GetState(sensors_id_t sensor);

bool Sensor_Ready(sensor_payload_t* data);

void Sensor_Dispatch(sensor_payload_t* data, status_notif_cb cb);

#endif
