#ifndef SENSORS_COMMON_H_
#define SENSORS_COMMON_H_

#include <stdlib.h>

#define IMU_LOG_ACTIVE
#define IMU_LOG_FREQUENCY 1000u	// [ms]


typedef enum{
	IMU = 0,
}sensors_t;


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


void Sensor_Init(void);

void Sensor_Task(void);

void Sensor_GetValue(sensors_t sensor, void* value);

#endif
