/*
 * filters.h
 *
 *  Created on: Sep 19, 2022
 *      Author: micha
 */

#ifndef APP_SENSOR_MANAGEMENT_FILTERS_H_
#define APP_SENSOR_MANAGEMENT_FILTERS_H_

#include "stm32f4xx_hal.h"
#include "sensors_common.h"

#define MAGNETIC_DECLINATION 6.33f

typedef struct {
	int16_t acc[3];
	int16_t gyro[3];
	int32_t axis[3];
	euler_angles_t orient;
	euler_angles_t orient_radians;
} prev_values_t;


typedef struct {
	euler_angles_t actual_orient;
	euler_angles_t acc_orient;
	euler_angles_t gyro_orient;
	euler_angles_t ang_speed;
	prev_values_t prev;
	int16_t dead_zone;
	int32_t samples;
	float alpha;
	uint32_t last_tick;
} SensorFilter_t;

typedef void (*data_filter_fnc_t)(int16_t *acc, int16_t *speed, int32_t *axis);
typedef euler_angles_t (*estim_filter_fnc_t)(int16_t *acc, int16_t *speed, int32_t *axis);


euler_angles_t ComplementaryFilter(int16_t *acc, int16_t *speed, int32_t *axis);


#endif /* APP_SENSOR_MANAGEMENT_FILTERS_H_ */
