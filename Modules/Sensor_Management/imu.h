/*
 * imu.h
 *
 *  Created on: Sep 18, 2022
 *      Author: micha
 */

#ifndef APP_SENSOR_MANAGEMENT_IMU_H_
#define APP_SENSOR_MANAGEMENT_IMU_H_

#include "ICM20600.h"
#include "AK09918.h"
#include "i2c_utils.h"
#include "filters.h"

enum {
	x = 0,
	y,
	z,
};

typedef struct {
	int32_t samples;
	int32_t offset[3];
	bool calibrated;
} gyro_calibration_t;

typedef struct {
	ICM_Handle_t driver;
	int16_t temperature;
	int16_t accel[3];
	int16_t speed[3];
	gyro_calibration_t calibration;
} Sensor_ICM_Handle_t;

typedef struct {
	int32_t samples;
	uint32_t timeout;
	int32_t offset[3];
	bool calibrated;
} magn_calibration_t;

typedef struct {
	AK0_Handle_t driver;
	int32_t axis[3];
	magn_calibration_t calibration;
} Sensor_AK_Handle_t;



struct IMU_Sensor_t{
	I2C_HandleTypeDef i2c_instance;
	Sensor_ICM_Handle_t icm;
	Sensor_AK_Handle_t ak;
	euler_angles_t orient;
	euler_angles_t prev_orient;
	estim_filter_fnc_t filter_callback;
	data_filter_fnc_t data_filter_callback;
	uint32_t read_timeout;
	uint32_t write_timeout;
};


#endif /* APP_SENSOR_MANAGEMENT_IMU_H_ */
