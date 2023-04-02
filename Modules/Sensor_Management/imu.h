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


typedef enum {
	IMU_OK = 0,
	IMU_ERROR,
	IMU_BUSY,
	IMU_TIMEOUT,

    IMU_ERR_DOR = 11,                // data skipped
	IMU_ERR_NOT_RDY,            // not ready
	IMU_ERR_TIMEOUT,            // read/write timeout
	IMU_ERR_SELFTEST_FAILED,    // self test failed
	IMU_ERR_OVERFLOW,           // sensor overflow, means |x|+|y|+|z| >= 4912uT
	IMU_ERR_WRITE_FAILED,       // fail to write
	IMU_ERR_READ_FAILED,        // fail to read
} IMU_ReturnCode_t;

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



typedef struct {
	I2C_HandleTypeDef i2c_instance;
	Sensor_ICM_Handle_t icm;
	Sensor_AK_Handle_t ak;
	euler_angles_t orient;
	euler_angles_t prev_orient;
	estim_filter_fnc_t filter_callback;
	data_filter_fnc_t data_filter_callback;
	uint32_t read_timeout;
	uint32_t write_timeout;
} IMU_Handle_t;

IMU_ReturnCode_t IMU_Initialize(IMU_Handle_t *imuInstance, uint8_t i2c_addr);

IMU_ReturnCode_t IMU_GetAcceleration(IMU_Handle_t *imuInstance);

IMU_ReturnCode_t IMU_GetGyro(IMU_Handle_t *imuInstance);

IMU_ReturnCode_t IMU_SetPowerMode(IMU_Handle_t *imuInstance, icm20600_power_type_t mode);

IMU_ReturnCode_t IMU_GetMagn(IMU_Handle_t *imuInstance);

void IMU_CalcEuler(IMU_Handle_t *imuInstance);

int16_t IMU_GetTemperature(IMU_Handle_t *imuInstance);

IMU_ReturnCode_t IMU_GyroCalibration(IMU_Handle_t *imuInstance);

IMU_ReturnCode_t IMU_MagnCalibration(IMU_Handle_t *imuInstance);

#endif /* APP_SENSOR_MANAGEMENT_IMU_H_ */
