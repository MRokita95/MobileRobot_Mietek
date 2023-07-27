/*
 * imu.c
 *
 *  Created on: Sep 18, 2022
 *      Author: micha
 */


#include "imu.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "imu_handle.h"


IMU_Handle_t IMU_Initialize(uint8_t i2c_addr, I2C_HandleTypeDef* i2c_inst)
{
	IMU_Handle_t imuInstance = malloc(sizeof(struct IMU_Sensor_t));
	
	assert_param(imuInstance != NULL);
	assert_param((i2c_addr == ICM20600_I2C_ADDR1) || (i2c_addr == ICM20600_I2C_ADDR2));

	imuInstance->icm.driver.addr = i2c_addr;
	imuInstance->i2c_instance = i2c_inst;

	imuInstance->read_timeout = HAL_MAX_DELAY;
	imuInstance->write_timeout = HAL_MAX_DELAY;

	//init calibration values
	imuInstance->icm.calibration.offset[x] = 0;
	imuInstance->icm.calibration.offset[y] = 0;
	imuInstance->icm.calibration.offset[z] = 0;

	imuInstance->ak.calibration.offset[x] = 0;
	imuInstance->ak.calibration.offset[y] = 0;
	imuInstance->ak.calibration.offset[z] = 0;


	ICM_initialize(&imuInstance->icm.driver, imuInstance->i2c_instance);

	//AK Driver not ready
	imuInstance->ak.driver.addr = AK09918_I2C_ADDR;	//the only address
	AK09918_err_type_t ak_status = AK_initialize(&imuInstance->ak.driver, imuInstance->i2c_instance, AK09918_CONTINUOUS_100HZ);
	imuInstance->ak.driver.device_ID = AK_getDeviceID(&imuInstance->ak.driver, imuInstance->i2c_instance);


	IMU_ReturnCode_t driver_status = (IMU_ReturnCode_t) imuInstance->icm.driver.status;
	imuInstance->icm.driver.status = HAL_OK;

	return imuInstance;

}

IMU_ReturnCode_t IMU_GetAcceleration(IMU_Handle_t imuInstance)
{
	ICM_getAcceleration(&imuInstance->icm.driver,
						imuInstance->i2c_instance,
						&imuInstance->icm.accel[x],
						&imuInstance->icm.accel[y],
						&imuInstance->icm.accel[z]);

	IMU_ReturnCode_t driver_status = (IMU_ReturnCode_t) imuInstance->icm.driver.status;
	imuInstance->icm.driver.status = HAL_OK;

	return driver_status;
}

IMU_ReturnCode_t IMU_GetGyro(IMU_Handle_t imuInstance)
{
	ICM_getGyroscope(&imuInstance->icm.driver,
			         imuInstance->i2c_instance,
					 &imuInstance->icm.speed[x],
					 &imuInstance->icm.speed[y],
					 &imuInstance->icm.speed[z]);

	IMU_ReturnCode_t driver_status = (IMU_ReturnCode_t) imuInstance->icm.driver.status;
	imuInstance->icm.driver.status = HAL_OK;

	return driver_status;
}

IMU_ReturnCode_t IMU_SetPowerMode(IMU_Handle_t imuInstance, uint8_t mode)
{
	ICM_setPowerMode(&imuInstance->icm.driver, imuInstance->i2c_instance, (icm20600_power_type_t)mode);

	IMU_ReturnCode_t driver_status = (IMU_ReturnCode_t) imuInstance->icm.driver.status;
	imuInstance->icm.driver.status = HAL_OK;

	return driver_status;
}

IMU_ReturnCode_t IMU_GetMagn(IMU_Handle_t imuInstance)
{
	AK09918_err_type_t status = AK_isDataReady(&imuInstance->ak.driver,
												imuInstance->i2c_instance);

	if (status != AK09918_ERR_OK){
		return IMU_ERROR;
	}

	status = AK_getData(&imuInstance->ak.driver,
						imuInstance->i2c_instance,
						&imuInstance->ak.axis[x],
						&imuInstance->ak.axis[y],
						&imuInstance->ak.axis[z]);

	if (status == AK09918_ERR_TIMEOUT){
		return IMU_TIMEOUT;
	} else {
		return IMU_OK;
	}

}

euler_angles_t IMU_CalcEuler(IMU_Handle_t imuInstance)
{
	imuInstance->prev_orient = imuInstance->orient;

	ICM_getAcceleration(&imuInstance->icm.driver,
							imuInstance->i2c_instance,
							&imuInstance->icm.accel[x],
							&imuInstance->icm.accel[y],
							&imuInstance->icm.accel[z]);

	IMU_ReturnCode_t driver_status = (IMU_ReturnCode_t) imuInstance->icm.driver.status;
	imuInstance->icm.driver.status = HAL_OK;

	ICM_getGyroscope(&imuInstance->icm.driver,
			         imuInstance->i2c_instance,
					 &imuInstance->icm.speed[x],
					 &imuInstance->icm.speed[y],
					 &imuInstance->icm.speed[z]);

	driver_status |= (IMU_ReturnCode_t) imuInstance->icm.driver.status;
	imuInstance->icm.driver.status = HAL_OK;

	//get magnetometer data
	IMU_GetMagn(imuInstance);

	//should be done in seperate function or in main.c
	imuInstance->filter_callback = ComplementaryFilter;

	int16_t gyro_calibrated[3];
	gyro_calibrated[x] = imuInstance->icm.speed[x] - imuInstance->icm.calibration.offset[x];
	gyro_calibrated[y] = imuInstance->icm.speed[y] - imuInstance->icm.calibration.offset[y];
	gyro_calibrated[z] = imuInstance->icm.speed[z] - imuInstance->icm.calibration.offset[z];

	int32_t magn_calibrated[3];
	magn_calibrated[x] = (int32_t)((float)(imuInstance->ak.axis[x] - imuInstance->ak.calibration.offset[x]) * imuInstance->ak.calibration.scale[x]);
	magn_calibrated[y] = (int32_t)((float)(imuInstance->ak.axis[y] - imuInstance->ak.calibration.offset[y]) * imuInstance->ak.calibration.scale[y]);
	magn_calibrated[z] = (int32_t)((float)(imuInstance->ak.axis[z] - imuInstance->ak.calibration.offset[z]) * imuInstance->ak.calibration.scale[z]);

	imuInstance->orient = imuInstance->filter_callback(	imuInstance->icm.accel,
														&gyro_calibrated,
														&magn_calibrated);

	return imuInstance->orient;

}

IMU_ReturnCode_t IMU_GyroCalibration(IMU_Handle_t imuInstance, uint32_t measures)
{

	imuInstance->icm.calibration.samples = measures;


	for (int32_t smp = 0; smp < imuInstance->icm.calibration.samples; smp++){

		ICM_getGyroscope(&imuInstance->icm.driver,
				         imuInstance->i2c_instance,
						 &imuInstance->icm.speed[x],
						 &imuInstance->icm.speed[y],
						 &imuInstance->icm.speed[z]);

		imuInstance->icm.calibration.offset[x] += imuInstance->icm.speed[x];
		imuInstance->icm.calibration.offset[y] += imuInstance->icm.speed[y];
		imuInstance->icm.calibration.offset[z] += imuInstance->icm.speed[z];

		if (imuInstance->icm.driver.status != IMU_OK) {
			return imuInstance->icm.driver.status;
		}

		osDelay(10);
	}

	imuInstance->icm.calibration.offset[x] = imuInstance->icm.calibration.offset[x]/imuInstance->icm.calibration.samples;
	imuInstance->icm.calibration.offset[y] = imuInstance->icm.calibration.offset[y]/imuInstance->icm.calibration.samples;
	imuInstance->icm.calibration.offset[z] = imuInstance->icm.calibration.offset[z]/imuInstance->icm.calibration.samples;

	imuInstance->icm.calibration.calibrated = true;

	return imuInstance->icm.driver.status;
}

int16_t IMU_GetTemperature(IMU_Handle_t imuInstance)
{
	imuInstance->icm.temperature =  ICM_getTemperature(&imuInstance->icm, imuInstance->i2c_instance);

	return imuInstance->icm.temperature;
}

IMU_ReturnCode_t IMU_MagnCalibration(IMU_Handle_t imuInstance, uint32_t measures)
{
    int32_t value_x_min = 0;
    int32_t value_x_max = 0;
    int32_t value_y_min = 0;
    int32_t value_y_max = 0;
    int32_t value_z_min = 0;
    int32_t value_z_max = 0;
    uint32_t timeStart = 0;
    int32_t act_smp = 0;

	imuInstance->ak.calibration.samples = measures;
	imuInstance->ak.calibration.timeout=100;

	AK09918_err_type_t status = AK_isDataReady(&imuInstance->ak.driver,
												imuInstance->i2c_instance);

	if (status != AK09918_ERR_OK){
		AK_reset(&imuInstance->ak.driver, imuInstance->i2c_instance);
		osDelay(100);
	}

	osDelay(100);
	status = AK_isDataReady(&imuInstance->ak.driver,
												imuInstance->i2c_instance);

    int32_t x;
    int32_t y;
    int32_t z;

    status = AK_getData(&imuInstance->ak.driver,
    										imuInstance->i2c_instance,
											&x,
											&y,
											&z);

    if (status != AK09918_ERR_OK){
    	return IMU_ERROR;
    }

    value_x_min = x;
    value_x_max = x;
    value_y_min = y;
    value_y_max = y;
    value_z_min = z;
    value_z_max = z;

    timeStart = HAL_GetTick()/portTICK_PERIOD_MS;
    while ((act_smp < imuInstance->ak.calibration.samples) && (status == AK09918_ERR_OK))
	{
    	if ((HAL_GetTick()/portTICK_PERIOD_MS - timeStart) >= imuInstance->ak.calibration.timeout)
    	{
    		status = AK_getData(&imuInstance->ak.driver,
													imuInstance->i2c_instance,
													&x,
													&y,
													&z);

			if (status != AK09918_ERR_OK){
				status = AK09918_ERR_OK; //cheat a little - continue to another iteration
				timeStart = HAL_GetTick()/portTICK_PERIOD_MS;
				continue;
			}

			status = AK_getData(&imuInstance->ak.driver,
						imuInstance->i2c_instance,
						&x,
						&y,
						&z);

			if (status != AK09918_ERR_OK){
				return IMU_ERROR;
			}

			/* Update x-Axis max/min value */
			if (value_x_min > x) {
				value_x_min = x;
			} else if (value_x_max < x) {
				value_x_max = x;
			}

			/* Update y-Axis max/min value */
			if (value_y_min > y) {
				value_y_min = y;
			} else if (value_y_max < y) {
				value_y_max = y;
			}

			/* Update z-Axis max/min value */
			if (value_z_min > z) {
				value_z_min = z;
			} else if (value_z_max < z) {
				value_z_max = z;
			}

			act_smp++;
			timeStart = HAL_GetTick()/portTICK_PERIOD_MS;
    	}
    }

    // imuInstance->ak.calibration.offset[0] = value_x_min + (value_x_max - value_x_min) / 2;
    // imuInstance->ak.calibration.offset[1] = value_y_min + (value_y_max - value_y_min) / 2;
    // imuInstance->ak.calibration.offset[2] = value_z_min + (value_z_max - value_z_min) / 2;

	imuInstance->ak.calibration.offset[0] = (value_x_max + value_x_min) / 2;
    imuInstance->ak.calibration.offset[1] = (value_y_max + value_y_min) / 2;
    imuInstance->ak.calibration.offset[2] = (value_z_max + value_z_min) / 2;

	

	float avg_delta_x = (value_x_max - value_x_min) / 2;
	float avg_delta_y = (value_y_max - value_y_min) / 2;
	float avg_delta_z = (value_z_max - value_z_min) / 2;
	float avg_delta = (avg_delta_x + avg_delta_y + avg_delta_z) / 3;

	imuInstance->ak.calibration.scale[0] = avg_delta / avg_delta_x;
	imuInstance->ak.calibration.scale[1] = avg_delta / avg_delta_y;
	imuInstance->ak.calibration.scale[2] = avg_delta / avg_delta_z;


    imuInstance->ak.calibration.calibrated = true;

    return IMU_OK;

}

void IMU_Destroy(IMU_Handle_t imuInstance)
{
	if(imuInstance != NULL){
		free(imuInstance);
	}
}
