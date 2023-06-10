#ifndef IMU_HANDLE_H_
#define IMU_HANDLE_H_

#include "sensors_common.h"
#include "stm32f4xx_hal.h"

#define IMU_GYRO_CALIB_CNT 				100u

#define ICM20600_I2C_ADDR1              0x68
#define ICM20600_I2C_ADDR2              0x69

// ICM20600 power mode
#define 	IMU_SLEEP_MODE  	0u
#define    	IMU_STANDYBY_MODE 	1u
#define    	IMU_ACC_LOW_POWER 	2u
#define    	IMU_ACC_LOW_NOISE 	3u
#define    	IMU_GYRO_LOW_POWER 	4u
#define    	IMU_GYRO_LOW_NOISE	5u
#define    	IMU_6AXIS_LOW_POWER	6u
#define    	IMU_6AXIS_LOW_NOISE 7u



//Handle for IMU Sensor
typedef struct IMU_Sensor_t* IMU_Handle_t;

//Return code
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







//Public Interface

IMU_Handle_t IMU_Initialize(uint8_t i2c_addr, I2C_HandleTypeDef* i2c_inst);

void IMU_Destroy(IMU_Handle_t imuInstance);

IMU_ReturnCode_t IMU_GyroCalibration(IMU_Handle_t imuInstance, uint32_t measures);

IMU_ReturnCode_t IMU_MagnCalibration(IMU_Handle_t imuInstance);

IMU_ReturnCode_t IMU_GetAcceleration(IMU_Handle_t imuInstance);

IMU_ReturnCode_t IMU_GetGyro(IMU_Handle_t imuInstance);

IMU_ReturnCode_t IMU_SetPowerMode(IMU_Handle_t imuInstance, uint8_t mode);

IMU_ReturnCode_t IMU_GetMagn(IMU_Handle_t imuInstance);

euler_angles_t IMU_CalcEuler(IMU_Handle_t imuInstance);

int16_t IMU_GetTemperature(IMU_Handle_t imuInstance);


#endif