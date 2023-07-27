#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "imu.h"
#include "filters.h"
#include "imu_handle.h"
#include "sensors_common.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"

extern IMU_Handle_t imu_sensor;
extern I2C_HandleTypeDef hi2c1;


static SemaphoreHandle_t orient_access_sem;
static euler_angles_t actual_orient;

static void orient_access(euler_angles_t* orient, bool write){
    
    if(xSemaphoreTake(orient_access_sem, 10) == pdPASS){
        if (write){
            actual_orient = *orient;
        } else {
            *orient = actual_orient;
        }
        xSemaphoreGive(orient_access_sem);
    }
}

static bool orient_change_detection(const euler_angles_t* new_orient){

    static euler_angles_t prev_orient;

    if ((abs((int16_t)new_orient->roll - (int16_t)prev_orient.roll) >= 1u) ||
        (abs((int16_t)new_orient->pitch - (int16_t)prev_orient.pitch) >= 1u) ||
        (abs((int32_t)new_orient->yaw - (int32_t)prev_orient.yaw) >= 1u)) {
        
        orient_access(new_orient, true);

        prev_orient = *new_orient;
        return true;
    }
    return false;
}



void Sensor_Init(){
    
	orient_access_sem = xSemaphoreCreateMutex();
    imu_sensor=IMU_Initialize(ICM20600_I2C_ADDR2, &hi2c1);

    bool cal_status = IMU_GyroCalibration(imu_sensor, IMU_GYRO_CALIB_CNT);

    if (cal_status == IMU_OK) {
        SENS_DEBUG("Calibrated OK \r\n");
    } else {
        SENS_DEBUG("Gyro NOT calibrated \r\n");
    }

    cal_status = IMU_MagnCalibration(imu_sensor, IMU_GYRO_CALIB_CNT);

    if (cal_status == IMU_OK) {
        SENS_DEBUG("Calibrated OK \r\n");
    } else {
        SENS_DEBUG("Gyro NOT calibrated \r\n");
    }
}

void Sensor_Task(){


    euler_angles_t orient = IMU_CalcEuler(imu_sensor);

    (void)orient_change_detection(&orient);

}


void Sensor_GetValue(sensors_t sensor, void* value){

    euler_angles_t orient;
    switch (sensor)
    {
    case IMU:
        orient_access(&orient, false);
        memcpy(value, &orient, sizeof(orient));
        break;
    
    default:
        break;
    }
}

