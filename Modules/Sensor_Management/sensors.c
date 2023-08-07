#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "imu.h"
#include "filters.h"
#include "sensor.h"
#include "imu_handle.h"
#include "sensors_common.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"



/**
 * @brief IMU sensor data access function callback
 * 
 */
static void imu_orient_access(const Sensor_t* me, void* data, bool write);

/**
 * @brief IMU sensor data update
 * 
 */
static void imu_orient_update(const Sensor_t* me);


/**
 * @brief PRIVATE DATA 
 * 
 */


static const struct data_handling_vtable Sensors_VTABLE[SENSOR_MAX] = 
{
	[IMU] = {.data_access = imu_orient_access, .data_update = imu_orient_update}
};


static Sensor_t sensors_table[SENSOR_MAX];


extern I2C_HandleTypeDef hi2c1;


/**
 * @brief PRIVATE FUNCTIONS 
 * 
 */
static void imu_orient_update(const Sensor_t* me){
    euler_angles_t orient = IMU_CalcEuler((IMU_Handle_t)me->sensor_handle);

    me->vptr->data_access(me, &orient, true);
}

static void imu_orient_access(const Sensor_t* me, void* data, bool write){
    
    static euler_angles_t actual_orient;

    if(xSemaphoreTake(me->data_access_sem, 10) == pdPASS){
        if (write){
            memcpy(&actual_orient, data, me->data_size);
        } else {
            memcpy(data, &actual_orient, me->data_size);
        }
        xSemaphoreGive(me->data_access_sem);
    }
}


/**
 * @brief PUBLIC FUNCTIONS 
 * 
 */
void* Sensor_Init(sensors_id_t sensor){
    
    switch (sensor)
    {
    case IMU:
        {
            IMU_Handle_t imu_sensor;

            imu_sensor=IMU_Initialize(ICM20600_I2C_ADDR2, &hi2c1);

            if (imu_sensor == NULL){
                return NULL;
            }

            sensors_table[sensor].sensor_id = sensor;
            sensors_table[sensor].data_access_sem = xSemaphoreCreateMutex();
            sensors_table[sensor].vptr = &Sensors_VTABLE[sensor];
            sensors_table[sensor].sensor_handle = (IMU_Handle_t)imu_sensor;
            sensors_table[sensor].data_size = sizeof(euler_angles_t);


            bool cal_status = IMU_GyroCalibration(imu_sensor, IMU_GYRO_CALIB_CNT);

            if (cal_status == IMU_OK) {
                SENS_DEBUG("Calibrated OK \r\n");
            } else {
                SENS_DEBUG("Gyro NOT calibrated \r\n");
            }

            //cal_status = IMU_MagnCalibration(imu_sensor, IMU_GYRO_CALIB_CNT);

            //if (cal_status == IMU_OK) {
                //SENS_DEBUG("Calibrated OK \r\n");
            //} else {
                //SENS_DEBUG("Gyro NOT calibrated \r\n");
            //}

            return imu_sensor;
        }
        break;
    }


    return NULL;
}

void Sensor_Task(){

    for (int8_t sens_idx = 0; sens_idx < SENSOR_MAX; sens_idx++){

    	Sensor_t *sensor = &sensors_table[sens_idx];
    	sensor->vptr->data_update(sensor);
    }

}


void Sensor_GetValue(sensors_id_t sensor_id, void* value){
	//TODO add guard
	Sensor_t *sensor = &sensors_table[sensor_id];
	sensor->vptr->data_access(sensor, value, false);
}

