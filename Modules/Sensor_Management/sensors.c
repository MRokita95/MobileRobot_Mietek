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
#include "application_defs.h"



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


static void imu_calib(Sensor_t* me);


/**
 * @brief PRIVATE DATA 
 * 
 */


static const struct data_handling_vtable Sensors_VTABLE[SENSOR_MAX] = 
{
	[IMU] = {.data_access = imu_orient_access, .data_update = imu_orient_update, .calibration = imu_calib}
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

static void imu_calib(Sensor_t* me){
    
    if (IMU_GyroCalibration(me, me->calib_handle.samples)){
        me->calib_handle.state = CALIB_OK;
        if (me->calib_handle.status_notif != NULL){
            me->calib_handle.status_notif(DONE_OK, 0);
        }
    } else {
        me->calib_handle.state = CALIB_FAILED;
        if (me->calib_handle.status_notif != NULL){
            me->calib_handle.status_notif(ERR, NULL);
        }
    }

    //cal_status = IMU_MagnCalibration(imu_sensor, IMU_GYRO_CALIB_CNT);
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
            static IMU_Handle_t imu_sensor;

            imu_sensor=IMU_Initialize(ICM20600_I2C_ADDR2, &hi2c1);

            if (imu_sensor == NULL){
                return NULL;
            }

            sensors_table[sensor].sensor_id = sensor;
            sensors_table[sensor].data_access_sem = xSemaphoreCreateMutex();
            sensors_table[sensor].vptr = &Sensors_VTABLE[sensor];
            sensors_table[sensor].sensor_handle = (IMU_Handle_t)imu_sensor;
            sensors_table[sensor].data_size = sizeof(euler_angles_t);
            sensors_table[sensor].calib_handle.state = UNCALIB;
            sensors_table[sensor].calib_handle.samples = IMU_GYRO_CALIB_CNT;
            sensors_table[sensor].init = true;
            return imu_sensor;
        }
        break;
    }


    return NULL;
}

void Sensor_Task(){

    for (int8_t sens_idx = 0; sens_idx < SENSOR_MAX; sens_idx++){

    	Sensor_t *sensor = &sensors_table[sens_idx];

        if(sensor->init){

            if (sensor->calib_handle.state == CALIB_OK){
                sensor->vptr->data_update(sensor);
            }
            else if (sensor->calib_handle.state == CALIB_REQUESTED) {
                sensor->vptr->calibration(sensor);
            }
        }
    }

}


void Sensor_GetValue(sensors_id_t sensor_id, void* value){
	//TODO add guard
	Sensor_t *sensor = &sensors_table[sensor_id];
    if(sensor->init){
	    sensor->vptr->data_access(sensor, value, false);
    }
}

sensor_status_t Sensor_GetState(sensors_id_t sensor_id){
    Sensor_t *sensor = &sensors_table[sensor_id];

    if (sensor->init && (sensor->calib_handle.state == CALIB_OK)){
        return SENSOR_WORKING;
    } else if (sensor->init){
        return SENSOR_UNCALIBRATED;
    } else {
        return SENSOR_DISABLED;
    }
}

void Sensor_SetState(sensors_id_t sensor_id, sensor_state_t state){
    Sensor_t *sensor = &sensors_table[sensor_id];

    sensor->init = (state == SENSOR_ENABLE);
}

bool Sensor_Ready(sensor_payload_t* data){
    
    return true;
}

void Sensor_Dispatch(sensor_payload_t* data, status_notif_cb cb){

    switch (data->senscmd.type)
    {
    case INIT:
        if (Sensor_Init(data->senscmd.sensor)){
            cb(DONE_OK, 0);
        } else {
            cb(ERR, 0);
        }
        break;

    case CALIBRATE:
        vTaskSuspendAll();
        sensors_table[data->senscmd.sensor].calib_handle.samples = data->senscmd.calibration_steps;
        sensors_table[data->senscmd.sensor].calib_handle.state = CALIB_REQUESTED;   //NOT THREAD SAFE!!!
        sensors_table[data->senscmd.sensor].calib_handle.status_notif = cb;
        xTaskResumeAll();
        break;

    case SET_MODE:
        /* TODO */
        break;

    case GET_ROLL:
    {
        euler_angles_t angle = {.roll = 0};
        bool state = Sensor_GetState(data->senscmd.sensor) == SENSOR_WORKING;
        if (state){
            Sensor_GetValue(data->senscmd.sensor, &angle);
            cb(DONE_OK, angle.roll);
        } else {
            cb(ERR, angle.roll);
        }
        break;
    }

    case GET_PITCH:
        /* code */
        break;

    case GET_HEADING:
        /* code */
        break;

    case GET_TEMP:
        /* code */
        break;

    case DEINIT:
        Sensor_SetState(data->senscmd.sensor, SENSOR_DISABLE);
        break;
    
    default:
        break;
    }
}
