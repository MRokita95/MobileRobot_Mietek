#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdint.h>
#include "sensors_common.h"
#include "imu.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"


typedef enum{
    UNCALIB,
    CALIB_REQUESTED,
    CALIB_OK,
    CALIB_FAILED,
}calibration_state;

typedef struct{
    calibration_state state;
    uint16_t samples;
    status_notif_cb status_notif;
}sensor_calibration_t;

/**
 * @brief Base Class Sensor
 * 
 */
typedef struct {
    sensors_id_t sensor_id;
    uint32_t address;
    struct data_handling_vtable const *vptr;
    SemaphoreHandle_t data_access_sem;
    void* sensor_handle;
    uint16_t data_size;
    bool init;
    sensor_calibration_t calib_handle;
} Sensor_t;


typedef void (*sensor_data_update)(const Sensor_t* me);
typedef void (*sensor_data_access)(const Sensor_t* me, void* value, bool write_mode);
typedef void (*sensor_calibration)(Sensor_t* me);

struct data_handling_vtable{
	sensor_data_access data_access;
	sensor_data_update data_update;
    sensor_calibration calibration;
};




#endif
