/*
 * complementary_filter.c
 *
 *  Created on: Sep 19, 2022
 *      Author: micha
 */

#include "filters.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <math.h>

#define G_acc 			981.0f
#define PI 				3.14f
#define RAD_TO_DEG		(180.0f / PI)
#define DEG_TO_RAD		(PI / 180.0f)

enum {
	x = 0,
	y,
	z,
};


SensorFilter_t filter_data = { .samples = 125, .alpha = 0.04, .dead_zone = 1 };

static int16_t* gyro_dead_zone(int16_t *speed, int16_t d_zone)
{
	static int16_t filtered_speed[3] = {0};

	for (uint8_t i = x; i <=z; i++){
		if (abs(speed[i]) > d_zone){
			filtered_speed[i] = speed[i];
		} else {
			filtered_speed[i] = 0;
		}
	}
	return filtered_speed;
}

static float yaw_calculation(int32_t *axis, float roll, float pitch)
{
    float Xheading = axis[x] * cosf(pitch) + axis[y] * sinf(roll) * sinf(pitch) + axis[z] * cosf(roll) * sinf(pitch);
    float Yheading = axis[y] * cosf(roll) - axis[z] * sinf(pitch);

    const float declination_shenzhen = -2.2;

    float heading = atan2f(Yheading, Xheading) + declination_shenzhen;

    return heading;
}


euler_angles_t ComplementaryFilter(int16_t *acc, int16_t *speed, int32_t *axis)
{
	euler_angles_t angles_estimate = {0};

	SensorFilter_t *filter = &filter_data;

	static uint8_t first_scan = 1u;
	float alpha = filter->alpha;
	if (first_scan == 1u){
		alpha = 1.0;
	}

	filter->acc_orient.roll = 	atanf((float)acc[y] / (float)acc[z]);
	filter->acc_orient.pitch = 	asinf((float)acc[x] / G_acc);

	int16_t *gyro_speed = gyro_dead_zone(speed, filter->dead_zone);

	filter->ang_speed.roll = 	(float)gyro_speed[x] * DEG_TO_RAD + sinf(filter->actual_orient.roll)*tanf(filter->actual_orient.pitch)*(float)gyro_speed[y] * DEG_TO_RAD
													+ cosf(filter->actual_orient.roll)*tanf(filter->actual_orient.pitch)*(float)gyro_speed[z] * DEG_TO_RAD;

	filter->ang_speed.pitch = 	(float)gyro_speed[y] * DEG_TO_RAD * cosf(filter->actual_orient.roll) - (float)gyro_speed[z] * DEG_TO_RAD *sinf(filter->actual_orient.roll);

	// delta time for integrator
	float dt = (filter->last_tick != 0) ?
			(float)HAL_GetTick()/(float)portTICK_PERIOD_MS - (float)filter->last_tick/(float)portTICK_PERIOD_MS :
			0.0;
	dt = dt / 1000;	//[s] unit

	filter->gyro_orient.roll = filter->actual_orient.roll +  dt*filter->ang_speed.roll;
	filter->gyro_orient.pitch = filter->actual_orient.pitch + dt*filter->ang_speed.pitch;


	filter->actual_orient.roll =  alpha*filter->acc_orient.roll + (1.0f - alpha)*filter->gyro_orient.roll;
	filter->actual_orient.pitch =  alpha*filter->acc_orient.pitch + (1.0f - alpha)*filter->gyro_orient.pitch;

	/*filter->actual_orient.yaw = yaw_calculation(axis,
												filter->actual_orient.roll,
												filter->actual_orient.pitch);*/


	angles_estimate.roll = filter->actual_orient.roll * RAD_TO_DEG;
	angles_estimate.pitch = filter->actual_orient.pitch * RAD_TO_DEG;
	angles_estimate.yaw = 0; //AK sensor not working for now
	//angles_estimate.yaw = filter->actual_orient.yaw * RAD_TO_DEG;

	first_scan = 0;
	filter->last_tick = HAL_GetTick();
	return angles_estimate;
}
