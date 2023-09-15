/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

#include "task_workers.h"
#include "robot.h"
#include "imu_handle.h"
#include "robot_app.h"
#include "comm.h"
#include "sensors_common.h"
#include "param_handle.h"
#include "commands.h"
#include "tracing.h"


#define CALIB_CNT     100u
#define MESSAGE_LENGTH 120u



typedef struct
{
	const char *timer_name;
	uint32_t period_ms;
	UBaseType_t auto_reload;
	uint8_t ID;
	TimerCallbackFunction_t timer_handle;
	StaticTimer_t buffer;
} Timer_t;


typedef struct
{
	char *task_name;
	uint8_t task_active;
	osSemaphoreId activ_semaphore;
	TaskFunction_t task_function;
	UBaseType_t priority;
    uint16_t stack_size;
    TickType_t frequency;
} Task_t;

void Robot_Application1(void);
void vTask_Robot(void const * argument);
void vTask_Communication(void const * argument);
void vTask_Sensors(void const * argument);
void vTask_Management(void const * argument);



char message_buffer[MESSAGE_LENGTH];

#define TIMER_NUMBERS 2u
#define EULER_TIMER 0u



static Task_t Tasks[TASK_NUMBERS] =
{
	[ROBOT_TASK] = {
				.task_name = "Robot Task",
				.task_active = 1,
				.task_function = vTask_Robot,
				.priority = osPriorityRealtime,
                .stack_size = configMINIMAL_STACK_SIZE+150,
                .frequency = ROB_TASK_FREQUENCY
		},

    [COMM_TASK] = {
				.task_name = "Comm Task",
				.task_active = 1,
				.task_function = vTask_Communication,
				.priority = osPriorityAboveNormal,
                .stack_size = configMINIMAL_STACK_SIZE,
                .frequency = COMM_TASK_FREQUENCY
		},

    [SENSOR_TASK] = {
				.task_name = "Sensors Task",
				.task_active = 1,
				.task_function = vTask_Sensors,
				.priority = osPriorityHigh,
                .stack_size = configMINIMAL_STACK_SIZE+125,
                .frequency = SENSOR_TASK_FREQUENCY
		},

    [MANAGE_TASK] = {
				.task_name = "Management Task",
				.task_active = 1,
				.task_function = vTask_Management,
				.priority = osPriorityNormal,
                .stack_size = configMINIMAL_STACK_SIZE,
                .frequency = MANAG_TASK_FREQUENCY
		}
};


IMU_Handle_t imu_sensor = NULL;
IMU_ReturnCode_t calibration_status;

Mobile_Platform_t robot;


void TasksWorkers_Init(){

    for (uint8_t task_idx = 0; task_idx < TASK_NUMBERS; task_idx ++){

      if (Tasks[task_idx].task_active != 0) {

        BaseType_t TASK_OK = xTaskCreate(Tasks[task_idx].task_function,
          Tasks[task_idx].task_name,
          Tasks[task_idx].stack_size,
          ( void * ) NULL,
          Tasks[task_idx].priority,
          NULL );

        assert_param(pdPASS == TASK_OK);
      }
  }

  /* WOrkers Initialization */
  Comm_Init();
  Param_Initialize();
  Robot_Init(&robot);
  imu_sensor = Sensor_Init(IMU);
  Trace_InitAccessInstances(&robot)
}


void vTask_Communication(void const * argument){


	TickType_t xNextWakeTime;

	const TickType_t xBlockTime = Tasks[COMM_TASK].frequency/portTICK_RATE_MS;
	xNextWakeTime = xTaskGetTickCount();

    for(;;){

    	vTaskDelayUntil( &xNextWakeTime, xBlockTime );
      
        Comm_Task();
    }
}


void vTask_Sensors(void const * argument) {

      

    TickType_t xNextWakeTime;

    const TickType_t xBlockTime = Tasks[SENSOR_TASK].frequency/portTICK_RATE_MS;
    xNextWakeTime = xTaskGetTickCount();

    for(;;){

      vTaskDelayUntil( &xNextWakeTime, xBlockTime );

      Sensor_Task();
  }
}


void vTask_Robot(void const * argument){


  TickType_t xNextWakeTime;

  const TickType_t xBlockTime = Tasks[ROBOT_TASK].frequency/portTICK_RATE_MS;
  xNextWakeTime = xTaskGetTickCount();

  for(;;){

	vTaskDelayUntil( &xNextWakeTime, xBlockTime );

	Robot_UpdateMotionStatus(&robot);

	Robot_Task(&robot);

  }
}


void vTask_Management(void const * argument) {
      

    TickType_t xNextWakeTime;

    const TickType_t xBlockTime = Tasks[MANAGE_TASK].frequency/portTICK_RATE_MS;
    xNextWakeTime = xTaskGetTickCount();

    for(;;){

      vTaskDelayUntil( &xNextWakeTime, xBlockTime );

      Management_Task();

	  /* Perform trace info gathering */
      Trace_PullData();
  }
}


void Robot_Application1()
{
	static int i = 0;
	if (i == 0){
		ROBOT_WAIT(2000);
		ROBOT_MOVE_TO_POINT(200, -800, -800);
		ROBOT_WAIT(2000);
		ROBOT_MOVE_TO_POINT(200, -800, -1000);
		ROBOT_WAIT(2000);
		ROBOT_MOVE_TO_POINT(300, 800, 1000);
		i = 1;
	}

}
