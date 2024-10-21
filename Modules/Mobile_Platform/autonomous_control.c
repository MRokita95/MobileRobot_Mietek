#include "state_handling.h"
#include "pid_handle.h"
#include "mobile_platform.h"

typedef struct autonomous_control
{
    pid_output_t speed_setpoint;
    pid_input_t distance_setpoint;
    pid_input_t distance_current;
    PID_handle_t pid_handle;
    PID_parameters_t pid_params;
    Mobile_Platform_t* robot_instance;
};

struct autonomous_control auto_handle = {
    .pid_params = {
        .max_output = 250,
        .integratorLimit = 500,
        .KP = 1.0,
        .KI = 1.0,
        .KD = 0.1,
    },
};


static void auto_control_init(void){
    auto_handle.pid_handle = PID_Init(&auto_handle.pid_params);
}

static void auto_control_execute(void){

    if (Robot_ActiveMode(auto_handle.robot_instance) == ORIENT_MODE){
        return;
    }
    
    pid_input_t current_speed = Robot_GetWheelSpeed(auto_handle.robot_instance, RIGHT);
    pid_output_t auto_handle.speed_setpoint = PID_Loop(&auto_handle.pid_handle, auto_handle.distance_current, auto_handle.distance_setpoint);


    Robot_SetSpeed(auto_handle.robot_instance, auto_handle.speed_setpoint);
}
