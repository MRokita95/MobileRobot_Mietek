#include "monitoring.h"
#include "rob_monitoring.h"
#include "rob_reaction.h"

#define ACTION_CHECKS_CNT 1u
#define PARAM_CHECKS_CNT 1u


static check_action_t checks_action[ACTION_CHECKS_CNT] = {
    [0] = {
        .state = MON_ENABLED,
        .dev_id = ROBOT_DEVICE,
        .condition = RobotMon_Stuck_Condition,
        .action = RobotMon_Stuck_Check,
        .init = RobotMon_RegisterInstance,
        .init2 = RobotReact_RegisterInstance,
        .reaction = RobotReact_Stop
    }
};

static check_param_t checks_param[PARAM_CHECKS_CNT];
 


void Monitoring_Init(){


}

void Monitorig_RegisterRobot(Mobile_Platform_t* robot){


    for (uint8_t check_idx = 0u; check_idx < ACTION_CHECKS_CNT; check_idx++){
        if (checks_action[check_idx].dev_id == ROBOT_DEVICE){
            checks_action[check_idx].init(robot);
            checks_action[check_idx].init2(robot);
        }
    }
}

void Monitoring_Execute(){

    for (uint8_t check_idx = 0u; check_idx < ACTION_CHECKS_CNT; check_idx++){

        if (checks_action[check_idx].state != MON_ENABLED || !checks_action[check_idx].condition()){
            continue;
        }
        checks_action[check_idx].samples++;

        if (checks_action[check_idx].samples >= checks_action[check_idx].interval){

            if (checks_action[check_idx].action()){

                checks_action[check_idx].reaction();
            }
            checks_action[check_idx].samples = 0;
        }
    }
}

void Monitoring_Setup(monitoring_state_t state){

    for (uint8_t check_idx = 0u; check_idx < ACTION_CHECKS_CNT; check_idx++){
        checks_action[check_idx].state = state;
    }
}
