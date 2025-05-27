#include "monitoring.h"
#include "rob_monitoring.h"
#include "rob_reaction.h"
#include "string.h"

#define ACTION_CHECKS_CNT 1u
#define PARAM_CHECKS_CNT 2u


static check_action_t m_checks_action[ACTION_CHECKS_CNT] = {
    [0] = {
        .state = MON_ENABLED,
        .dev_id = ROBOT_DEVICE,
        .interval = 5,
        .condition[0] = RobotMon_OnMovement,
        .action = RobotMon_Stuck_Check,
        .reaction = RobotReact_Stop
    }
};

static check_param_t m_checks_param[PARAM_CHECKS_CNT] = {
    [0] = {
        .state = MON_ENABLED,
        .type = FLOAT_CHECK,
        .interval = 3,
        .param_id = 0,
        .low_limit.f_value = -20.f,
        .high_limit.f_value = 20.f,
        .condition[0] = RobotMon_OnMovement,
        .condition[1] = ImuMon_ActiveCheck,
        .getval = ImuMon_GetAngle,
        .reaction = RobotReact_StopAndSafeReturn,
    },
    [1] = {
        .state = MON_ENABLED,
        .type = FLOAT_CHECK,
        .interval = 3,
        .param_id = 1,
        .low_limit.f_value = -35.f,
        .high_limit.f_value = 35.f,
        .condition[0] = RobotMon_OnMovement,
        .condition[1] = ImuMon_ActiveCheck,
        .getval = ImuMon_GetAngle,
        .reaction = RobotReact_StopAndSafeReturn,
    }
};


static bool run_condition(conditional_cb* conditions){

    bool run_check = true;
    for (uint8_t cond_idx = 0; cond_idx < MAX_CONDITIONS_CNT; cond_idx++){
        conditional_cb cond = conditions[cond_idx];
        if (cond != NULL){
            if (!cond()){
                run_check = false;
            }
        }
    }
    return run_check;
}


void Monitoring_Init(){


}

void Monitorig_RegisterRobot(Mobile_Platform_t* robot){

    RobotMon_RegisterInstance(robot);
    RobotReact_RegisterInstance(robot);
}

void Monitoring_Execute(){

    //ACTIONS
    for (uint8_t check_idx = 0u; check_idx < ACTION_CHECKS_CNT; check_idx++){

        check_action_t* check = &m_checks_action[check_idx];

        if (check->state != MON_ENABLED){
            continue;
        }

        if (!run_condition(check->condition)){
            if (check->samples > 0){
                check->samples--;
            }
            continue;
        }


        if (check->action()){

            check->samples++;

            if (check->samples >= check->interval){

                check->reaction();
                check->samples = 0;
            }
        } else {
            
            if (check->samples > 0){
                check->samples--;
            }
        }
    }


    //PARAMS
    for (uint8_t check_idx = 0u; check_idx < PARAM_CHECKS_CNT; check_idx++){

        check_param_t* check = &m_checks_param[check_idx];

        if (check->state != MON_ENABLED){
            continue;
        }


        if (!run_condition(check->condition)){
            if (check->samples > 0){
                check->samples--;
            }
            continue;
        }
        

        check->current_value = check->getval(check->param_id);

        if (check->type == FLOAT_CHECK){
            if (check->current_value.f_value < check->low_limit.f_value ||
                check->current_value.f_value > check->high_limit.f_value ||
                check->avg_value.f_value < check->low_limit.f_value ||
                check->avg_value.f_value > check->high_limit.f_value
                ){

                    check->samples++;
            } else {
                if (check->samples > 0){
                    check->samples--;
                }
            }
        }

        if (check->samples >= check->interval){
            check->reaction();
            check->samples = 0;
            memset(&check->avg_value, 0, sizeof(union value));
            memset(&check->prev_value, 0, 2*sizeof(union value));
        }

        if (check->type == FLOAT_CHECK){
            check->avg_value.f_value = (check->prev_value[0].f_value + check->prev_value[1].f_value + check->current_value.f_value) / 3;
        }
        check->prev_value[0] = check->current_value;
        check->prev_value[1] = check->prev_value[0];
    }
}

void Monitoring_Setup(monitoring_state_t state){

    for (uint8_t check_idx = 0u; check_idx < ACTION_CHECKS_CNT; check_idx++){
        m_checks_action[check_idx].state = state;
        m_checks_action[check_idx].samples = 0;
    }

    for (uint8_t check_idx = 0u; check_idx < PARAM_CHECKS_CNT; check_idx++){
        m_checks_param[check_idx].state = state;
        m_checks_param[check_idx].samples = 0;
        memset(&m_checks_param[check_idx].avg_value, 0, sizeof(union value));
        memset(&m_checks_param[check_idx].prev_value, 0, 2*sizeof(union value));
    }
}
