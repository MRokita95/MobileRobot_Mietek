#include "monitoring.h"
#include "rob_monitoring.h"
#include "events.h"
#include "rob_reaction.h"

#define ACTION_CHECKS_CNT 1u
#define PARAM_CHECKS_CNT 1u

static check_action_t checks_action[ACTION_CHECKS_CNT] = {
    [0] = {
        .event_notif = EVENT_ROBOT_STUCKED,
        .condition = RobotMon_Stuck_Condition,
        .action = RobotMon_Stuck_Check
    }
};

static check_param_t checks_param[PARAM_CHECKS_CNT];
 


void Monitoring_Init(){

    Event_Register(EVENT_ROBOT_STUCKED, RobotReact_Stop);
}


void Monitoring_Execute(){

    for (uint8_t check_idx = 0u; check_idx < ACTION_CHECKS_CNT; check_idx++){

        if (checks_action[check_idx].condition()){

            if (checks_action[check_idx].action()){

                Event_Notif(checks_action[check_idx].event_notif);
            }
        }
    }
}
