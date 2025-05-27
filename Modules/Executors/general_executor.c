#include "logic_executor.h"
#include "application_defs.h"
#include "robot.h"
#include "sensors_common.h"
#include "robot_app.h"
#include "monitoring.h"
#include "param_handle.h"


bool GeneralExecutor_Ready(general_payload_t* data){

    return true;
    
}

void GeneralExecutor_Dispatch(general_payload_t* data, status_notif_cb cb){

    general_action_type action = data->gencmd.type;

    switch (action)
    {

    case GA_RESET_CMD_QUEUE:
        Command_ResetQueue(NORMAL_SEVERITY);
        cb(DONE_OK, 0);
        break;

    case GA_RESET_TRACE_QUEUE:
        Trace_ResetQueue();
        cb(DONE_OK, 0);
        break;

    case GA_START_MONITORINGS:
        Monitoring_Setup(MON_ENABLED);
        cb(DONE_OK, 0);
        break;

    case GA_STOP_MONITORINGS:
        Monitoring_Setup(MON_DISABLED);
        cb(DONE_OK, 0);
        break;

    case GA_SET_PARAM:
    {
        float fvalue = (float)data->gencmd.value;
        fvalue = fvalue / 100.f;
        param_ret_status_t sts = Param_Set(data->gencmd.param_id, &fvalue);
        if (sts == PARAM_OK){
            cb(DONE_OK, 0);
        } else {
            cb(ERR, 0);
        }
    }
    
    default:
        break;
    }


}
