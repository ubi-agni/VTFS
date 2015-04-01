#include "tacservotask.h"

TacServoTask::TacServoTask(TACTaskNameT taskname)
{
    curtaskname.tact = taskname;
    desired_cp_myrmex[0] = 8.0;
    desired_cp_myrmex[1] = 8.0;
    desired_cf_myrmex = 0.1;
}
void TacServoTask::switchtotask(TACTaskNameT taskname){
    curtaskname.tact = taskname;
}

void TacServoTask::set_desired_cp_myrmex(double *cp){
    desired_cp_myrmex[0] = cp[0];
    desired_cp_myrmex[1] = cp[1];
}

void TacServoTask::set_desired_cf_myrmex(double cf){
    desired_cf_myrmex = cf;
}
