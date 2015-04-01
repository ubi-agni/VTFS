#include "forceservotask.h"

ForceServoTask::ForceServoTask(FORCETaskNameT taskname)
{
    curtaskname.forcet = taskname;
    desired_cf_kuka = 0.5;
}
void ForceServoTask::switchtotask(FORCETaskNameT taskname){
    curtaskname.forcet = taskname;
}


void ForceServoTask::set_desired_cf_kuka(double cf){
    desired_cf_kuka = cf;
}
