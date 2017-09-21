#include "forceservotask.h"

ForceServoTask::ForceServoTask(FORCETaskNameT taskname)
{
    curtaskname.forcet = taskname;
    desired_cf_kuka = 0.5;
    desired_pose_range.setZero();
    desired_dis = 0;
}
void ForceServoTask::switchtotask(FORCETaskNameT taskname){
    curtaskname.forcet = taskname;
}


void ForceServoTask::set_desired_cf_kuka(double cf){
    desired_cf_kuka = cf;
}

Eigen::Vector3d ForceServoTask::get_desired_rotation_range(){
    Eigen::Vector3d des;
    des.setZero();
    des = desired_pose_range;
    return des;
}

double ForceServoTask::get_desired_mv_dis(){
    return desired_dis;
}

void ForceServoTask::set_desired_mv_dis(double s){
    desired_dis =s;
}