#include "visservotask.h"

void VisuoServoTask::switchtotask(VISTaskNameT tn){
    curtaskname.vist = tn;
}

void VisuoServoTask::switchtoglobalframe(){
    mft = GLOBAL;
}
void VisuoServoTask::switchtolocalframe(){
    mft = LOCAL;
}

VisuoServoTask::VisuoServoTask(VISTaskNameT tn)
{
    curtaskname.vist = tn;
    desired_pose_range.setZero();
    desired_dis = 0;
}

Eigen::Vector3d VisuoServoTask::get_desired_rotation_range(){
    Eigen::Vector3d des;
    des.setZero();
    des = desired_pose_range;
    return des;
}

double VisuoServoTask::get_desired_mv_dis(){
    return desired_dis;
}

void VisuoServoTask::set_desired_mv_dis(double s){
    desired_dis =s;
}