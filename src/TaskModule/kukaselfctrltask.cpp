#include "kukaselfctrltask.h"

void KukaSelfCtrlTask::switchtotask(PROTaskNameT tn){
    curtaskname.prot = tn;
}

void KukaSelfCtrlTask::switchtoglobalframe(){
    mft = GLOBAL;
}
void KukaSelfCtrlTask::switchtolocalframe(){
    mft = LOCAL;
}

KukaSelfCtrlTask::KukaSelfCtrlTask(PROTaskNameT tn)
{
    curtaskname.prot = tn;
    desired_pose_range.setZero();
    desired_dis = 0;
}


Eigen::Vector3d KukaSelfCtrlTask::get_desired_rotation_range(){
    Eigen::Vector3d des;
    des.setZero();
    des = desired_pose_range;
    return des;
}

double KukaSelfCtrlTask::get_desired_mv_dis(){
    return desired_dis;
}

void KukaSelfCtrlTask::set_desired_mv_dis(double s){
    desired_dis =s;
}