#include "tacservotask.h"

TacServoTask::TacServoTask(TACTaskNameT taskname)
{
    curtaskname.tact = taskname;
    desired_cp_myrmex[0] = 8.0;
    desired_cp_myrmex[1] = 8.0;
    desired_cf_myrmex = 0.1;
    act_taxel_id = 0;
    tft = TAXEL_NUM;
    desired_cp_mid.setZero();
    desired_nv_mid.setZero();
    dir_x = 0;
    dir_y = 0;
    desired_pose_range.setZero();
    desired_orien_myrmex = 0;
    desired_dis = 0;
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


void TacServoTask::set_desired_orien_myrmex(double orien){
    desired_orien_myrmex = orien;
}
void TacServoTask::set_desired_taxel_mid(int id){
    act_taxel_id = id;
}

void TacServoTask::set_desired_cf_mid(double p){
    taxel_pressure = p;
}

void TacServoTask::set_taxelfb_type_mid(TacFBType type){
    tft = type;
}

void TacServoTask::set_desired_position_mid(Eigen::Vector3d position){
    desired_cp_mid = position;
}
void TacServoTask::get_desired_position_mid(Eigen::Vector3d &position){
    position = desired_cp_mid;
}

void TacServoTask::set_desired_nv_mid(Eigen::Vector3d nv){
    desired_nv_mid = nv;
}
void TacServoTask::get_desired_nv_mid(Eigen::Vector3d &nv){
    nv = desired_nv_mid;
}

void TacServoTask::set_desired_cp_moving_dir(double x, double y){
    dir_x = x;
    dir_y = y;
}
void TacServoTask::set_desired_rotation_range(double r,double p,double y){
    desired_pose_range(0) = r;
    desired_pose_range(1) = p;
    desired_pose_range(2) = y;
}
Eigen::Vector3d TacServoTask::get_desired_rotation_range(){
    Eigen::Vector3d des;
    des.setZero();
    des = desired_pose_range;
    return des;
}

double TacServoTask::get_desired_mv_dis(){
    return desired_dis;
}

void TacServoTask::set_desired_mv_dis(double s){
    desired_dis =s;
}
