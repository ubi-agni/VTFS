#ifndef TACSERVOTASK_H
#define TACSERVOTASK_H

#include "task.h"

class TacServoTask : public Task
{
public:
    TacServoTask(TACTaskNameT taskname);
    Eigen::Vector3d get_desired_p_eigen() {return desired_p_eigen;}
    Eigen::Vector3d get_initial_p_eigen() {return initial_p_eigen;}
    Eigen::Matrix3d get_desired_o_eigen(){return desired_o_eigen;}
    Eigen::Vector3d get_desired_o_ax(){return desired_o_ax;}
    void set_desired_p_eigen(Eigen::Vector3d p) {desired_p_eigen =  p;}
    void set_initial_p_eigen(Eigen::Vector3d p) {}
    void set_desired_o_eigen(Eigen::Matrix3d o_eigen){desired_o_eigen = o_eigen;}
    void set_desired_o_ax(Eigen::Vector3d o_ax){desired_o_ax = o_ax;}
    void set_desired_cp_myrmex(double *);
    void set_desired_cf_myrmex(double);
    void set_desired_cf_kuka(double){}
    void get_desired_cp_myrmex(double *cp){cp[0] = desired_cp_myrmex[0];cp[1] = desired_cp_myrmex[1];}
    void get_desired_cf_myrmex(double& cf_myrmex){cf_myrmex = desired_cf_myrmex;}
    void get_desired_cf_kuka(double& cf_kuka){cf_kuka = desired_cf_kuka;}
    void set_desired_taxel_mid(int);
    void set_desired_position_mid(Eigen::Vector3d);
    void get_desired_position_mid(Eigen::Vector3d &);
    void set_desired_nv_mid(Eigen::Vector3d);
    void get_desired_nv_mid(Eigen::Vector3d &);
    void get_desired_taxel_mid(int &id){id = act_taxel_id;}
    void set_desired_cf_mid(double);
    void get_desired_cf_mid(double &p){p = taxel_pressure;}
    void switchtotask(TACTaskNameT taskname);
    void set_taxelfb_type_mid(TacFBType type);
    //set the desired contact point motion direction while the tactool sliding
    //on the unknown corner
    void set_desired_cp_moving_dir(double x, double y);
    //set the desired rotation range of end-effector in euler representation(in rad)
    void set_desired_rotation_range(double,double,double);
    double dir_x;
    double dir_y;
    Eigen::Vector3d desired_pose_range;
private:
    double desired_cp_myrmex[2];
    double desired_cf_myrmex;
    double desired_cf_kuka;
    int act_taxel_id;
    double taxel_pressure;
    Eigen::Vector3d desired_cp_mid;
    Eigen::Vector3d desired_nv_mid;

};

#endif // TACSERVOTASK_H
