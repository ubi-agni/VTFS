#ifndef VISUOSERVOTASK_H
#define VISUOSERVOTASK_H
#include "task.h"

class VisuoServoTask : public Task
{
public:
    VisuoServoTask(VISTaskNameT);
    Eigen::Vector3d get_initial_p_eigen(){return initial_p_eigen;}
    Eigen::Vector3d get_desired_p_eigen() {return desired_p_eigen;}
    Eigen::Matrix3d get_desired_o_eigen(){return desired_o_eigen;}
    Eigen::Vector3d get_desired_o_ax(){return desired_o_ax;}
    void set_desired_p_eigen(Eigen::Vector3d p) {desired_p_eigen =  p;}
    void set_initial_p_eigen(Eigen::Vector3d p){initial_p_eigen = p;}
    void set_desired_o_eigen(Eigen::Matrix3d o_eigen){desired_o_eigen = o_eigen;}
    void set_desired_o_ax(Eigen::Vector3d o_ax){desired_o_ax = o_ax;}
    void set_desired_axis_dir(Eigen::Vector3d ax_dir){}
    void set_desired_surf_nv(Eigen::Vector3d surf_nv){}
    void set_contact_frame(Eigen::Matrix3d){}
    void set_desired_cp_myrmex(double *){}
    void set_desired_cf_myrmex(double){}
    void set_desired_orien_myrmex(double){}
    void set_desired_cf_kuka(double){}
    void set_desired_taxel_mid(int){}
    void set_desired_position_mid(Eigen::Vector3d){}
    void get_desired_position_mid(Eigen::Vector3d &){}
    void set_desired_nv_mid(Eigen::Vector3d){}
    void get_desired_nv_mid(Eigen::Vector3d &){}
    void set_desired_cf_mid(double){}
    void switchtotask(VISTaskNameT p);
    void switchtoglobalframe();
    void switchtolocalframe();
    void set_taxelfb_type_mid(TacFBType type){}
    void set_desired_cp_moving_dir(double x, double y){}
    void set_desired_rotation_range(double,double,double){}
    Eigen::Vector3d get_desired_rotation_range();
    double get_desired_mv_dis();
    void set_desired_mv_dis(double s);
};

#endif // VISUOSERVOTASK_H
