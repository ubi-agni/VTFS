#ifndef FORCESERVOCONTROLLER_H
#define FORCESERVOCONTROLLER_H
#include "actcontroller.h"
#include "UtilModule/msgcontenttype.h"
#include <fstream>
#include "UtilModule/TemporalSmoothingFilter.h"

class ForceServoController : public ActController
{
public:
    ForceServoController(ParameterManager &p);
    //hold the object in its current pose
    void update_robot_reference(Robot *){}
    //move the object to the desired reference designed by Task
    void update_robot_reference(Robot *, Task *){}
    void update_robot_reference(Robot *, Task *, myrmex_msg *){}
    void update_robot_reference(Robot *, Task *,FingertipTac *){}
    void update_robot_reference(Robot *, Task *, Eigen::VectorXd, RobotState*);
    void update_robot_reference(Robot *, Task *, Eigen::Vector3d, RobotState* rs){}
    void update_robot_reference(ManipTool *, Robot *, Task *,myrmex_msg *){}
    void update_robot_reference(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState*){}
    void update_controller_para(Eigen::Vector3d,PROTaskNameT){}
    void update_controller_para(std::pair<Eigen::Vector3d,double>&,PROTaskNameT){}
    void update_controller_para(Eigen::Vector3d vel,Eigen::Vector3d r_vel,PROTaskNameT tnt){}
    void update_controller_para_stiffness(){}
    void get_desired_lv(Robot *, Task *){}
    void get_desired_lv(Robot *, Task *, myrmex_msg *){}
    void get_desired_lv(Robot *, Task *, FingertipTac *){}
    void get_desired_lv(Robot *, Task *, Eigen::VectorXd kukaft,RobotState*);
    void get_desired_lv(Robot *, Task *, Eigen::Vector3d axis_dir,RobotState*){}
    void get_desired_lv(ManipTool *, Robot *, Task *,myrmex_msg *){}
    void get_desired_lv(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState* rs){}
    void updateTacServoCtrlParam(TACTaskNameT){}
    void updateProServoCtrlParam(PROTaskNameT tnt){}
    void updateForceServoCtrlParam(FORCETaskNameT fnt);
    void get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov){}
    void set_pm(ParameterManager &p){}
    void set_init_TM(Eigen::Matrix3d tm) {m_init_tm = tm;pose_o_eigen_l.setZero();}
private:
//    void initForceServoCtrlParam(ForceTaskNameT);
    //!select matrix
    std::map<FORCETaskNameT, Eigen::MatrixXd> sm;
    //!task jacobian matrix
    std::map<FORCETaskNameT, Eigen::MatrixXd> tjkm;
    //!pose kp parameter
    std::map<FORCETaskNameT, Eigen::MatrixXd> Kpp;
    //!pose ki parameter
    std::map<FORCETaskNameT, Eigen::MatrixXd> Kpi;
    //!pose kd parameter
    std::map<FORCETaskNameT, Eigen::MatrixXd> Kpd;
    //!orien increamental matrix
    std::map<FORCETaskNameT, Eigen::MatrixXd> Kop;
    void initForceServoCtrlParam(FORCETaskNameT tnt);
    Eigen::VectorXd deltape;
    Eigen::Vector3d llv_f,lov_f;
public:
    Eigen::VectorXd deltais;
    Eigen::VectorXd deltais_int;
    Eigen::VectorXd deltais_old;
    Eigen::VectorXd delta_obj_int;
    Eigen::VectorXd delta_obj_old;
    Eigen::VectorXd delta_obj_int_o;
    Eigen::VectorXd delta_obj_old_o;
    TemporalSmoothingFilter<Eigen::Vector3d>* lv_filter;
    std::ofstream vel_rec2;
};

#endif // FORCESERVOCONTROLLER_H
