#ifndef TACSERVOCONTROLLER_H
#define TACSERVOCONTROLLER_H

#include "ControllerModule/actcontroller.h"
#include <map>
#include "UtilModule/msgcontenttype.h"
#include <fstream>
#include "TacModule/contactdetector.h"


class TacServoController : public ActController
{
public:
    TacServoController(ParameterManager &p);
    //hold the object in its current pose
    void update_robot_reference(Robot *);
    //move the object to the desired reference designed by Task
    void update_robot_reference(Robot *, Task *){}
    //myrmex tactile servo controller
    void update_robot_reference(Robot *, Task *, myrmex_msg *);
    //mid tactile servo controller
    void update_robot_reference(Robot *, Task *,FingertipTac *);
    void update_robot_reference(Robot *, Task *, Eigen::VectorXd, RobotState*){}
    void update_robot_reference(Robot *, Task *, Eigen::Vector3d, RobotState* rs){}
    void update_robot_reference(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState* ){}
    //myrmex tool tactile servo controller
    void update_robot_reference(ManipTool *, Robot *, Task *,myrmex_msg *);
    void update_controller_para(Eigen::Vector3d,PROTaskNameT);
    void update_controller_para(std::pair<Eigen::Vector3d,double>&,PROTaskNameT){}
    void update_controller_para(Eigen::Vector3d vel,Eigen::Vector3d r_vel,PROTaskNameT tnt){}
    void update_controller_para_stiffness();
    void get_desired_lv(Robot *, Task *){}
    //myrmex tactile servo computing the desired velocity
    void get_desired_lv(Robot *, Task *, myrmex_msg *);
    //mid tactile servo computing the desird vel
    void get_desired_lv(Robot *, Task *, FingertipTac *);
    //myrmex tool tactile servo computing the desird vel
    void get_desired_lv(ManipTool *, Robot *, Task *,myrmex_msg *);
    void get_desired_lv(Robot *, Task *, Eigen::VectorXd kukaft,RobotState*){}
    void get_desired_lv(Robot *, Task *, Eigen::Vector3d axis_dir,RobotState*){}
    void get_desired_lv(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState* rs){}
    void updateTacServoCtrlParam(TACTaskNameT);
    void updateProServoCtrlParam(PROTaskNameT tnt){}
    void updateForceServoCtrlParam(FORCETaskNameT fnt){}
    void get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov);
    void set_pm(ParameterManager &p);
    void set_init_TM(Eigen::Matrix3d tm) {m_init_tm = tm;pose_o_eigen_l.setZero();}

private:
    //!select matrix
    std::map<TACTaskNameT, Eigen::MatrixXd> sm;
    //!task jacobian matrix
    std::map<TACTaskNameT, Eigen::MatrixXd> tjkm;
    //!pose kp parameter
    std::map<TACTaskNameT, Eigen::MatrixXd> Kpp;
    //!pose ki parameter
    std::map<TACTaskNameT, Eigen::MatrixXd> Kpi;
    //!pose kd parameter
    std::map<TACTaskNameT, Eigen::MatrixXd> Kpd;
    //!orien increamental matrix
    std::map<TACTaskNameT, Eigen::MatrixXd> Kop;
    Eigen::VectorXd deltape;
    Eigen::Vector3d llv_tac,lov_tac;
    void initTacServoCtrlParam(TACTaskNameT);
    ContactDetector *cdet_ptr;

public:
    Eigen::VectorXd deltais;
    Eigen::VectorXd deltais_int;
    Eigen::VectorXd deltais_old;
    Eigen::VectorXd delta_obj_int;
    Eigen::VectorXd delta_obj_old;
    Eigen::VectorXd delta_obj_int_o;
    Eigen::VectorXd delta_obj_old_o;
};

#endif // TACSERVOCONTROLLER_H
