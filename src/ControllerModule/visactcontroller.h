#ifndef VISACTCONTROLLER_H
#define VISACTCONTROLLER_H
#include "ControllerModule/actcontroller.h"
#include "RobotModule/Robot.h"
#include "TaskModule/visservotask.h"


class VisActController : public ActController
{
public:
    VisActController(ParameterManager &p);
    //hold the object in its current pose
    void update_robot_reference(Robot *){};
    //move the object to the desired reference designed by Task
    void update_robot_reference(Robot *, Task *){};
    void update_robot_reference(Robot *, Task *,myrmex_msg *){}
    void update_robot_reference(Robot *, Task *,FingertipTac *){}
    void update_robot_reference(Robot *, Task *, Eigen::VectorXd, RobotState*){}
    void update_robot_reference(Robot *, Task *, Eigen::Vector3d, RobotState* rs){}
    void update_robot_reference(ManipTool *, Robot *, Task *,myrmex_msg *){}
    void update_robot_reference(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState*);
    void update_controller_para(Eigen::Vector3d,PROTaskNameT){}
    void update_controller_para(std::pair<Eigen::Vector3d,double>&,PROTaskNameT){}
    void update_controller_para(Eigen::Vector3d vel,Eigen::Vector3d r_vel,PROTaskNameT tnt){}
    void update_controller_para_stiffness();
    void updateTacServoCtrlParam(TACTaskNameT){}
    void updateProServoCtrlParam(PROTaskNameT tnt){}
    void updateForceServoCtrlParam(FORCETaskNameT fnt){}
    void updateVisServoCtrlParam(VISTaskNameT tnt);
    void set_pm(ParameterManager &p);
    void get_desired_lv(Robot *, Task *){}
    void get_desired_lv(Robot *, Task *, myrmex_msg *){}
    void get_desired_lv(Robot *, Task *, FingertipTac *){}
    void get_desired_lv(Robot *, Task *, Eigen::VectorXd kukaft,RobotState*){}
    void get_desired_lv(Robot *, Task *, Eigen::Vector3d axis_dir,RobotState*){}
    void get_desired_lv(ManipTool *, Robot *, Task *,myrmex_msg *){}
    void get_desired_lv(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState* rs);
    void get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov);
    void set_init_TM(Eigen::Matrix3d tm) {m_init_tm = tm;}
private:
    void get_joint_position();
    void set_eff_command(Eigen::Vector3d p, Eigen::Vector3d o);
    void set_eff_command(Eigen::Vector3d p, Eigen::Matrix3d o);
    void initVisServoCtrlParam(VISTaskNameT);
    //!select matrix
    std::map<VISTaskNameT, Eigen::MatrixXd> vsm;
    //!pose kp parameter
    std::map<VISTaskNameT, Eigen::MatrixXd> Kpp;
    //!pose kp parameter
    std::map<VISTaskNameT, Eigen::Matrix3d> Kop;
    Eigen::Vector3d llv_vis,lov_vis;
    Eigen::VectorXd lv_vis;
    int counter;
};

#endif // VISACTCONTROLLER_H
