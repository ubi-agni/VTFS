#ifndef PROACTCONTROLLER_H
#define PROACTCONTROLLER_H
#include "actcontroller.h"
#include "RobotModule/Robot.h"
#include "TaskModule/kukaselfctrltask.h"
#include "dmp/representation/dmp/umitsmp.h"
#include <chrono>



class ProActController : public ActController
{
public:
    ProActController(ParameterManager &p);
    ProActController(ParameterManager &p, const std::string& dmpFileName);
    //hold the object in its current pose
    void update_robot_reference(Robot *);
    //move the object to the desired reference designed by Task
    void update_robot_reference(Robot *, Task *);
    void update_robot_reference(Robot *, Task *,myrmex_msg *){}
    void update_robot_reference(Robot *, Task *,FingertipTac *){}
    void update_robot_reference(Robot *, Task *, Eigen::VectorXd, RobotState*){}
    void update_robot_reference(Robot *, Task *, Eigen::Vector3d, RobotState* rs){}
    void update_robot_reference(ManipTool *, Robot *, Task *,myrmex_msg *){}
    void update_robot_reference(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState*);
    void update_controller_para(Eigen::Vector3d,PROTaskNameT);
    void update_controller_para(std::pair<Eigen::Vector3d,double>&,PROTaskNameT);
    void update_controller_para(Eigen::Vector3d vel,Eigen::Vector3d r_vel,PROTaskNameT tnt);
    void update_controller_para_stiffness();
    void updateTacServoCtrlParam(TACTaskNameT){}
    void updateProServoCtrlParam(PROTaskNameT tnt);
    void updateForceServoCtrlParam(FORCETaskNameT fnt){}
    void set_pm(ParameterManager &p);
    void get_desired_lv(Robot *, Task *);
    void get_desired_lv(Robot *, Task *, myrmex_msg *){}
    void get_desired_lv(Robot *, Task *, FingertipTac *){}
    void get_desired_lv(Robot *, Task *, Eigen::VectorXd kukaft,RobotState*){}
    void get_desired_lv(Robot *, Task *, Eigen::Vector3d axis_dir,RobotState*){}
    void get_desired_lv(ManipTool *, Robot *, Task *,myrmex_msg *){}
    void get_desired_lv(Robot *, Task *,Eigen::Vector3d,Eigen::Vector3d, RobotState* rs);
    void get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov);
    void set_init_TM(Eigen::Matrix3d tm) {m_init_tm = tm; pose_o_eigen_l.setZero();}
private:
    void get_joint_position();
    void set_eff_command(Eigen::Vector3d p, Eigen::Vector3d o);
    void set_eff_command(Eigen::Vector3d p, Eigen::Matrix3d o);
    void initProServoCtrlParam(PROTaskNameT);
    //!select matrix
    std::map<PROTaskNameT, Eigen::MatrixXd> psm;
    //!pose kp parameter
    std::map<PROTaskNameT, Eigen::MatrixXd> Kpp;
    //!pose kp parameter
    std::map<PROTaskNameT, Eigen::Matrix3d> Kop;
    Eigen::Vector3d llv_pro,lov_pro;
    Eigen::VectorXd lv_pro;
    

public:
    double delta_ag, delta_ag_int;
    DMP::UMITSMPPtr dmpCtrl;
    double motionTimeDuration;
    double canVal;
    std::vector<DMP::DMPState> currentState;
    std::ofstream pro_data_store;
    Eigen::Vector3d init_position;
    double est_rot_angle;
    TemporalSmoothingFilter<Eigen::Vector3d>* lv_filter;
    
    double angleFilter;
    Eigen::Vector3d origForce;
    Eigen::Matrix3d contactFrame;

//   std::chrono::system_clock::time_point beginClock;
};

#endif // PROACTCONTROLLER_H
