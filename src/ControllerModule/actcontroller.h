#ifndef ACTCONTROLLER_H
#define ACTCONTROLLER_H
#include "RobotModule/Robot.h"
#include "TaskModule/task.h"
#include "UtilModule/Util.h"
#include <utility>

#include "UtilModule/msgcontenttype.h"
#include "parametermanager.h"
#include "RobotModule/RobotState.h"
#include "TacModule/fingertiptac.h"

class Robot;
class RobotState;
class ActController
{
public:
    ActController(ParameterManager& );
    virtual void update_robot_reference(Robot *) = 0;
    virtual void update_robot_reference(Robot *, Task *) = 0;
    virtual void update_robot_reference(Robot *, Task *,myrmex_msg *) = 0;
    virtual void update_robot_reference(Robot *, Task *,FingertipTac *) = 0;
    virtual void update_robot_reference(Robot *, Task *, Eigen::VectorXd, RobotState* rs) = 0;
    virtual void update_controller_para(Eigen::Vector3d,PROTaskNameT) = 0;
    virtual void update_controller_para(std::pair<Eigen::Vector3d,double>&,PROTaskNameT) = 0;
    virtual void update_controller_para(Eigen::Vector3d vel,Eigen::Vector3d r_vel,PROTaskNameT tnt) = 0;
    virtual void update_controller_para_stiffness() = 0;
    virtual void updateTacServoCtrlParam(TACTaskNameT) = 0;
    virtual void updateProServoCtrlParam(PROTaskNameT tnt) = 0;
    virtual void updateForceServoCtrlParam(FORCETaskNameT fnt) = 0;
    virtual void set_init_TM(Eigen::Matrix3d tm) = 0;
    virtual void get_desired_lv(Robot *, Task *) = 0;
    virtual void get_desired_lv(Robot *, Task *, myrmex_msg *) = 0;
    virtual void get_desired_lv(Robot *, Task *, FingertipTac *) = 0;
    virtual void get_desired_lv(Robot *, Task *, Eigen::VectorXd kukaft,RobotState*)=0;
    virtual void get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov) = 0;
    virtual void set_pm(ParameterManager &p) = 0;
    void local_to_global(const Eigen::Vector3d p_in, const Eigen::Matrix3d o_in,\
                         const Eigen::Vector3d lv, const Eigen::Vector3d ov,\
                        Eigen::Vector3d& p_out, Eigen::Vector3d& o_out);

    Eigen::Matrix3d m_init_tm;
    void set_llv_limit(Eigen::Vector3d lv){m_llv_limit = lv;}
    Eigen::Vector3d get_llv_limit(){return m_llv_limit;}
    void set_euler_limit(Eigen::Vector3d euler){m_euler_limit = euler;}
    Eigen::Vector3d get_euler_limit(){return m_euler_limit;}
    Eigen::Vector3d pose_o_eigen_l;
    void limit_vel(Eigen::Vector3d,\
                   Eigen::Vector3d&, Eigen::Vector3d&);
    void limit_eef_euler(Eigen::Vector3d lim);
    static Eigen::Vector3d llv,lov;
    ParameterManager pm;
protected:
    double cart_command[6];
    Eigen::Vector3d glv,gov;
    Eigen::Vector3d eff_p_command, eff_o_command_vec;
    Eigen::Matrix3d eff_o_command_mat;
    Eigen::Vector3d m_llv_limit;
    Eigen::Vector3d m_euler_limit;


};


#endif // ACTCONTROLLER_H
