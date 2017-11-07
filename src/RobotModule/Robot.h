#ifndef ROBOT_H
#define ROBOT_H

#include <kdl/chain.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/chainiksolver.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <UtilModule/kdl_to_eigen.h>
#include <Eigen/Dense>
#include "ControllerModule/CtrlParam.h"
#include "ControllerModule/actcontroller.h"
#include "UtilModule/RebaType.h"

#include <cbf/types.h>
#include <cbf/primitive_controller.h>
#include <cbf/potential.h>
#include <cbf/square_potential.h>
#include <cbf/axis_angle_potential.h>
#include <cbf/composite_potential.h>
#include <cbf/composite_transform.h>
#include <cbf/config.h>
#include <cbf/kdl_transforms.h>
#include <cbf/identity_transform.h>
#include <cbf/generic_transform.h>
#include <cbf/dummy_resource.h>
#include <cbf/dummy_reference.h>

//#include <cbf/cddyn_filter.h>
//#include <cbf/bypass_filter.h>
//#include <cbf/error_controllers.h>

class ActController;
class Robot
{
public:
    Robot();
    virtual void get_joint_position_act() = 0;
    virtual void get_joint_position_mea()= 0;
    virtual void get_joint_position_mea(double *) = 0;
    virtual void update_robot_state() = 0;
    virtual void update_cbf_controller() = 0;
    virtual void set_joint_command() = 0;
    virtual void set_joint_command(RobotModeT) = 0;
    virtual void update_robot_stiffness() = 0;
    virtual void update_robot_cp_stiffness(Eigen::VectorXd cps,Eigen::VectorXd cpd) = 0;
    virtual void update_robot_cp_exttcpft(Eigen::VectorXd ft) = 0;
    virtual void setAxisStiffnessDamping (double* s, double* d) = 0;
    virtual void switch2cpcontrol() = 0;
    virtual void switch2jntcontrol() = 0;
    virtual void request_monitor_mode() = 0;
    virtual void no_move() = 0;
    virtual void waitForFinished() = 0;
    virtual RobotNameT get_robotname() = 0;
    virtual double gettimecycle() = 0;
    virtual void calibForce(int st) = 0;
    virtual void getTcpFtCalib (Eigen::Vector3d &cf) = 0;
    void set_cart_command(double *);
    virtual void set_init_TM(Eigen::Matrix3d tm) = 0;
    virtual Eigen::Matrix3d get_init_TM() = 0;
    virtual void addSegmentinChain(Eigen::Matrix3d R,Eigen::Vector3d p) = 0;
    virtual void backKukaChain(ToolNameT tn) = 0;
    virtual void initCbf(ControllerT ctrl=LSSolution_Ctrl) = 0;
    Eigen::Vector3d get_cur_cart_p();
    Eigen::Matrix3d get_cur_cart_o();
    virtual Eigen::Vector3d get_cur_vel() = 0;
    KDL::Chain *baseToTool;
    KDL::Chain *worldToTool;
    KDL::ChainFkSolverPos_recursive* baseToToolFkSolver;
    KDL::ChainFkSolverPos_recursive* worldToToolFkSolver;
    KDL::ChainIkSolverVel_pinv* worldToToolIkSolver;
    KDL::ChainJntToJacSolver* worldToToolJacSolver;
    pthread_mutex_t primitiveControllerMutex;
    CBF::PrimitiveControllerPtr primitiveControllerP;
    CBF::SubordinateControllerPtr subordinateControllerP;
    CBF::DummyResourcePtr kukaResourceP;
    CBF::DummyReferencePtr currentTaskTargetP;
    CBF::DummyReferencePtr currentTaskReferenceP;
    CBF::DummyReferencePtr currentSubordinateTaskReferenceP;
    CBF::SquarePotentialPtr xyzSquarePotential;
    KDL::Jacobian Jac_kdl;
    KDL::Frame get_eef_pose();
    KDL::Frame get_seg_pose(int index);
    KDL::JntArray q;
    KDL::Frame pose;
    virtual void get_eef_ft(Eigen::Vector3d&,Eigen::Vector3d&) = 0;
    ToolNameT toolname;
protected:
    Eigen::Matrix3d m_init_tm;
    Eigen::Matrix3d m_TM_eigen;
    Eigen::Vector3d m_p_eigen;
    double cart_command[6];

public:
    Eigen::Vector3d eff_force;
    Eigen::Vector3d eff_torque;
    double pose_frombase[12];
};

#endif // ROBOT_H
