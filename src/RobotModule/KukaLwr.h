#ifndef KUKALWR_H
#define KUKALWR_H

#include "Robot.h"
#include "UtilModule/RebaType.h"


#include <fri_okc_comm.h>
#include <fri_okc_types.h>
#include <fri_okc_helper.h>

#include "ComModule/ComOkc.h"
#include "UtilModule/Util.h"
#include "UtilModule/jntlimitfilter.h"
//#include <fstream>


#include <string>

using namespace KDL;
using namespace CBF;

class KukaLwr : public Robot
{
public:
    KukaLwr(RobotNameT connectToRobot, ComOkc& com, ToolNameT tn = sensing_pole);
    void waitForFinished();
    bool isConnected();
    void update_robot_state();
    void get_joint_position_act();
    void get_joint_position_mea();
    void get_joint_position_mea(double *);
    void get_eef_ft(Eigen::Vector3d&,Eigen::Vector3d&);
    void set_joint_command();
    void set_joint_command(RobotModeT rmt);
    void update_cbf_controller();
    double gettimecycle();
    void no_move();
    void setReference (CBF::FloatVector new_ref);
    void setReference (double* positions);
    void setReference (CBF::Float x, CBF::Float y, CBF::Float z, CBF::Float ra, CBF::Float rb, CBF::Float rc);
//    void setAxisStiffnessDamping (lbr_axis_t stiff, lbr_axis_t damp);
    int control_period;
    fri_float_t jnt_position_act[7];
    fri_float_t jnt_position_mea[7];
    fri_float_t jnt_command[7];
    fri_float_t new_cartpos[12];
    void setAxisStiffnessDamping (double* s, double* d);
    void update_robot_stiffness();
    void update_robot_cp_stiffness(Eigen::VectorXd cps,Eigen::VectorXd cpd);
    void update_robot_cp_exttcpft(Eigen::VectorXd ft);
    void switch2cpcontrol();
    void switch2jntcontrol();
    void request_monitor_mode();
    JntLimitFilter *jlf;
    RobotNameT get_robotname(){return rn;}
    Eigen::Matrix3d get_init_TM(){return m_init_tm;}
    void set_init_TM(Eigen::Matrix3d tm) {m_init_tm = tm;}
    Eigen::Vector3d get_cur_vel();
    fri_float_t old_cartpos[12];
    Eigen::Vector3d forceCorr,forceCorrStdDev;
    void calibForce(int sampletimes);
    void getTcpFtCalib (Eigen::Vector3d &cf);
    void addSegmentinChain(Eigen::Matrix3d R,Eigen::Vector3d p);
    void initCbf();
private:
    void initChains(ToolNameT tn=sensing_pole);
    void initReference (CBF::FloatVector& f);
    void initSubordinateReference(CBF::FloatVector& f);
    void initKukaResource();

//    volatile sig_atomic_t pseudo_converge_count;
    bool isFinished();
    bool isPseudoConverged();
    void GetCtrlPeriod(int& );
    RobotNameT rn;
    ComOkc* okc_node;
    CBF::FloatVector updates;
    int robot_id;
//    std::ofstream Jnt_rec;
};

#endif // KUKALWR_H
