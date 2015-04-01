

/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : This is a contact quality test app while kuka finger is following
                kuka palm with/without tactile servo.
 ============================================================================
 */

//for ROS
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#endif

//for ICL
#include <ICLQt/Common.h>
#include <ICLCV/HoughLineDetector.h>
#include <ICLFilter/UnaryCompareOp.h>
#include <ICLCV/RegionDetector.h>
#include <ICLUtils/FPSLimiter.h>
#include <ICLGeom/Scene.h>
#include <ICLGeom/ComplexCoordinateFrameSceneObject.h>
#include <ICLMarkers/FiducialDetector.h>
#include <ICLUtils/Mutex.h>

#include "ComModule/comrsb.h"
#include "ComModule/ComOkc.h"
#include "RobotModule/KukaLwr.h"
#include "RobotModule/Robot.h"
#include "ControllerModule/proactcontroller.h"
#include "TaskModule/kukaselfctrltask.h"
#include "ControllerModule/tacservocontroller.h"
#include "TaskModule/tacservotask.h"
//#include "ControllerModule/CtrlParam.h"
#include <fstream>
#include <sys/stat.h>     //create folder for data record
#include <sys/types.h>

#include "RobotModule/RobotState.h"

#include "UtilModule/Util.h"
#include <utility>
#include "LearningModule/TransformEstimator.h"

Eigen::Matrix3d pose_tm;
Eigen::Vector3d pose_p;

std::ofstream TDataRecord;
std::ofstream VelRecordL;
std::ofstream VelRecordR;
std::ofstream CtcRecord;
std::ofstream EvaCtcRecord;

HSplit gui;
ComOkc *com_okc_left;
ComOkc *com_okc_right;
ComRSB *com_rsb;
Robot *kuka_left_arm;
std::vector<ActController *> left_ac_vec;
std::vector<Task *> left_task_vec;
TaskNameT left_taskname;

Robot *kuka_right_arm;
std::vector<ActController *> right_ac_vec;
std::vector<Task *> right_task_vec;
TaskNameT right_taskname;

RsbDataType rdtlefttac,rdtleftkuka;
myrmex_msg left_myrmex_msg;

RsbDataType rdtrighttac,rdtrightkuka;
myrmex_msg right_myrmex_msg;

ParameterManager *left_pm, *right_pm;
RobotState *left_rs,*right_rs;

kuka_msg left_kuka_msg,right_kuka_msg;
TFEstimator *tfest;

Eigen::Vector3d activel_vel;
std::pair<Eigen::Vector3d,Eigen::Vector3d> activer_vel;
Eigen::Matrix3d TMLearn;
Eigen::Vector3d TLearn;
Eigen::Vector3d p_cad;
Eigen::Matrix3d tm_cad;
bool evalflag;

bool start_rotate, estflag,start_follow;
RobotModeT left_rmt,right_rmt;
Eigen::Vector3d tw_gv;
Eigen::Vector3d tw_go;

#ifdef HAVE_ROS
// ROS objects
tf::TransformBroadcaster *br;
sensor_msgs::JointState js;
ros::Publisher jsPub;
ros::NodeHandle *nh;

#endif

#define CF_THRESHOLD 0.2

#define left_newP_x 0.10
#define left_newP_y 0.3
#define left_newP_z 0.30

#define left_newO_x 0.0
#define left_newO_y M_PI/2;
#define left_newO_z 0.0;

#define right_newP_x 0.20
#define right_newP_y 0.3
#define right_newP_z 0.30

#define right_newO_x 0.0
#define right_newO_y -M_PI/2;
#define right_newO_z 0.0;

struct timeval v_last2;
struct timeval v_old,v_cur;

//using mutex locking controller ptr while it is switching.
Mutex mutex_leftact,mutex_rightact,mutex_tactile,mutex_vel,mutex_ros_vis;


void sliding1_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}

void sliding2_cb(void){
    double cp[2];
    cp[0] = 3;
    cp[1] = 3;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}

void sliding3_cb(void){
    double cp[2];
    cp[0] = 3;
    cp[1] = 12;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}

void sliding4_cb(void){
    double cp[2];
    cp[0] = 12;
    cp[1] = 3;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}

void sliding5_cb(void){
    double cp[2];
    cp[0] = 12;
    cp[1] = 12;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}

void moveto_cb(void){
    right_rmt = NormalMode;
    start_follow = false;
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = left_newP_x;
    p(1) = left_newP_y;
    p(2) = left_newP_z+0.05;

    o(0) = left_newO_x;
    o(1) = left_newO_y;
    o(2) = left_newO_z;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    left_task_vec.back()->mt = JOINTS;
    left_task_vec.back()->mft = GLOBAL;
    left_task_vec.back()->set_desired_p_eigen(p);
    left_task_vec.back()->set_desired_o_ax(o);
    mutex_leftact.unlock();
    std::cout<<"left robot self movement and move to new pose"<<std::endl;

    p(0) = right_newP_x;
    p(1) = right_newP_y;
    p(2) = right_newP_z+0.05;

    o(0) = right_newO_x;
    o(1) = right_newO_y;
    o(2) = right_newO_z;
    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*left_pm));
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = GLOBAL;
    right_task_vec.back()->set_desired_p_eigen(p);
    right_task_vec.back()->set_desired_o_ax(o);
    mutex_rightact.unlock();
    std::cout<<"right robot self movement and move to new pose"<<std::endl;
}

void contact_cb(void){
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = CONTACT_FORCE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}

void back_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RP_LINEFOLLOW;
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();

    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLYP;
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}

void forward_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RP_LINEFOLLOW;
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();

    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLYN;
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}


void up_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RP_LINEFOLLOW;
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();

    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLXP;
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}

void down_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RP_LINEFOLLOW;
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();

    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLXN;
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}


void left_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RP_LINEFOLLOW;
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();

    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLZP;
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}

void right_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RP_LINEFOLLOW;
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();

    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLZN;
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}

void idle_cb(){
    mutex_leftact.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RP_NOCONTROL;
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;
    mutex_leftact.unlock();

    mutex_rightact.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RP_NOCONTROL;
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}

void leftbrake_cb(void){
    com_okc_left->start_brake();
}
void rightbrake_cb(void){
    com_okc_right->start_brake();
}

void leftnobrake_cb(void){
    com_okc_left->release_brake();
}

void rightnobrake_cb(void){
    com_okc_right->release_brake();
}

void updateadmittance_cb(){
    mutex_leftact.lock();
    delete left_pm;
    left_pm = new ParameterManager("left_arm_param.xml");
    left_ac_vec.back()->set_pm(*left_pm);
    left_taskname.tact = CONTACT_FORCE_TRACKING;
    left_ac_vec.back()->updateTacServoCtrlParam(left_taskname.tact);
    mutex_leftact.unlock();
}

void f_rxp_cb(){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_rightact.lock();
    if(start_rotate == false){
        kuka_right_arm->set_init_TM(kuka_right_arm->get_cur_cart_o());
        right_ac_vec.clear();
        right_ac_vec.push_back(new ProActController(*right_pm));
        right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    }
    right_task_vec.clear();
    right_taskname.prot = RRXP;
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;


    mutex_leftact.lock();
    if(start_rotate == false){
        left_ac_vec.clear();
        left_task_vec.clear();
        kuka_left_arm->set_init_TM(kuka_left_arm->get_cur_cart_o());
        left_ac_vec.push_back(new ProActController(*left_pm));
        left_ac_vec.back()->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }

    left_taskname.prot = RP_ROTATEFOLLOW;
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    start_rotate = true;
}
void f_rxn_cb(){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_rightact.lock();
    if(start_rotate == false){
        kuka_right_arm->set_init_TM(right_rs->robot_orien["eef"]);
        right_ac_vec.clear();
        right_ac_vec.push_back(new ProActController(*right_pm));
        right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }
    right_task_vec.clear();
    right_taskname.prot = RRXN;
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();

    std::cout<<"right arm switch to open motion mode"<<std::endl;

    mutex_leftact.lock();
    if(start_rotate == false){
        left_ac_vec.clear();
        kuka_left_arm->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.push_back(new ProActController(*left_pm));
        left_ac_vec.back()->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }
    left_task_vec.clear();
    left_taskname.prot = RP_ROTATEFOLLOW;
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();
    start_rotate = true;
}
void f_ryp_cb(){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_rightact.lock();
    if(start_rotate == false){
        right_ac_vec.clear();
        right_ac_vec.push_back(new ProActController(*right_pm));
        right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }
    kuka_right_arm->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.clear();
    right_taskname.prot = RRYP;
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();

    std::cout<<"right arm switch to open motion mode"<<std::endl;

    mutex_leftact.lock();
    if(start_rotate == false){
        left_ac_vec.clear();
        left_ac_vec.push_back(new ProActController(*left_pm));
        left_ac_vec.back()->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }
    kuka_left_arm->set_init_TM(left_rs->robot_orien["eef"]);
    left_task_vec.clear();
    left_taskname.prot = RP_ROTATEFOLLOW;
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    start_rotate = true;

}
void f_ryn_cb(){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_rightact.lock();
    if(start_rotate == false){
        kuka_right_arm->set_init_TM(right_rs->robot_orien["eef"]);
        right_ac_vec.clear();
        right_ac_vec.push_back(new ProActController(*right_pm));
        right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }
    right_task_vec.clear();
    right_taskname.prot = RRYN;
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();


    mutex_leftact.lock();
    if(start_rotate == false){
        kuka_left_arm->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.clear();
        left_ac_vec.push_back(new ProActController(*left_pm));
        left_ac_vec.back()->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }
    left_task_vec.clear();
    left_taskname.prot = RP_ROTATEFOLLOW;
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);
    mutex_leftact.unlock();
    start_rotate = true;
//    std::cout<<"right arm switch to open motion mode"<<std::endl;

}
void f_rzp_cb(){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_rightact.lock();
    if(start_rotate == false){
        kuka_right_arm->set_init_TM(right_rs->robot_orien["eef"]);
        right_ac_vec.clear();
        right_task_vec.clear();
        right_ac_vec.push_back(new ProActController(*right_pm));
        right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    }
    right_taskname.prot = RRZP;
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();

//    mutex_leftact.lock();
//    if(start_rotate == false){
//        delete left_ac;
//        delete left_task;
//        kuka_left_arm->set_init_TM(left_rs->robot_orien["eef"]);
//        left_ac = new ProActController(*left_pm);
//        left_ac->set_init_TM(left_rs->robot_orien["eef"]);
//    }
//    left_taskname.prot = RP_ROTATEFOLLOW;
//    left_task = new KukaSelfCtrlTask(left_taskname.prot);
//    left_task ->mft = LOCAL;
//    left_task ->mt = JOINTS;
//    mutex_leftact.unlock();

    start_rotate = true;
    std::cout<<"right arm switch to open motion mode"<<std::endl;

}
void f_rzn_cb(){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_rightact.lock();
    if(start_rotate == false){
        right_ac_vec.clear();
        right_task_vec.clear();
        right_ac_vec.push_back(new ProActController(*right_pm));
        right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    }
    kuka_right_arm->set_init_TM(right_rs->robot_orien["eef"]);
    right_taskname.prot = RRZN;
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;
    mutex_rightact.unlock();

//    mutex_leftact.lock();
//    if(start_rotate == false){
//        delete left_ac;
//        delete left_task;
//        kuka_left_arm->set_init_TM(left_rs->robot_orien["eef"]);
//        left_ac = new ProActController(*left_pm);
//        left_ac->set_init_TM(left_rs->robot_orien["eef"]);
//    }
//    left_taskname.prot = RP_ROTATEFOLLOW;
//    left_task = new KukaSelfCtrlTask(left_taskname.prot);
//    left_task ->mft = LOCAL;
//    left_task ->mt = JOINTS;
//    mutex_leftact.unlock();

    start_rotate = true;
    std::cout<<"right arm switch to open motion mode"<<std::endl;

}

void startf_cb(){
    start_follow = true;
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    if(start_rotate == false){
        kuka_left_arm->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.clear();
        left_ac_vec.push_back(new ProActController(*left_pm));
        left_ac_vec.back()->set_init_TM(left_rs->robot_orien["eef"]);
        left_ac_vec.push_back(new TacServoController(*left_pm));
        left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    }
    left_task_vec.clear();
    left_taskname.prot = RP_BOTHFOLLOW;
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = SENSING_POLE_TRACKING;
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cp_myrmex(cp);
    left_task_vec.back()->set_desired_cf_myrmex(CF_THRESHOLD);

    mutex_leftact.unlock();
}


void run_leftarm(){
    Eigen::Vector3d local_l_v;
    Eigen::Vector3d tw_lv,tw_lo;
    tw_lv.setZero();
    tw_lo.setZero();
    std::pair<Eigen::Vector3d,double> local_r_v;
    local_l_v.setZero();
    if((com_okc_left->data_available == true)&&(com_okc_left->controller_update == false)){
        //for left arm
//        kuka_left_arm->update_robot_stiffness();
        //only call for this function, the ->jnt_position_act is updated
        left_rs->updated(kuka_left_arm);
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        //using all kinds of controllers to update the reference
        mutex_leftact.lock();
        for(int i = 0; i < left_ac_vec.size();i++){
            if(left_task_vec[i]->mt == JOINTS)
                mutex_vel.lock();
            if(start_follow==true){
                left_rs->twist_ltog(tw_gv,tw_go,tw_lv,tw_lo);
                left_ac_vec[i]->update_controller_para(tw_lv,tw_lo,left_task_vec[i]->curtaskname.prot);
            }
            mutex_vel.unlock();
            left_ac_vec[i]->update_robot_reference(kuka_left_arm,left_task_vec[i]);
            if(left_task_vec[i]->mt == TACTILE){
                mutex_tactile.lock();
                left_ac_vec[i]->update_robot_reference(kuka_left_arm,left_task_vec[i],&right_myrmex_msg);
                mutex_tactile.unlock();
            }
        }
        //update with act_vec
        left_ac_vec[0]->llv.setZero();
        left_ac_vec[0]->lov.setZero();
        mutex_leftact.unlock();
        //use CBF to compute the desired joint angle rate
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command(left_rmt);
        left_rs->old_hm = left_rs->cur_hm;
        com_okc_left->controller_update = true;
    }

}

void run_rightarm(){
    if((com_okc_right->data_available == true)&&(com_okc_right->controller_update == false)){
        if(gettimeofday(&v_old,NULL)){
            std::cout<<"gettimeofday function error at the current time"<<std::endl;
        }
//        std::cout<<"time difference for two sampling step is ........................................."<<timeval_diff(NULL,&v_old,&v_last2)<<std::endl;
        v_last2 = v_old;
        //for right arm
//        kuka_right_arm->update_robot_stiffness();
        //only call for this function, the ->jnt_position_act is updated
        right_rs->updated(kuka_right_arm);
        mutex_vel.lock();
        right_rs->Est_eef_twist(kuka_right_arm,tw_gv,tw_go);
        //hacking way realize the twist estimation
//        activel_vel = right_rs->EstRobotEefLVel_Ref(kuka_right_arm);
//        VelRecordL<<activel_vel(0)<<","<<activel_vel(1)<<","<<activel_vel(2)<<std::endl;
//        if(start_rotate == true){
//            activer_vel = right_rs->EstRobotEefRVel_InitF(kuka_right_arm,estflag);
//            if(estflag ==true){
//                VelRecordR<<activer_vel.first(0)\
//                       <<","<<activer_vel.first(1)\
//                      <<","<<activer_vel.first(2)\
//                     <<","<<activer_vel.second(0)\
//                    <<","<<activer_vel.second(1)\
//                   << std::endl;
//            }
//        }

        mutex_vel.unlock();
        kuka_right_arm->get_joint_position_act();
        kuka_right_arm->update_robot_state();
        //using all kinds of controllers to update the reference
        mutex_rightact.lock();
        for(int i = 0; i < right_ac_vec.size();i++){
            if(right_task_vec[i]->mt == JOINTS)
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i]);
            if(right_task_vec[i]->mt == TACTILE){
                mutex_tactile.lock();
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i],&right_myrmex_msg);
                mutex_tactile.unlock();
            }
        }
        right_ac_vec[0]->llv.setZero();
        right_ac_vec[0]->lov.setZero();
        mutex_rightact.unlock();
        //use CBF to compute the desired joint angle rate
        kuka_right_arm->update_cbf_controller();
        kuka_right_arm->set_joint_command(right_rmt);
        right_rs->old_hm = right_rs->cur_hm;
        com_okc_right->controller_update = true;
        if(gettimeofday(&v_cur,NULL)){
            std::cout<<"gettimeofday function error at the current time"<<std::endl;
        }
//        std::cout<<"controller can finish it compuation in "<<timeval_diff(NULL,&v_cur,&v_old)<<std::endl;
    }
}

void tactileviarsb(){
    //via network--RSB, the contact information are obtained.
    static FPSLimiter fpsl2(500);
    fpsl2.wait();
    mutex_tactile.lock();
    com_rsb->tactile_receive(left_myrmex_msg,"leftmyrmex");
    com_rsb->tactile_receive(right_myrmex_msg,"rightmyrmex");
//    std::cout<<"left contact information "<<left_myrmex_msg.cf<<","<<left_myrmex_msg.cogx<<","<<left_myrmex_msg.cogy<<","<<left_myrmex_msg.contactflag<<std::endl;
//    std::cout<<"right contact information "<<right_myrmex_msg.cf<<","<<right_myrmex_msg.cogx<<","<<right_myrmex_msg.cogy<<","<<right_myrmex_msg.contactflag<<std::endl;
    TDataRecord<<right_myrmex_msg.cf<<","<<right_myrmex_msg.cogx<<","<<right_myrmex_msg.cogy<<","<<right_myrmex_msg.contactflag<<std::endl;
    left_kuka_msg.p = left_rs->robot_position["eef"];
    left_kuka_msg.o = left_rs->robot_orien["eef"];
    left_kuka_msg.ft.setZero(6);
    com_rsb->kuka_msg_send(left_kuka_msg,"leftkuka");
    right_kuka_msg.p = right_rs->robot_position["eef"];
    right_kuka_msg.o = right_rs->robot_orien["eef"];
    right_kuka_msg.ft.setZero(6);
    com_rsb->kuka_msg_send(right_kuka_msg,"rightkuka");
//    std::cout<<"right kuka msg is sent"<<std::endl;
    mutex_tactile.unlock();
}

void ctp(){
    static FPSLimiter fpsl3(10);
    fpsl3.wait();
    Eigen::Vector3d left_p,right_p;
    left_p.setZero();
    right_p.setZero();
    if(right_myrmex_msg.contactflag == true){
        right_p = right_rs->EstCtcPosition_Ref(kuka_right_arm,right_myrmex_msg);
        left_p = left_rs->robot_position["eef"];
        CtcRecord<<left_p(0)<<","<<left_p(1)<<","<<left_p(2)<<","<<right_p(0)<<","<<right_p(1)<<","<<right_p(2)<<std::endl;
    }
}


void ld_cb(){
    if(right_myrmex_msg.contactflag == true)
        tfest->load(left_rs->EstCtcPosition_KUKAFINGER(),right_rs->EstCtcPosition_KUKAPALM(kuka_right_arm,right_myrmex_msg));
}

void gettf_cb(){
    TMLearn.setZero();
    TLearn.setZero();
    tfest->EstRt(TMLearn,TLearn);
    std::cout<<"computed TM are"<<std::endl;
    std::cout<<TMLearn<<std::endl;
    std::cout<<"computed T are "<<std::endl;
    std::cout<<TLearn<<std::endl;
    Eigen::Vector3d r2l_g;
    tm_cad.setZero();
    p_cad.setZero();
    r2l_g.setZero();
    tm_cad = right_rs->robot_orien["base"].transpose() * left_rs->robot_orien["base"];
    r2l_g = left_rs->robot_position["base"] - right_rs->robot_position["base"];
    p_cad = right_rs->robot_orien["base"].transpose() * r2l_g;
    std::cout<<"CAD TM are"<<std::endl;
    std::cout<<tm_cad<<std::endl;
    std::cout<<"CAD T are "<<std::endl;
    std::cout<<p_cad<<std::endl;
}

void caleva_cb(){
    evalflag = true;
    std::cout<<"start eva"<<std::endl;

}

void clear_cb(){
    tfest->clear();
}

void rightgrav_cb(){
    right_rmt = PsudoGravityCompensation;
}

void evalfun(){
    static FPSLimiter fpsl4(10);
    fpsl4.wait();
    Eigen::Vector3d p_error,p_error2;
    p_error.setZero();
    p_error2.setZero();
    if((evalflag == true)&&(right_myrmex_msg.contactflag == true)){
        p_error = right_rs->EstCtcPosition_KUKAPALM(kuka_right_arm,right_myrmex_msg) - (TLearn + TMLearn*left_rs->EstCtcPosition_KUKAFINGER());
        p_error2 = right_rs->EstCtcPosition_KUKAPALM(kuka_right_arm,right_myrmex_msg) - (p_cad + tm_cad*left_rs->EstCtcPosition_KUKAFINGER());
        EvaCtcRecord<<p_error(0)<<","<<p_error(1)<<","<<p_error(2)<<","<<p_error2(0)<<","<<p_error2(1)<<","<<p_error2(2)<<std::endl;
    }
}



#ifdef HAVE_ROS
void ros_publisher()
{
    static FPSLimiter fpsl4(100);
    fpsl4.wait();


    for(unsigned int i=0 ; i< 7;++i)
    {
        //there is a arm name changed because the confliction between openkc and kukas in rviz
        js.position[i]=right_rs->JntPosition_mea[i];
        js.position[i+7]=left_rs->JntPosition_mea[i];

    }
    js.header.stamp=ros::Time::now();


    //create a ROS tf object and fill it orientation only currently
    //tf::Matrix3x3 tfR;
    //tf::Transform transform;
//    pose_tm = right_rs->robot_orien["eef"];
//    // left kuka
//    tf::matrixEigenToTF (pose_tm, tfR);
//    transform.setOrigin( tf::Vector3(right_rs->robot_position["eef"](0), right_rs->robot_position["eef"](1), right_rs->robot_position["eef"](2)));
//    transform.setBasis(tfR);

//    //broadcast this transform to ROS relative to world
//    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "kukaLeftEndEffector"));

//    // right kuka
//    pose_tm = left_rs->robot_orien["eef"];
//    tf::matrixEigenToTF (pose_tm, tfR);
//    transform.setOrigin( tf::Vector3(right_rs->robot_position["eef"](0), right_rs->robot_position["eef"](1), right_rs->robot_position["eef"](2)));
//    transform.setBasis(tfR);

//    //broadcast this transform to ROS relative to world
//    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "kukaRightEndEffector"));

    // send a joint_state
    jsPub.publish(js);

    ros::spinOnce();

}
#endif

void init(){
    gui << (VBox()
        << Button("Contact").handle("contact_task")
        << Button("Back").handle("back_task")
        << Button("Forward").handle("forward_task")
        << Button("Up").handle("up_task")
        << Button("Down").handle("down_task")
        << Button("Left").handle("left_task")
        << Button("Right").handle("right_task")
        << Button("f_rxp").handle("f_rxp_task")
        << Button("f_rxn").handle("f_rxn_task")
        << Button("f_ryp").handle("f_ryp_task")
        << Button("f_ryn").handle("f_ryn_task")
        << Button("f_rzp").handle("f_rzp_task")
        << Button("f_rzn").handle("f_rzn_task")
        << Button("Idle").handle("idle_task")
        << Button("LoadData").handle("ld_task")
        << Button("GetTF").handle("gettf_task")
        << Button("CalEva").handle("caleva_task")
        << Button("ClearData").handle("clear_task")
        )
    << (VBox()
            << Button("S1").handle("sliding1_task")
            << Button("S2").handle("sliding2_task")
            << Button("S3").handle("sliding3_task")
            << Button("S4").handle("sliding4_task")
            << Button("S5").handle("sliding5_task")
        )
    << (VBox()
            << Button("Moveto").handle("moveto_task")
            << Button("right_grav").handle("rightgrav_task")
            << Button("start_follow").handle("startf_task")
            << Button("LeftBrake").handle("leftbrake_task")
            << Button("RightBrake").handle("rightbrake_task")
            << Button("LeftNoBrake").handle("leftnobrake_task")
            << Button("RightNoBrake").handle("rightnobrake_task")
            << Button("UAdmittanceParam").handle("uadmittanceparam_task")
            << Button("right_grav").handle("rightgrav_task")
        )
    << Show();
    gui["contact_task"].registerCallback(utils::function(contact_cb));
    gui["moveto_task"].registerCallback(utils::function(moveto_cb));
    gui["leftbrake_task"].registerCallback(utils::function(leftbrake_cb));
    gui["rightbrake_task"].registerCallback(utils::function(rightbrake_cb));
    gui["leftnobrake_task"].registerCallback(utils::function(leftnobrake_cb));
    gui["rightnobrake_task"].registerCallback(utils::function(rightnobrake_cb));
    gui["back_task"].registerCallback(utils::function(back_cb));
    gui["forward_task"].registerCallback(utils::function(forward_cb));
    gui["up_task"].registerCallback(utils::function(up_cb));
    gui["down_task"].registerCallback(utils::function(down_cb));
    gui["left_task"].registerCallback(utils::function(left_cb));
    gui["right_task"].registerCallback(utils::function(right_cb));
    gui["f_rxp_task"].registerCallback(utils::function(f_rxp_cb));
    gui["f_rxn_task"].registerCallback(utils::function(f_rxn_cb));
    gui["f_ryp_task"].registerCallback(utils::function(f_ryp_cb));
    gui["f_ryn_task"].registerCallback(utils::function(f_ryn_cb));
    gui["f_rzp_task"].registerCallback(utils::function(f_rzp_cb));
    gui["f_rzn_task"].registerCallback(utils::function(f_rzn_cb));
    gui["idle_task"].registerCallback(utils::function(idle_cb));
    gui["sliding1_task"].registerCallback(utils::function(sliding1_cb));
    gui["sliding2_task"].registerCallback(utils::function(sliding2_cb));
    gui["sliding3_task"].registerCallback(utils::function(sliding3_cb));
    gui["sliding4_task"].registerCallback(utils::function(sliding4_cb));
    gui["sliding5_task"].registerCallback(utils::function(sliding5_cb));
    gui["ld_task"].registerCallback(utils::function(ld_cb));
    gui["gettf_task"].registerCallback(utils::function(gettf_cb));
    gui["caleva_task"].registerCallback(utils::function(caleva_cb));
    gui["clear_task"].registerCallback(utils::function(clear_cb));
    gui["uadmittanceparam_task"].registerCallback(utils::function(updateadmittance_cb));
    gui["rightgrav_task"].registerCallback(utils::function(rightgrav_cb));
    gui["startf_task"].registerCallback(utils::function(startf_cb));

    //    std::cout<<"before start rsb........................................................................"<<std::endl;
    com_rsb = new ComRSB();
    rdtlefttac = LeftMyrmex;
    rdtleftkuka = LeftKukaEff;
    rdtrighttac = RightMyrmex;
    rdtrightkuka = RightKukaEff;
    com_rsb->add_msg(rdtlefttac);
    com_rsb->add_msg(rdtleftkuka);
    com_rsb->add_msg(rdtrighttac);
    com_rsb->add_msg(rdtrightkuka);
    //    std::cout<<" after start rsb..............................................................................."<<std::endl;

    left_pm = new ParameterManager("left_arm_param.xml");
    right_pm = new ParameterManager("right_arm_param.xml");
    com_okc_left = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc_right = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);
    com_okc_left->connect();
    com_okc_right->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc_left);
    kuka_right_arm = new KukaLwr(kuka_right,*com_okc_right);

    left_rs = new RobotState(kuka_left_arm);
    right_rs = new RobotState(kuka_right_arm);

    Eigen::Vector3d p_left,o_left,p_right,o_right;
    p_left.setZero();
    o_left.setZero();
    p_right.setZero();
    o_right.setZero();

    left_ac_vec.push_back(new ProActController(*left_pm));
    left_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    p_left(0) = left_newP_x;
    p_left(1) = left_newP_y;
    p_left(2) = left_newP_z;
    o_left(0) = left_newO_x;
    o_left(1) = left_newO_y;
    o_left(2) = left_newO_z;
    left_task_vec.back()->set_desired_p_eigen(p_left);
    left_task_vec.back()->set_desired_o_ax(o_left);
    kuka_left_arm->setAxisStiffnessDamping(left_ac_vec.back()->pm.stiff_ctrlpara.axis_stiffness, \
                                           left_ac_vec.back()->pm.stiff_ctrlpara.axis_damping);
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));

    p_right(0) = right_newP_x;
    p_right(1) = right_newP_y;
    p_right(2) = right_newP_z;
    o_right(0) = right_newO_x;
    o_right(1) = right_newO_y;
    o_right(2) = right_newO_z;
    right_task_vec.back()->set_desired_p_eigen(p_right);
    right_task_vec.back()->set_desired_o_ax(o_right);
    kuka_right_arm->setAxisStiffnessDamping(right_ac_vec.back()->pm.stiff_ctrlpara.axis_stiffness, \
                                           right_ac_vec.back()->pm.stiff_ctrlpara.axis_damping);
    std::cout<<"finish init"<<std::endl;
    activel_vel.setZero();
    activer_vel.first.setZero();
    activer_vel.second.setZero();
    start_rotate = false;
    estflag = false;
    tfest = new TFEstimator();
    evalflag = false;
    pose_tm.setZero();
    pose_p.setZero();
    left_rmt = NormalMode;
    right_rmt = NormalMode;
    tw_gv.setZero();
    tw_go.setZero();
    start_follow = false;
#ifdef HAVE_ROS
    std::string left_kuka_arm_name="la";
    std::string right_kuka_arm_name="ra";
    js.name.push_back(left_kuka_arm_name+"_arm_0_joint");
    js.name.push_back(left_kuka_arm_name+"_arm_1_joint");
    js.name.push_back(left_kuka_arm_name+"_arm_2_joint");
    js.name.push_back(left_kuka_arm_name+"_arm_3_joint");
    js.name.push_back(left_kuka_arm_name+"_arm_4_joint");
    js.name.push_back(left_kuka_arm_name+"_arm_5_joint");
    js.name.push_back(left_kuka_arm_name+"_arm_6_joint");
    js.name.push_back(right_kuka_arm_name+"_arm_0_joint");
    js.name.push_back(right_kuka_arm_name+"_arm_1_joint");
    js.name.push_back(right_kuka_arm_name+"_arm_2_joint");
    js.name.push_back(right_kuka_arm_name+"_arm_3_joint");
    js.name.push_back(right_kuka_arm_name+"_arm_4_joint");
    js.name.push_back(right_kuka_arm_name+"_arm_5_joint");
    js.name.push_back(right_kuka_arm_name+"_arm_6_joint");

    js.position.resize(14);
    js.velocity.resize(14);
    js.effort.resize(14);

    js.header.frame_id="kuka_frame";

    jsPub = nh->advertise<sensor_msgs::JointState> ("joint_states", 2);
    ros::spinOnce();

    br = new tf::TransformBroadcaster();

    std::cout<<"ros init finished"<<std::endl;
#endif

}

int main(int argc, char* argv[])
{
#ifdef HAVE_ROS
    ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle();
#endif
    if(mkdir("/tmp/data",0777)==-1)//creating a directory
    {
        std::cout<<"creat folder failed"<<std::endl;
// 		exit(1);
        }
        int temp;
         if ((temp = mkdir("/tmp/data",0777)) != 0) {
        fprintf(stderr, "ERROR %d: unable to mkdir; %s\n", errno, strerror(errno));
    }

    std::string data_f ("/tmp/data/");
    TDataRecord.open((data_f+std::string("TDR.txt")).c_str());
    VelRecordL.open((data_f+std::string("VELL.txt")).c_str());
    VelRecordR.open((data_f+std::string("VELR.txt")).c_str());
    CtcRecord.open((data_f+std::string("CTC.txt")).c_str());
    EvaCtcRecord.open((data_f+std::string("EVACTC.txt")).c_str());
#ifdef HAVE_ROS
    int ret = ICLApp(argc,argv,"-input|-i(2)",init,run_leftarm,run_rightarm,tactileviarsb,ctp,ros_publisher).exec();
    //int ret = ICLApp(argc,argv,"-input|-i(2)",init,run_leftarm,run_rightarm,tactileviarsb,ctp).exec();
    ros::shutdown();
    delete br;
    delete nh;
#else
    int ret = ICLApp(argc,argv,"-input|-i(2)",init,run_leftarm,run_rightarm,tactileviarsb,ctp,evalfun).exec();
#endif
    return ret;
}



