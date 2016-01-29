

/*
 ============================================================================
 Name        : righttactoolservo.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : kuka+shadow hand grasp tool do tactile servo, one end of the tool is covered by the myrmex sensor
 ============================================================================
 */

//for ROS
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
//#include <agni_utils/tactile_calibration.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#endif
#include <iomanip>


#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
#include <iostream>

#include "Timer.h"
#include "comrsb.h"
#include "ComOkc.h"
#include "KukaLwr.h"
#include "Robot.h"
#include "proactcontroller.h"
#include "kukaselfctrltask.h"
#include "forceservocontroller.h"
#include "forceservotask.h"
#include "tacservocontroller.h"
#include "tacservotask.h"
#include "RobotState.h"
#include "Util.h"
#include <fstream>
#include <mutex>
#include "gamaft.h"
#include "RebaType.h"
#include "msgcontenttype.h"
#include "pcafeature.h"
#include "regression2d.h"
#include "maniptool.h"
#include "contactdetector.h"

#include <deque> //for estimating the normal direction of tool-end

#include <math.h>       /* acos */

//desired contact pressure
#define TAC_F 0.1
#define NV_EST_LEN 200

#ifdef HAVE_ROS
// ROS objects
tf::TransformBroadcaster *br;
sensor_msgs::JointState js;
ros::Publisher jsPub;
ros::NodeHandle *nh;
//ros::Publisher gamma_force_marker_pub;
ros::Publisher nv_est_marker_pub;
#endif

ComOkc *com_okc;
Robot *kuka_right_arm;
std::vector<ActController *> right_ac_vec;
std::vector<Task *> right_task_vec;
TaskNameT right_taskname;

ComRSB *com_rsb;
RsbDataType rdtleftkuka;
RsbDataType rdtlefttac;
RsbDataType rdtfiducial;
myrmex_msg left_myrmex_msg;
markered_object_msg tactoolmarker;
ParameterManager* pm;
RobotState *right_rs;
PCAFeature *pcaf;
//for estimate the rotation of the real tactool xy
Regression2d *rg2d;
Eigen::Vector2d tacp2d;
Reg_param rgp;

//tactool initialization
ManipTool *mt_ptr;
RG_Pose init_tool_pose;

//contact detector init
ContactDetector *cdt;

#define newP_x 0.1
#define newP_y 0.4
#define newP_z 0.30

#define newO_x 0.0
#define newO_y -M_PI/2;
#define newO_z 0.0;


double initP_x,initP_y,initP_z;
double initO_x,initO_y,initO_z;


//using mutex locking controller ptr while it is switching.
std::mutex mutex_act, mutex_force,mutex_tac,mutex_ft,mutex_vis;
//estimated force/torque from fri
Eigen::Vector3d estkukaforce,estkukamoment;
Eigen::Vector3d filtered_force;
Eigen::VectorXd ft;
TemporalSmoothingFilter<Eigen::Vector3d>* cf_filter;
uint32_t marker_shape;
ToolNameT tn;

bool StopFlag;

//define the robot control mode: normal mode vs psudo_gravity_compensation mode
RobotModeT rmt;

//save the last 500 position of robot end-effector
std::deque<Eigen::Vector3d> robot_eef_deque;
//save tactile trajectory in order to estimate y axis
std::vector<Eigen::Vector2d> tac_tra_vec;
bool rec_flag_nv_est;
bool rec_flag_xy_est;
bool vis_est_ort;
Eigen::Vector3d est_tool_nv;
Eigen::Matrix3d init_est_tool_ort;
Eigen::Matrix3d cur_est_tool_ort;
Eigen::Matrix3d cur_update_tool_ort;
Eigen::Matrix3d rel_eef_tactool;

//for update the tactool contact frame xy-tangent surface to the real tac sensor frame
Eigen::Matrix3d real_tactool_ctcframe,rotationmatrix;

struct ExploreAction{
    double ra_xaxis;
    double ra_yaxis;
    double la_xaxis;
    double la_yaxis;
    ExploreAction(){
        ra_xaxis = 0;
        ra_yaxis = 0;
        la_xaxis = 0;
        la_yaxis = 0;
    }
};
//exploration action for tactool learning
ExploreAction ea;

void tactileviarsb(){
    //via network--RSB, the contact information are obtained.
    mutex_tac.lock();
    com_rsb->tactile_receive(left_myrmex_msg,"leftmyrmex");
//    std::cout<<"tactile output"<<std::endl;
//    std::cout<<"myrmex readout "<<left_myrmex_msg.cogx<<","<<left_myrmex_msg.cogy<<std::endl;

    mutex_tac.unlock();
}

void vismarkerviarsb(){
    mutex_vis.lock();
    com_rsb->fiducialmarker_receive(tactoolmarker);
//    std::cout<<"fiducial marker p"<<tactoolmarker.p<<std::endl;
//    std::cout<<"fiducial marker o"<<tactoolmarker.orientation<<std::endl;
    mutex_vis.unlock();
}

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}

void tactool_force_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.forcet = F_MAINTAIN;
    right_ac_vec.push_back(new ForceServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new ForceServoTask(right_taskname.forcet));
    right_task_vec.back()->mt = FORCE;
    right_task_vec.back()->set_desired_cf_kuka(3);
    mutex_act.unlock();
    std::cout<<"maintain the force...................."<<std::endl;
}

void tactool_tactile_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.tact = LEARN_TACTOOL_CONTACT;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for maintain contact"<<std::endl;
}

void tactool_taxel_sliding_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    double cp[2];
    cp[0] = 7.5;
    cp[1] = 7.5;
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.tact = CONTACT_POINT_FORCE_TRACKING;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->emt = NOEXPLORE;
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
    right_task_vec.back()->set_desired_cp_myrmex(cp);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}


void tactool_exploring_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLXN;
    right_ac_vec.push_back(new ProActController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;

    right_taskname.tact = COVER_OBJECT_SURFACE;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_mid(TAC_F);

    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}

void tactool_cablefollow_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.prot = RLZN;
    right_ac_vec.push_back(new ProActController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new KukaSelfCtrlTask(right_taskname.prot));
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->mt = JOINTS;

    right_taskname.tact = LINEAR_TRACKING;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;

    right_task_vec.back()->set_desired_cf_mid(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}

void tactool_taxel_rolling_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.tact = LEARN_TACTOOL_ROLLING;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
    right_task_vec.back()->set_desired_rotation_range(ea.ra_xaxis,ea.ra_yaxis,0);
    mutex_act.unlock();
    std::cout<<"tactile servoing for rolling to the desired point"<<std::endl;
}


void tactool_grav_comp_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to psudo_gravity_compasenstation control"<<std::endl;
    rmt = PsudoGravityCompensation;
    rec_flag_nv_est = true;
}

void tactool_normal_ctrl_cb(boost::shared_ptr<std::string> data){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) = initP_x = right_rs->robot_position["eef"](0);
    p(1) = initP_y= right_rs->robot_position["eef"](1);
    p(2) = initP_z= right_rs->robot_position["eef"](2);

    o = tm2axisangle(right_rs->robot_orien["eef"]);
    initO_x = o(0);
    initO_y = o(1);
    initO_z = o(2);
    right_task_vec.back()->set_desired_p_eigen(p);
    right_task_vec.back()->set_desired_o_ax(o);
    std::cout<<"switch to normal control"<<std::endl;
    std::cout<<"switch to normal control"<<std::endl;
    rmt = NormalMode;
}

void nv_est(){
    if(rec_flag_nv_est == true){
        rec_flag_nv_est = false;
        pcaf->GetData(robot_eef_deque);
        est_tool_nv = pcaf->getSlope_batch();
        est_tool_nv.normalize();
        init_est_tool_ort = gen_ort_basis(est_tool_nv);
        init_tool_pose.o = init_est_tool_ort;
        //Ttool2eef = Tg2eef * Ttool2g; to this end, the Ttool can be updated by Ttool2g = Teef2g * Ttool2eef;
        //which is Ttool = Teef * rel_eef_tactool;
        rel_eef_tactool = right_rs->robot_orien["robot_eef"].transpose() * init_est_tool_ort;
        if(left_myrmex_msg.contactflag == true){
            mt_ptr->ts.init_ctc_x = left_myrmex_msg.cogx;
            mt_ptr->ts.init_ctc_y = left_myrmex_msg.cogy;
        }
        std::cout<<"init tactile ctc "<<mt_ptr->ts.init_ctc_x<<","<<mt_ptr->ts.init_ctc_y<<std::endl;
        mt_ptr->ts.eef_pose.push_back(init_tool_pose);
        mt_ptr->ts.rel_o = rel_eef_tactool;
        vis_est_ort = true;
        mutex_act.lock();
        right_ac_vec.clear();
        right_task_vec.clear();
        right_ac_vec.push_back(new ProActController(*pm));
        right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
        right_task_vec.back()->mt = JOINTS;
        right_task_vec.back()->mft = GLOBAL;

        //robot is kept in the current pose
        Eigen::Vector3d p,o;
        p.setZero();
        o.setZero();

        //get start point position in cartesian space
        p(0) = initP_x = right_rs->robot_position["eef"](0);
        p(1) = initP_y= right_rs->robot_position["eef"](1);
        p(2) = initP_z= right_rs->robot_position["eef"](2);

        o = tm2axisangle(right_rs->robot_orien["eef"]);
        initO_x = o(0);
        initO_y = o(1);
        initO_z = o(2);
        right_task_vec.back()->set_desired_p_eigen(p);
        right_task_vec.back()->set_desired_o_ax(o);
        std::cout<<"switch to normal control"<<std::endl;
        rmt = NormalMode;
        mutex_act.unlock();
    }
}


void nv_est_cb(boost::shared_ptr<std::string> data){
    rec_flag_nv_est = false;
    pcaf->GetData(robot_eef_deque);
    est_tool_nv = pcaf->getSlope_batch();
    est_tool_nv.normalize();
    init_est_tool_ort = gen_ort_basis(est_tool_nv);
    init_tool_pose.o = init_est_tool_ort;
    //Ttool2eef = Tg2eef * Ttool2g; to this end, the Ttool can be updated by Ttool2g = Teef2g * Ttool2eef;
    //which is Ttool = Teef * rel_eef_tactool;
    rel_eef_tactool = right_rs->robot_orien["robot_eef"].transpose() * init_est_tool_ort;
    if(left_myrmex_msg.contactflag == true){
        mt_ptr->ts.init_ctc_x = left_myrmex_msg.cogx;
        mt_ptr->ts.init_ctc_y = left_myrmex_msg.cogy;
    }
    mt_ptr->ts.eef_pose.push_back(init_tool_pose);
    mt_ptr->ts.rel_o = rel_eef_tactool;
    rec_flag_nv_est = true;
    vis_est_ort = true;
    //robot is kept in the current pose
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) = initP_x = right_rs->robot_position["eef"](0);
    p(1) = initP_y= right_rs->robot_position["eef"](1);
    p(2) = initP_z= right_rs->robot_position["eef"](2);

    o = tm2axisangle(right_rs->robot_orien["eef"]);
    initO_x = o(0);
    initO_y = o(1);
    initO_z = o(2);
    right_task_vec.back()->set_desired_p_eigen(p);
    right_task_vec.back()->set_desired_o_ax(o);
    std::cout<<"switch to normal control"<<std::endl;
    rmt = NormalMode;
}

void xy_est_cb(){
    rec_flag_xy_est = false;
    rgp = rg2d->get_kb_batch(tac_tra_vec);
    //rgp.sign_k is using atan2 to esimate the quadrant in which the contact points trajectory is located
    //assume that tactool is take a linear exploration moving along +x axis
    double DeltaGama;
    if(rgp.sign_k == 1)
        DeltaGama = atan(rgp.k) - M_PI_2;
    else
        DeltaGama = atan(rgp.k) + M_PI_2;
    //rotation matrix from sdot to s (sdot is virtual arbitary sensor frame defined by the normal direction
    //s is the estimated real tactile sensor frame)
    rotationmatrix(0,0) = cos(DeltaGama);
    rotationmatrix(0,1) = (-1)*sin(DeltaGama);
    rotationmatrix(1,0) = sin(DeltaGama);
    rotationmatrix(1,1) = cos(DeltaGama);

    mt_ptr->ts.rotate_s2sdot = rotationmatrix;
}

void update_contact_frame_cb(boost::shared_ptr<std::string> data){
    xy_est_cb();
}


void col_est_nv(){
    if((rec_flag_nv_est == true)&&(left_myrmex_msg.contactflag!=true)){
        robot_eef_deque.push_back(right_rs->robot_position["robot_eef"]);
        robot_eef_deque.pop_front();
    }
}

//collect all data for estimating the real tactile sensor x y axis
void col_est_xy(){
    if(rec_flag_xy_est == true){
        tacp2d.setZero();
        tacp2d(0) = left_myrmex_msg.cogx;
        tacp2d(1) = left_myrmex_msg.cogy;
        tac_tra_vec.push_back(tacp2d);
    }
}


void moveto_cb(boost::shared_ptr<std::string> data){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) =  0.3;
    p(1) =  right_rs->robot_position["eef"](1);
    p(2) = right_rs->robot_position["eef"](2);;

    o(0) = 0.0;
    o(1) = -M_PI/2;
    o(2) = 0.0;
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*pm));
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = GLOBAL;
    right_task_vec.back()->set_desired_p_eigen(p);
    right_task_vec.back()->set_desired_o_ax(o);
    mutex_act.unlock();
    std::cout<<"robot self movement and move to new pose"<<std::endl;
}

void ftcalib_cb(boost::shared_ptr<std::string> data){
    kuka_right_arm->calibForce(1000);
}


void brake_cb(boost::shared_ptr<std::string> data){
    com_okc->start_brake();
}

void nobrake_cb(boost::shared_ptr<std::string> data){
    com_okc->release_brake();
}


void rotatexangle_cb(boost::shared_ptr<std::string> data){
   ea.ra_xaxis = 0.01 * std::stoi(*data);
   std::cout<<"rot along x "<<ea.ra_xaxis<<std::endl;
   mutex_act.lock();
   right_ac_vec.clear();
   right_task_vec.clear();
   right_taskname.tact = LEARN_TACTOOL_ROLLING;
   right_ac_vec.push_back(new TacServoController(*pm));
   right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
   right_task_vec.push_back(new TacServoTask(right_taskname.tact));
   right_task_vec.back()->emt = ROTATEEXPLORE;
   right_task_vec.back()->mt = TACTILE;
   right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
   right_task_vec.back()->set_desired_rotation_range(ea.ra_xaxis,ea.ra_yaxis,0);
   mutex_act.unlock();
   std::cout<<"tactile servoing for rolling to the desired point"<<std::endl;
}

void rotateyangle_cb(boost::shared_ptr<std::string> data){
   ea.ra_yaxis = 0.01 * std::stoi(*data);
   std::cout<<"rot along y "<<ea.ra_yaxis<<std::endl;
   mutex_act.lock();
   right_ac_vec.clear();
   right_task_vec.clear();
   right_taskname.tact = LEARN_TACTOOL_ROLLING;
   right_ac_vec.push_back(new TacServoController(*pm));
   right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
   right_task_vec.push_back(new TacServoTask(right_taskname.tact));
   right_task_vec.back()->emt = ROTATEEXPLORE;
   right_task_vec.back()->mt = TACTILE;
   right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
   right_task_vec.back()->set_desired_rotation_range(ea.ra_xaxis,ea.ra_yaxis,0);
   mutex_act.unlock();
   std::cout<<"tactile servoing for rolling to the desired point"<<std::endl;
}

void linexlen_cb(boost::shared_ptr<std::string> data){
   ea.la_xaxis = 0.5 * std::stoi(*data);
   std::cout<<"vel along x "<<ea.la_xaxis<<std::endl;
   mutex_act.lock();
   right_ac_vec.clear();
   right_task_vec.clear();
   right_taskname.tact = LEARN_TACTOOL_SLIDING;
   right_ac_vec.push_back(new TacServoController(*pm));
   right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
   right_task_vec.push_back(new TacServoTask(right_taskname.tact));
   right_task_vec.back()->emt = LINEAREXPLORE;
   right_task_vec.back()->mt = TACTILE;
   right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
   right_task_vec.back()->set_desired_cp_moving_dir(ea.la_xaxis,ea.la_yaxis);
   mutex_act.unlock();
   rec_flag_xy_est = true;
   std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}

void lineylen_cb(boost::shared_ptr<std::string> data){
   ea.la_yaxis = 0.5 * std::stoi(*data);
   std::cout<<"vel along y "<<ea.la_yaxis<<std::endl;
   mutex_act.lock();
   right_ac_vec.clear();
   right_task_vec.clear();
   right_taskname.tact = LEARN_TACTOOL_SLIDING;
   right_ac_vec.push_back(new TacServoController(*pm));
   right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
   right_task_vec.push_back(new TacServoTask(right_taskname.tact));
   right_task_vec.back()->emt = LINEAREXPLORE;
   right_task_vec.back()->mt = TACTILE;
   right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
   right_task_vec.back()->set_desired_cp_moving_dir(ea.la_xaxis,ea.la_yaxis);
   mutex_act.unlock();
   std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
   rec_flag_xy_est = true;
}

#ifdef HAVE_ROS
void run(){
    ros::Rate r(500);
    while (ros::ok()){
      r.sleep();
      ros::spinOnce();
    }
}

void ros_publisher(){
    //prepare joint state data
    for(unsigned int i=0 ; i< 7;++i){
        //there is a arm name changed because the confliction between openkc and kukas in rviz
        js.position[i]=right_rs->JntPosition_mea[i];
        js.position[i+7]=0;
    }

    js.header.stamp=ros::Time::now();
    //publish the gamma force vector
    if(nv_est_marker_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker nv_est_marker;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        nv_est_marker.color.r = 1.0f;
        nv_est_marker.color.g = 0.0f;
        nv_est_marker.color.b = 0.0f;
        nv_est_marker.color.a = 1.0;
        mutex_tac.unlock();

        nv_est_marker.header.frame_id = "frame";
        nv_est_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        nv_est_marker.ns = "KukaRos";
        nv_est_marker.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        nv_est_marker.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        nv_est_marker.action = visualization_msgs::Marker::ADD;

        nv_est_marker.points.resize(2);
        nv_est_marker.points[0].x = right_rs->robot_position["eef"](0);
        nv_est_marker.points[0].y = right_rs->robot_position["eef"](1);
        nv_est_marker.points[0].z = right_rs->robot_position["eef"](2);

        nv_est_marker.points[1].x = right_rs->robot_position["eef"](0)+est_tool_nv(0)/100;
        nv_est_marker.points[1].y = right_rs->robot_position["eef"](1)+est_tool_nv(1)/100;
        nv_est_marker.points[1].z = right_rs->robot_position["eef"](2)+est_tool_nv(2)/100;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        nv_est_marker.scale.x = .001;
        nv_est_marker.scale.y = .001;
        nv_est_marker.scale.z = .001;

        nv_est_marker.lifetime = ros::Duration();
        nv_est_marker_pub.publish(nv_est_marker);
    }

    //create a ROS tf object and fill it orientation only currently
    //broadcast this transform to ROS relative to world(kuka_endeffector)
    if(vis_est_ort == true){
        tf::Matrix3x3 tfR;
        tf::Transform transform;
        cur_est_tool_ort = right_rs->robot_orien["robot_eef"] * rel_eef_tactool;
        tf::matrixEigenToTF (cur_est_tool_ort, tfR);
        transform.setOrigin( tf::Vector3(right_rs->robot_position["eef"](0), right_rs->robot_position["eef"](1), right_rs->robot_position["eef"](2)) );
        transform.setBasis(tfR);
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "est_tactool_frame"));

        //update contact frame after the tac exploration action
        real_tactool_ctcframe =  cur_est_tool_ort * rotationmatrix;
        tf::matrixEigenToTF (real_tactool_ctcframe, tfR);
        transform.setOrigin( tf::Vector3(right_rs->robot_position["eef"](0), right_rs->robot_position["eef"](1), right_rs->robot_position["eef"](2)) );
        transform.setBasis(tfR);
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "updated_tactool_frame"));
    }




    // send a joint_state
    jsPub.publish(js);
//    ros::spinOnce();
}

#endif

std::string get_selfpath() {
    char buff[PATH_MAX];
    ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
    if (len != -1) {
      buff[len] = '\0';

      std::string path = std::string(buff);
      //remove exec name
      std::size_t found = path.find_last_of("/");
      path = path.substr(0,found);
      //remove bin
      found = path.find_last_of("/");
      path = path.substr(0,found);
      //remove build
      found = path.find_last_of("/");
      path = path.substr(0,found);
      return path;
    }
    else{
        std::cout<<"get exacutable file path failaure"<<std::endl;
        exit(0);
    }

    /* handle error condition */
}

void init(){
    cdt = new ContactDetector();
    std::string selfpath = get_selfpath();
    robot_eef_deque.assign(NV_EST_LEN, Eigen::Vector3d::Zero());
    pcaf = new PCAFeature(NV_EST_LEN);
    est_tool_nv.setZero();
    init_est_tool_ort.setIdentity();
    cur_est_tool_ort.setIdentity();
    cur_update_tool_ort.setIdentity();
    rel_eef_tactool.setIdentity();
    rec_flag_nv_est = false;
    rec_flag_xy_est = false;
    vis_est_ort = false;
    rmt = NormalMode;
    tn = tactool;
    StopFlag = false;
    //initialize hand-hold manipulation tool--here is a tactile brush
    mt_ptr = new ManipTool();
    rg2d = new Regression2d();
    real_tactool_ctcframe.setIdentity();
    rotationmatrix.setIdentity();

    mt_ptr ->mtt = Tacbrush;
    mt_ptr->ts.dof_num = 0;
//    init_tool_pose.p.setZero();
//    init_tool_pose.o.setZero();
//    init_tool_pose.rel_o.setZero();
    //declare the cb function
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_force(tactool_force_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_tactile(tactool_tactile_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_taxel_sliding(tactool_taxel_sliding_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_taxel_rolling(tactool_taxel_rolling_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_exploring(tactool_exploring_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_moveto(moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_nv_est(nv_est_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_ftcalib(ftcalib_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_follow(tactool_cablefollow_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_grav_comp_ctrl(tactool_grav_comp_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_normal_ctrl(tactool_normal_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_update_contact_frame(update_contact_frame_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_brake(brake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_nobrake(nobrake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);
    boost::function<void(boost::shared_ptr<std::string>)> slider_linexlen(linexlen_cb);
    boost::function<void(boost::shared_ptr<std::string>)> slider_lineylen(lineylen_cb);
    boost::function<void(boost::shared_ptr<std::string>)> slider_rotatexangle(rotatexangle_cb);
    boost::function<void(boost::shared_ptr<std::string>)> slider_rotateyangle(rotateyangle_cb);

    std::string config_filename = selfpath + "/etc/right_arm_param.xml";
    if(is_file_exist(config_filename.c_str()) == false){
        config_filename = "right_arm_param.xml";
        if(is_file_exist(config_filename.c_str()) == false){
            std::cout<<"not find the tactile servo controller configure file"<<std::endl;
            exit(0);
        }
    }

    pm = new ParameterManager(config_filename,Myrmex);
    com_okc = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_right_arm = new KukaLwr(kuka_right,*com_okc,tn);
    right_rs = new RobotState(kuka_right_arm);
    kuka_right_arm->get_joint_position_act();
    kuka_right_arm->update_robot_state();
    right_rs->updated(kuka_right_arm);
//    std::cout<<"state in the initialized stage are"<<right_rs->robot_position["eef"]<<std::endl;
    right_ac_vec.push_back(new ProActController(*pm));
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) = initP_x = right_rs->robot_position["eef"](0);
    p(1) = initP_y= right_rs->robot_position["eef"](1);
    p(2) = initP_z= right_rs->robot_position["eef"](2);

    o = tm2axisangle(right_rs->robot_orien["eef"]);
    initO_x = o(0);
    initO_y = o(1);
    initO_z = o(2);
    right_task_vec.back()->set_desired_p_eigen(p);
    right_task_vec.back()->set_desired_o_ax(o);
    kuka_right_arm->setAxisStiffnessDamping(right_ac_vec.back()->pm.stiff_ctrlpara.axis_stiffness, \
                                           right_ac_vec.back()->pm.stiff_ctrlpara.axis_damping);
    com_rsb = new ComRSB();
    rdtleftkuka = LeftKukaEff;
    rdtlefttac = LeftMyrmex;
    rdtfiducial = FiducialMarkerFeature;
    com_rsb->add_msg(rdtleftkuka);
    com_rsb->add_msg(rdtlefttac);
    com_rsb->add_msg(rdtfiducial);
    ft.setZero(6);
    estkukaforce.setZero();
    estkukamoment.setZero();
    cf_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(5,Average,Eigen::Vector3d(0,0,0));
    //register cb function
    com_rsb->register_external("/foo/moveto",button_moveto);
    com_rsb->register_external("/foo/nv_est",button_nv_est);
    com_rsb->register_external("/foo/tactool_force",button_tactool_force);
    com_rsb->register_external("/foo/tactool_tactile",button_tactool_tactile);
    com_rsb->register_external("/foo/tactool_taxel_sliding",button_tactool_taxel_sliding);
    com_rsb->register_external("/foo/tactool_taxel_rolling",button_tactool_taxel_rolling);
    com_rsb->register_external("/foo/tactool_exploring",button_tactool_exploring);
    com_rsb->register_external("/foo/ftcalib",button_ftcalib);
    com_rsb->register_external("/foo/tactool_follow",button_tactool_follow);
    com_rsb->register_external("/foo/tactool_grav_comp_ctrl",button_tactool_grav_comp_ctrl);
    com_rsb->register_external("/foo/tactool_normal_ctrl",button_tactool_normal_ctrl);
    com_rsb->register_external("/foo/update_contact_frame",button_update_contact_frame);
    com_rsb->register_external("/foo/brake",button_brake);
    com_rsb->register_external("/foo/nobrake",button_nobrake);
    com_rsb->register_external("/foo/closeprog",button_closeprog);
    com_rsb->register_external("/foo/linexlen",slider_linexlen);
    com_rsb->register_external("/foo/lineylen",slider_lineylen);
    com_rsb->register_external("/foo/rotatexangle",slider_rotatexangle);
    com_rsb->register_external("/foo/rotateyangle",slider_rotateyangle);

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

    js.header.frame_id="frame";
    nv_est_marker_pub = nh->advertise<visualization_msgs::Marker>("nv_est_marker", 2);

    jsPub = nh->advertise<sensor_msgs::JointState> ("joint_states", 2);
    ros::spinOnce();

    br = new tf::TransformBroadcaster();

    std::cout<<"ros init finished"<<std::endl;
#endif
}



void run_rightarm(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){

        //contact status detection
        if(cdt->get_ctc_status(left_myrmex_msg.contactflag) == initcontact){
            //the normal direction will be esimated while the first contact is done.
            nv_est();
        }

//        kuka_right_arm->update_robot_stiffness();
        kuka_right_arm->get_joint_position_act();
        kuka_right_arm->update_robot_state();
        right_rs->updated(kuka_right_arm);
        //using kuka estimated force/torque control
//        kuka_right_arm->getTcpFtCalib(estkukaforce);
//        filtered_force = cf_filter->push(estkukaforce);
//        std::cout<<"force are "<<estkukaforce(0)<<","<<estkukaforce(1)<<","<<estkukaforce(2)<<std::endl;
//        mutex_force.lock();
//        for(int i = 0; i < 3; i++){
//            ft(i) = filtered_force[i];
//            ft(i+3) = estkukamoment(i);
//        }
//        mutex_force.unlock();
        //collect robot-eef position into a deque for computing the estimating nv
        col_est_nv();
        col_est_xy();
        //using all kinds of controllers to update the reference
        mutex_act.lock();
        for(unsigned int i = 0; i < right_ac_vec.size();i++){
            if(right_task_vec[i]->mt == JOINTS)
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i]);
            if(right_task_vec[i]->mt == FORCE){
                mutex_force.lock();
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i],ft,right_rs);
                mutex_force.unlock();
            }
            if(right_task_vec[i]->mt == TACTILE){
                mutex_tac.lock();
                right_ac_vec[i]->update_robot_reference(mt_ptr,kuka_right_arm,right_task_vec[i],&left_myrmex_msg);
                mutex_tac.unlock();
            }
        }
        //update with act_vec
        right_ac_vec[0]->llv.setZero();
        right_ac_vec[0]->lov.setZero();
        mutex_act.unlock();

        //use CBF to compute the desired joint angle rate
        kuka_right_arm->update_cbf_controller();
        kuka_right_arm->set_joint_command(rmt);
        com_okc->controller_update = true;
        com_okc->data_available = false;
    }
}


int main(int argc, char* argv[])
{
    //for data recording
    std::string data_f ("/tmp/");

    #ifdef HAVE_ROS
        ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
    #endif
    init();
    #ifdef HAVE_ROS
    //start ros run thread
    Timer thrd_rosrun(run);
    thrd_rosrun.setSingleShot(false);
    thrd_rosrun.setInterval(Timer::Interval(1));
    thrd_rosrun.start(true);
    #endif

    //start myrmex read thread
    Timer thrd_myrmex_read(tactileviarsb);
    thrd_myrmex_read.setSingleShot(false);
    thrd_myrmex_read.setInterval(Timer::Interval(5));
    thrd_myrmex_read.start(true);

    //start myrmex read thread
    Timer thrd_vismarker_read(vismarkerviarsb);
    thrd_vismarker_read.setSingleShot(false);
    thrd_vismarker_read.setInterval(Timer::Interval(30));
    thrd_vismarker_read.start(true);


    //start kuka arm control thread
    Timer thrd_kuka_ctrl(run_rightarm);
    thrd_kuka_ctrl.setSingleShot(false);
    thrd_kuka_ctrl.setInterval(Timer::Interval(1));
    thrd_kuka_ctrl.start(true);
    #ifdef HAVE_ROS
    //start ros publisher thread
    Timer thrd_rospublisher(ros_publisher);
    thrd_rospublisher.setSingleShot(false);
    thrd_rospublisher.setInterval(Timer::Interval(20));
    thrd_rospublisher.start(true);
    #endif
    //main thread is hanging
    while(!StopFlag){
    }
    //stop ros
    #ifdef HAVE_ROS
    ros::shutdown();
    delete nh;
    #endif
    //stop robot
    //todo
    #ifdef HAVE_ROS
    //stop the timer
    thrd_rosrun.stop();
    #endif
    thrd_kuka_ctrl.stop();
    thrd_myrmex_read.stop();
    thrd_vismarker_read.stop();
    #ifdef HAVE_ROS
    thrd_rospublisher.stop();
    #endif
    return 0;
}




