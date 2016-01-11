
/*
 ============================================================================
 Name        : tactoolservo.cpp
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

#include <deque> //for estimating the normal direction of tool-end

//desired contact pressure
#define TAC_F 0.5
#define NV_EST_LEN 500

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
Robot *kuka_left_arm;
std::vector<ActController *> left_ac_vec;
std::vector<Task *> left_task_vec;
TaskNameT left_taskname;

ComRSB *com_rsb;
RsbDataType rdtleftkuka;
RsbDataType rdtlefttac;
myrmex_msg left_myrmex_msg;
ParameterManager* pm;
RobotState *left_rs;
kuka_msg left_kuka_msg;
gamaFT *ft_gama;
PCAFeature *pcaf;

#define newP_x -0.1
#define newP_y 0.4
#define newP_z 0.30

#define newO_x 0.0
#define newO_y M_PI/2;
#define newO_z 0.0;


double initP_x,initP_y,initP_z;
double initO_x,initO_y,initO_z;
//arm_payload gravity(hand+tool-in-hand or midfinger)
Eigen::Vector3d arm_payload_g;


//using mutex locking controller ptr while it is switching.
std::mutex mutex_act, mutex_force,mutex_tac,mutex_ft;
//estimated force/torque from fri
Eigen::Vector3d estkukaforce,estkukamoment;
Eigen::Vector3d filtered_force;
Eigen::VectorXd ft;
TemporalSmoothingFilter<Eigen::Vector3d>* cf_filter;
uint32_t marker_shape;
ToolNameT tn;

bool StopFlag;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_f_filter;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_t_filter;

//define the robot control mode: normal mode vs psudo_gravity_compensation mode
RobotModeT rmt;

//save the last 500 position of robot end-effector
std::deque<Eigen::Vector3d> robot_eef_deque;
bool rec_flag_nv_est;
bool vis_est_ort;
Eigen::Vector3d est_tool_nv;
Eigen::Matrix3d est_tool_ort;
Eigen::Matrix3d rel_eef_tactool;

void tactileviarsb(){
    //via network--RSB, the contact information are obtained.
    mutex_tac.lock();
    com_rsb->tactile_receive(left_myrmex_msg,"leftmyrmex");
//    std::cout<<"tactile output"<<std::endl;
//    std::cout<<"myrmex readout "<<left_myrmex_msg.cogx<<","<<left_myrmex_msg.cogy<<std::endl;
    left_kuka_msg.p = left_rs->robot_position["eef"];
    left_kuka_msg.o = left_rs->robot_orien["eef"];
    left_kuka_msg.ft.setZero(6);
    mutex_tac.unlock();
}

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}

void tactool_force_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.forcet = F_MAINTAIN;
    left_ac_vec.push_back(new ForceServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new ForceServoTask(left_taskname.forcet));
    left_task_vec.back()->mt = FORCE;
    left_task_vec.back()->set_desired_cf_kuka(3);
    mutex_act.unlock();
    std::cout<<"maintain the force"<<std::endl;
}

void tactool_tactile_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = CONTACT_FORCE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_mid(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for maintain contact"<<std::endl;
}

void tactool_taxel_sliding_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = CONTACT_POINT_FORCE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_mid(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}


void tactool_exploring_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RLXN;
    left_ac_vec.push_back(new ProActController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = COVER_OBJECT_SURFACE;
    left_ac_vec.push_back(new TacServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_mid(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}

void tactool_cablefollow_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.prot = RLZN;
    left_ac_vec.push_back(new ProActController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new KukaSelfCtrlTask(left_taskname.prot));
    left_task_vec.back()->mft = LOCAL;
    left_task_vec.back()->mt = JOINTS;

    left_taskname.tact = LINEAR_TRACKING;
    left_ac_vec.push_back(new TacServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_mid(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}

void tactool_taxel_rolling_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = COVER_OBJECT_SURFACE;
    left_ac_vec.push_back(new TacServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_mid(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for rolling to the desired point"<<std::endl;
}


void tactool_grav_comp_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to psudo_gravity_compasenstation control"<<std::endl;
    rmt = PsudoGravityCompensation;
    rec_flag_nv_est = true;
}

void tactool_normal_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to normal control"<<std::endl;
    rmt = NormalMode;
}

void nv_est_cb(boost::shared_ptr<std::string> data){
    pcaf->GetData(robot_eef_deque);
    est_tool_nv = pcaf->getSlope_batch();
    est_tool_nv.normalize();
    est_tool_ort = gen_ort_basis(est_tool_nv);
    //Ttool2eef = Tg2eef * Ttool2g; to this end, the Ttool can be updated by Ttool2g = Teef2g * Ttool2eef;
    //which is Ttool = Teef * rel_eef_tactool;
    rel_eef_tactool = left_rs->robot_orien["robot_eef"].transpose() * est_tool_ort;
    rec_flag_nv_est = false;
    vis_est_ort = true;
    std::cout<<"normal direction is "<<est_tool_nv(0)<<","<<est_tool_nv(1)<<","<<est_tool_nv(2)<<std::endl;
    std::cout<<"orthogonal basis "<<est_tool_ort<<std::endl;
}

void col_est_nv(){
    if((rec_flag_nv_est == true)&&(left_myrmex_msg.contactflag!=true)){
        std::cout<<"robot trajectory is "<<left_rs->robot_position["robot_eef"](0)<<","\
                   <<left_rs->robot_position["robot_eef"](1)<<","<<left_rs->robot_position["robot_eef"](2)<<std::endl;
        robot_eef_deque.push_back(left_rs->robot_position["robot_eef"]);
        robot_eef_deque.pop_front();
    }
}


void moveto_cb(boost::shared_ptr<std::string> data){
    rec_flag_nv_est = true;
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) =  -0.1;
    p(1) =  left_rs->robot_position["eef"](1);
    p(2) = left_rs->robot_position["eef"](2);;

    o(0) = 0.0;
    o(1) = M_PI/2;
    o(2) = 0.0;
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_ac_vec.push_back(new ProActController(*pm));
    left_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    left_task_vec.back()->mt = JOINTS;
    left_task_vec.back()->mft = GLOBAL;
    left_task_vec.back()->set_desired_p_eigen(p);
    left_task_vec.back()->set_desired_o_ax(o);
    mutex_act.unlock();
    std::cout<<"robot self movement and move to new pose"<<std::endl;
}

void ftcalib_cb(boost::shared_ptr<std::string> data){
    kuka_left_arm->calibForce(1000);
}

void gamaftcalib_cb(boost::shared_ptr<std::string> data){
    std::cout<<"calibrate gama ft sensor is actived"<<std::endl;
    ft_gama->calibFT(500);
    arm_payload_g = left_rs->robot_orien["robot_eef"]*ft_gama->mean_ft_f;
    std::cout<<"tool bias: "<<left_rs->robot_orien["robot_eef"]*ft_gama->mean_ft_f<<std::endl;
}



void brake_cb(boost::shared_ptr<std::string> data){
    com_okc->start_brake();
}

void nobrake_cb(boost::shared_ptr<std::string> data){
    com_okc->release_brake();
}


#ifdef HAVE_ROS
int counter_t = 0;
// receive FT Sensor data
void
recvFT(const geometry_msgs::WrenchStampedConstPtr& msg){

    mutex_ft.lock();
//    std::cout <<"Fx"<<"\t"<<"Fy"<<"\t"<<"Fz"<<"\t"<<"Tx"<<"\t"<<"Ty"<<"\t"<<"Tz"<<std::endl;
//    std::cout << msg->wrench.force.x<<"\t"<<msg->wrench.force.y<<"\t"<<msg->wrench.force.z<<"\t"<<msg->wrench.torque.x<<"\t"<<msg->wrench.torque.y<<"\t"<<msg->wrench.torque.z<<std::endl;
    ft_gama->raw_ft_f(0) = msg->wrench.force.x;
    ft_gama->raw_ft_f(1) = msg->wrench.force.y-0.1;
    ft_gama->raw_ft_f(2) = msg->wrench.force.z;
    ft_gama->raw_ft_t(0) = msg->wrench.torque.x;
    ft_gama->raw_ft_t(1) = msg->wrench.torque.y;
    ft_gama->raw_ft_t(2) = msg->wrench.torque.z;


    //get rid of tool-pokingstick gravity
    ft_gama->raw_ft_f = ft_gama->raw_ft_f + left_rs->robot_orien["robot_eef"].transpose()*(-1)*arm_payload_g;
    counter_t ++;
    Eigen::Vector3d tmp;
    tmp.setZero();

    ft_gama->calib_ft_f = ft_gama->raw_ft_f;
    ft_gama->calib_ft_t = ft_gama->raw_ft_t;
    ft_gama->filtered_gama_f = gama_f_filter->push(ft_gama->calib_ft_f);
    ft_gama->filtered_gama_t = gama_t_filter->push(ft_gama->calib_ft_t);

//    if(counter_t%500==0){
////        tmp = left_rs->robot_orien["robot_eef"].transpose()*arm_payload_g;
////        std::cout<<"projected value: "<<tmp(0)<<","<<tmp(1)<<","<<tmp(2)<<std::endl;
//        counter_t = 0;
//        std::cout<<"estimated contact force: "<<ft_gama->filtered_gama_f(0)<<","<<ft_gama->filtered_gama_f(1)<<","<<ft_gama->filtered_gama_f(2)<<std::endl;
//    }
    mutex_ft.unlock();
}


#endif


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
        js.position[i]=0;
        js.position[i+7]=left_rs->JntPosition_mea[i];
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
        nv_est_marker.points[0].x = left_rs->robot_position["eef"](0);
        nv_est_marker.points[0].y = left_rs->robot_position["eef"](1);
        nv_est_marker.points[0].z = left_rs->robot_position["eef"](2);

        nv_est_marker.points[1].x = left_rs->robot_position["eef"](0)+est_tool_nv(0)/100;
        nv_est_marker.points[1].y = left_rs->robot_position["eef"](1)+est_tool_nv(1)/100;
        nv_est_marker.points[1].z = left_rs->robot_position["eef"](2)+est_tool_nv(2)/100;

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
        tf::matrixEigenToTF (est_tool_ort, tfR);
        transform.setOrigin( tf::Vector3(left_rs->robot_position["eef"](0), left_rs->robot_position["eef"](1), left_rs->robot_position["eef"](2)) );
        transform.setBasis(tfR);
        br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "est_tactool_frame"));
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

    std::string selfpath = get_selfpath();
    robot_eef_deque.assign(NV_EST_LEN, Eigen::Vector3d::Zero());
    pcaf = new PCAFeature(NV_EST_LEN);
    est_tool_nv.setZero();
    est_tool_ort.setIdentity();
    rel_eef_tactool.setIdentity();
    rec_flag_nv_est = false;
    vis_est_ort = false;
    rmt = NormalMode;
    ft_gama = new gamaFT;
    arm_payload_g.setZero();
    tn = tactool;
    StopFlag = false;
    //declare the cb function
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_force(tactool_force_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_tactile(tactool_tactile_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_taxel_sliding(tactool_taxel_sliding_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_taxel_rolling(tactool_taxel_rolling_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_exploring(tactool_exploring_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_moveto(moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_nv_est(nv_est_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_ftcalib(ftcalib_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_gamaftcalib(gamaftcalib_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_follow(tactool_cablefollow_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_grav_comp_ctrl(tactool_grav_comp_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_normal_ctrl(tactool_normal_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_brake(brake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_nobrake(nobrake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);

    std::string config_filename = selfpath + "/etc/left_arm_mid_param.xml";
    if(is_file_exist(config_filename.c_str()) == false){
        config_filename = "left_arm_mid_param.xml";
        if(is_file_exist(config_filename.c_str()) == false){
            std::cout<<"not find the tactile servo controller configure file"<<std::endl;
            exit(0);
        }
    }

    pm = new ParameterManager(config_filename);
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc,tn);
    left_rs = new RobotState(kuka_left_arm);
    kuka_left_arm->get_joint_position_act();
    kuka_left_arm->update_robot_state();
    left_rs->updated(kuka_left_arm);
    std::cout<<"state in the initialized stage are"<<left_rs->robot_position["eef"]<<std::endl;
    left_ac_vec.push_back(new ProActController(*pm));
    left_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) = initP_x = left_rs->robot_position["eef"](0);
    p(1) = initP_y= left_rs->robot_position["eef"](1);
    p(2) = initP_z= left_rs->robot_position["eef"](2);

    o = tm2axisangle(left_rs->robot_orien["eef"]);
    initO_x = o(0);
    initO_y = o(1);
    initO_z = o(2);
    left_task_vec.back()->set_desired_p_eigen(p);
    left_task_vec.back()->set_desired_o_ax(o);
    kuka_left_arm->setAxisStiffnessDamping(left_ac_vec.back()->pm.stiff_ctrlpara.axis_stiffness, \
                                           left_ac_vec.back()->pm.stiff_ctrlpara.axis_damping);
    com_rsb = new ComRSB();
    rdtleftkuka = LeftKukaEff;
    rdtlefttac = LeftMyrmex;
    com_rsb->add_msg(rdtleftkuka);
    com_rsb->add_msg(rdtlefttac);
    ft.setZero(6);
    estkukaforce.setZero();
    estkukamoment.setZero();
    cf_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(5,Average,Eigen::Vector3d(0,0,0));
    gama_f_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    gama_t_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    //register cb function
    com_rsb->register_external("/foo/moveto",button_moveto);
    com_rsb->register_external("/foo/nv_est",button_nv_est);
    com_rsb->register_external("/foo/tactool_force",button_tactool_force);
    com_rsb->register_external("/foo/tactool_tactile",button_tactool_tactile);
    com_rsb->register_external("/foo/tactool_taxel_sliding",button_tactool_taxel_sliding);
    com_rsb->register_external("/foo/tactool_taxel_rolling",button_tactool_taxel_rolling);
    com_rsb->register_external("/foo/tactool_exploring",button_tactool_exploring);
    com_rsb->register_external("/foo/ftcalib",button_ftcalib);
    com_rsb->register_external("/foo/gamaftcalib",button_gamaftcalib);
    com_rsb->register_external("/foo/tactool_follow",button_tactool_follow);
    com_rsb->register_external("/foo/tactool_grav_comp_ctrl",button_tactool_grav_comp_ctrl);
    com_rsb->register_external("/foo/tactool_normal_ctrl",button_tactool_normal_ctrl);
    com_rsb->register_external("/foo/brake",button_brake);
    com_rsb->register_external("/foo/nobrake",button_nobrake);
    com_rsb->register_external("/foo/closeprog",button_closeprog);

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



void run_leftarm(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
//        kuka_left_arm->update_robot_stiffness();
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        left_rs->updated(kuka_left_arm);
        //using kuka estimated force/torque control
//        kuka_left_arm->getTcpFtCalib(estkukaforce);
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
        //using all kinds of controllers to update the reference
        mutex_act.lock();
        for(unsigned int i = 0; i < left_ac_vec.size();i++){
            if(left_task_vec[i]->mt == JOINTS)
                left_ac_vec[i]->update_robot_reference(kuka_left_arm,left_task_vec[i]);
            if(left_task_vec[i]->mt == FORCE){
                mutex_force.lock();
                left_ac_vec[i]->update_robot_reference(kuka_left_arm,left_task_vec[i],ft,left_rs);
                mutex_force.unlock();
            }
            if(left_task_vec[i]->mt == TACTILE){
                mutex_tac.lock();
                left_ac_vec[i]->update_robot_reference(kuka_left_arm,left_task_vec[i],&left_myrmex_msg);
                mutex_tac.unlock();
            }
        }
        //update with act_vec
        left_ac_vec[0]->llv.setZero();
        left_ac_vec[0]->lov.setZero();
        mutex_act.unlock();

        //use CBF to compute the desired joint angle rate
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command(rmt);
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
        ros::Subscriber	GammaFtSub;
        std::cout<<"Connect to ATI FT sensor"<<std::endl;
        GammaFtSub=nh->subscribe("/ft_sensor/wrench", 1, // buffer size
                                &recvFT);


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


    //start kuka arm control thread
    Timer thrd_kuka_ctrl(run_leftarm);
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
    #ifdef HAVE_ROS
    thrd_rospublisher.stop();
    #endif
    return 0;
}




