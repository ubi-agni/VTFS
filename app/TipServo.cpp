/*
 ============================================================================
 Name        : TipServo.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : teensytactile fingertip tactile servoing
 ============================================================================
 */

//for ROS
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <sr_robot_msgs/UBI0All.h>
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

#include "fingertiptac.h"
#include "midtacfeature.h"
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

//for feature extact from teensyfingertip
#include "midtacfeature.h"

//teensy fingertip taxel num
#define TAC_NUM 12
//desired contact pressure
#define TAC_F 0.8

#ifdef HAVE_ROS
// ROS objects
tf::TransformBroadcaster *br;
sensor_msgs::JointState js;
ros::Publisher jsPub;
ros::NodeHandle *nh;
ros::Publisher marker_pub,marker_array_pub,marker_nvarray_pub;
ros::Publisher act_marker_pub,act_marker_nv_pub,gamma_force_marker_pub,act_taxel_pub,act_taxel_nv_pub;
ros::Publisher des_ct_marker_pub,des_ct_nv_marker_pub;
#endif
std::ofstream Tposition,Tft;

ComOkc *com_okc;
Robot *kuka_left_arm;
std::vector<ActController *> left_ac_vec;
std::vector<Task *> left_task_vec;
TaskNameT left_taskname;

ComRSB *com_rsb;
RsbDataType rdtleftkuka;
ParameterManager* pm;
RobotState *left_rs;
kuka_msg left_kuka_msg;
gamaFT *ft_gama;

#define newP_x -0.1
#define newP_y 0.2
#define newP_z 0.20

#define newO_x 0.0
#define newO_y M_PI/2;
#define newO_z 0.0;


double initP_x,initP_y,initP_z;
double initO_x,initO_y,initO_z;
Eigen::Vector3d tool_vec_g;


//using mutex locking controller ptr while it is switching.
std::mutex mutex_act, mutex_force,mutex_tac,mutex_ft;
//estimated force/torque from fri
Eigen::Vector3d estkukaforce,estkukamoment;
Eigen::Vector3d filtered_force;
Eigen::VectorXd ft;
TemporalSmoothingFilter<Eigen::Vector3d>* cf_filter;
//initialize fingertip instance
FingertipTac *ftt;
uint32_t marker_shape;
ToolNameT tn;
bool vmt;
float tac_index;

bool StopFlag;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_f_filter;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_t_filter;

//flag to record data in order to use gamma ft sensor to calbirate mid tactile fingertip
bool start_taccalib_rec;
//define the robot control mode: normal mode vs psudo_gravity_compensation mode
RobotModeT rmt;

//for moving tactile pattern extaction
MidTacFeature *mtf;
//estimated contact point position, normal direction
Eigen::Vector3d cp_g,cendp_g,cnv_g,cendp_g_gamma;
//estimated contact taxel posion and normal direction
Eigen::Vector3d c_taxel_p_l,c_taxel_nv_l,c_taxel_p_g,c_taxel_endp_g,c_taxel_nv_g;

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}

void tip_force_cb(boost::shared_ptr<std::string> data){
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

void tip_tactile_cb(boost::shared_ptr<std::string> data){
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

void tip_taxel_sliding_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = CONTACT_POINT_FORCE_TRACKING;
    left_ac_vec.push_back(new TacServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_mid(TAC_F);
    left_task_vec.back()->set_desired_taxel_mid((int)tac_index);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}


void tip_exploring_cb(boost::shared_ptr<std::string> data){
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
    left_task_vec.back()->set_taxelfb_type_mid(TAXEL_POSITION);
    //compute the desired cp on fingertip-Area I
    left_task_vec.back()->set_desired_position_mid(ftt->get_Center_position(1));
    left_task_vec.back()->set_desired_nv_mid(ftt->get_Center_nv(1));
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding/rolling to the desired point"<<std::endl;
}

void tip_cablefollow_cb(boost::shared_ptr<std::string> data){
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
//    left_task_vec.back()->set_desired_taxel_mid((int)tac_index);
    //using the specified position
    left_task_vec.back()->set_taxelfb_type_mid(TAXEL_POSITION);
    //compute the desired cp on fingertip-Area I
    left_task_vec.back()->set_desired_position_mid(ftt->get_Center_position(1));
    left_task_vec.back()->set_desired_nv_mid(ftt->get_Center_nv(1));
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}

void tip_taxel_rolling_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.tact = COVER_OBJECT_SURFACE;
    left_ac_vec.push_back(new TacServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new TacServoTask(left_taskname.tact));
    left_task_vec.back()->mt = TACTILE;
    left_task_vec.back()->set_desired_cf_mid(TAC_F);
    //using the sepcified taxel as the desired point
//    left_task_vec.back()->set_desired_taxel_mid((int)tac_index);
    //using the specified position
    left_task_vec.back()->set_taxelfb_type_mid(TAXEL_POSITION);
    //compute the desired cp on fingertip-Area I
    left_task_vec.back()->set_desired_position_mid(ftt->get_Center_position(1));
    left_task_vec.back()->set_desired_nv_mid(ftt->get_Center_nv(1));
    mutex_act.unlock();
    std::cout<<"tactile servoing for rolling to the desired point"<<std::endl;
}

void tip_follow_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    left_ac_vec.clear();
    left_task_vec.clear();
    left_taskname.forcet = F_CURVETRACKING;
    left_ac_vec.push_back(new ForceServoController(*pm));
    left_ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task_vec.push_back(new ForceServoTask(left_taskname.forcet));
    left_task_vec.back()->mt = FORCE;
    left_task_vec.back()->set_desired_cf_kuka(3);
    mutex_act.unlock();
    std::cout<<"curve tracking"<<std::endl;
}

void tip_grav_comp_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to psudo_gravity_compasenstation control"<<std::endl;
    rmt = PsudoGravityCompensation;
}

void tip_normal_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to normal control"<<std::endl;
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
    rmt = NormalMode;
}


void moveto_cb(boost::shared_ptr<std::string> data){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = newP_x;
    p(1) = newP_y;
    p(2) = newP_z;

    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
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
    tool_vec_g = left_rs->robot_orien["robot_eef"]*ft_gama->mean_ft_f;
    std::cout<<"tool bias: "<<left_rs->robot_orien["robot_eef"]*ft_gama->mean_ft_f<<std::endl;
}


void taccalib_rec_cb(boost::shared_ptr<std::string> data){
    start_taccalib_rec = true;
}

void brake_cb(boost::shared_ptr<std::string> data){
    com_okc->start_brake();
}

void nobrake_cb(boost::shared_ptr<std::string> data){
    com_okc->release_brake();
}

void updateadmittance_cb(boost::shared_ptr<std::string> data){
//    mutex_act.lock();
//    left_taskname.tact = Z_ORIEN_TRACKING;
//    ac->update_controller_para();
//    ac->updateTacServoCtrlParam(left_taskname.tact);
//    mutex_act.unlock();
}

void markerarray_cb(boost::shared_ptr<std::string> data){
    if((*data == "true"))
        vmt = true;
    else
        vmt = false;
}

void tacindex_cb(boost::shared_ptr<std::string> data){
   tac_index = std::stoi(*data);
}

#ifdef HAVE_ROS
// receive Tactile data from UBI Fingertips
void
recvTipTactile(const sr_robot_msgs::UBI0AllConstPtr& msg){
    // for first sensor each taxel
    //Todo: clear data and all estimated value;
    mutex_tac.lock();
    double press_val;
    Eigen::Vector3d pos_val,nv_val;
//    std::cout <<"0"<<"\t"<<"1"<<"\t"<<"2"<<"\t"<<"3"<<"\t"<<"4"<<"\t"<<"5"<<"\t"<<"6"<<"\t"<<"7"<<"\t"<<"8"<<"\t"<<"9"<<"\t"<<"10"<<"\t"<<"11"<<"\t"<<std::endl;
//    std::cout << std::fixed;
    ROS_ASSERT(ftt->data.fingertip_tac_pressure.size() == msg->tactiles[0].distal.size());
    for(size_t j = 0; j < msg->tactiles[0].distal.size(); ++j) {
        press_val= 1.0-(msg->tactiles[0].distal[j]/1023.0);
        if(press_val < MID_THRESHOLD) press_val = 0.0;
        ftt->data.fingertip_tac_pressure[j] = press_val;
//        std::cout<<std::setprecision(5)<<press_val<<"\t";
    }
//    std::cout<<std::endl;
    mutex_tac.unlock();
}

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
    ft_gama->raw_ft_f = ft_gama->raw_ft_f + left_rs->robot_orien["robot_eef"].transpose()*(-1)*tool_vec_g;
    counter_t ++;
    Eigen::Vector3d tmp;
    tmp.setZero();

    ft_gama->calib_ft_f = ft_gama->raw_ft_f;
    ft_gama->calib_ft_t = ft_gama->raw_ft_t;
    ft_gama->filtered_gama_f = gama_f_filter->push(ft_gama->calib_ft_f);
    ft_gama->filtered_gama_t = gama_t_filter->push(ft_gama->calib_ft_t);

//    if(counter_t%500==0){
////        tmp = left_rs->robot_orien["robot_eef"].transpose()*tool_vec_g;
////        std::cout<<"projected value: "<<tmp(0)<<","<<tmp(1)<<","<<tmp(2)<<std::endl;
//        counter_t = 0;
//        std::cout<<"estimated contact force: "<<ft_gama->filtered_gama_f(0)<<","<<ft_gama->filtered_gama_f(1)<<","<<ft_gama->filtered_gama_f(2)<<std::endl;
//    }
    if(start_taccalib_rec == true){
        mutex_tac.lock();
        Tft<<ft_gama->filtered_gama_f(0)<<","<<ft_gama->filtered_gama_f(1)<<","<<ft_gama->filtered_gama_f(2)<<",";
        Tft<<ftt->data.fingertip_tac_pressure[0]<<","<<ftt->data.fingertip_tac_pressure[1]<<","<<ftt->data.fingertip_tac_pressure[2]<<","\
                 <<ftt->data.fingertip_tac_pressure[3]<<","<<ftt->data.fingertip_tac_pressure[4]<<","<<ftt->data.fingertip_tac_pressure[5]<<","\
                 <<ftt->data.fingertip_tac_pressure[6]<<","<<ftt->data.fingertip_tac_pressure[7]<<","<<ftt->data.fingertip_tac_pressure[8]<<","\
                 <<ftt->data.fingertip_tac_pressure[9]<<","<<ftt->data.fingertip_tac_pressure[10]<<","<<ftt->data.fingertip_tac_pressure[11]<<std::endl;
        mutex_tac.unlock();
    }
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
    bool contact_f = false;
    mutex_tac.lock();
    contact_f = ftt->isContact(ftt->data);
    mutex_tac.unlock();

    // Publish the marker
    if ((marker_pub.getNumSubscribers() >= 1)){
        //prepare marker info
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/rh_ffdistal";
        marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "KukaRos";
        marker.id = 0;
        marker.frame_locked = true;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = marker_shape;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        mutex_tac.lock();
        marker.pose.position.x = ftt->data.fingertip_tac_position.at((int)tac_index)(0)/1000;
        marker.pose.position.y = ftt->data.fingertip_tac_position.at((int)tac_index)(1)/1000;
        marker.pose.position.z = ftt->data.fingertip_tac_position.at((int)tac_index)(2)/1000;
        mutex_tac.unlock();
        /*        std::cout<<"fingertip tacxel position "<<(int)tac_index<< "is "\
                        <<ftt->data.fingertip_tac_position.at((int)tac_index)(0)<<","\
                         <<ftt->data.fingertip_tac_position.at((int)tac_index)(1)<<","\
                            <<ftt->data.fingertip_tac_position.at((int)tac_index)(2)<<std::endl;*/
        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        if(contact_f == true)
            marker.color.a = 1.0;
        else
            marker.color.a = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = .002;
        marker.scale.y = .002;
        marker.scale.z = .002;

        marker.lifetime = ros::Duration();
        marker_pub.publish(marker);
    }

    //publish the actived taxel
    if ((act_taxel_pub.getNumSubscribers() >= 1)){
        visualization_msgs::Marker act_taxel;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        act_taxel.color.r = 0.0f;
        act_taxel.color.g = 0.0f;
        act_taxel.color.b = ftt->pressure;
        if(contact_f == true)
            act_taxel.color.a = 1.0;
        else
            act_taxel.color.a = 0.0;
        mutex_tac.unlock();

        act_taxel.header.frame_id = "frame";
        act_taxel.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        act_taxel.ns = "KukaRos";
        act_taxel.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        act_taxel.type = visualization_msgs::Marker::CUBE;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        act_taxel.action = visualization_msgs::Marker::ADD;
        act_taxel.pose.position.x = c_taxel_p_g(0);
        act_taxel.pose.position.y = c_taxel_p_g(1);
        act_taxel.pose.position.z = c_taxel_p_g(2);
        act_taxel.pose.orientation.x = 0.0;
        act_taxel.pose.orientation.y = 0.0;
        act_taxel.pose.orientation.z = 0.0;
        act_taxel.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        act_taxel.scale.x = .002;
        act_taxel.scale.y = .002;
        act_taxel.scale.z = .002;

        act_taxel.lifetime = ros::Duration();
        act_taxel_pub.publish(act_taxel);
    }
    //publish the actived taxel normal vector
    if(act_taxel_nv_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker act_taxel_nv;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        act_taxel_nv.color.r = 0.0f;
        act_taxel_nv.color.g = 0.0f;
        act_taxel_nv.color.b = 1.0f;
        if(contact_f == true)
            act_taxel_nv.color.a = 1.0;
        else
            act_taxel_nv.color.a = 0.0;
        mutex_tac.unlock();

        act_taxel_nv.header.frame_id = "frame";
        act_taxel_nv.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        act_taxel_nv.ns = "KukaRos";
        act_taxel_nv.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        act_taxel_nv.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        act_taxel_nv.action = visualization_msgs::Marker::ADD;

        act_taxel_nv.points.resize(2);
        act_taxel_nv.points[0].x = c_taxel_p_g(0);
        act_taxel_nv.points[0].y = c_taxel_p_g(1);
        act_taxel_nv.points[0].z = c_taxel_p_g(2);

        act_taxel_nv.points[1].x = c_taxel_endp_g(0);
        act_taxel_nv.points[1].y = c_taxel_endp_g(1);
        act_taxel_nv.points[1].z = c_taxel_endp_g(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        act_taxel_nv.scale.x = .001;
        act_taxel_nv.scale.y = .001;
        act_taxel_nv.scale.z = .001;

        act_taxel_nv.lifetime = ros::Duration();
        act_taxel_nv_pub.publish(act_taxel_nv);
    }

    //publish the actived position
    if ((act_marker_pub.getNumSubscribers() >= 1)){
        visualization_msgs::Marker act_marker;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        act_marker.color.r = ftt->pressure;
        act_marker.color.g = 0.0f;
        act_marker.color.b = 0.0f;
        if(contact_f == true)
            act_marker.color.a = 1.0;
        else
            act_marker.color.a = 0;
        mutex_tac.unlock();

        act_marker.header.frame_id = "frame";
        act_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        act_marker.ns = "KukaRos";
        act_marker.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        act_marker.type = visualization_msgs::Marker::CUBE;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        act_marker.action = visualization_msgs::Marker::ADD;
        act_marker.pose.position.x = cp_g(0);
        act_marker.pose.position.y = cp_g(1);
        act_marker.pose.position.z = cp_g(2);
        act_marker.pose.orientation.x = 0.0;
        act_marker.pose.orientation.y = 0.0;
        act_marker.pose.orientation.z = 0.0;
        act_marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        act_marker.scale.x = .002;
        act_marker.scale.y = .002;
        act_marker.scale.z = .002;

        act_marker.lifetime = ros::Duration();
        act_marker_pub.publish(act_marker);
    }
    //publish the actived normal vector
    if(act_marker_nv_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker act_marker_nv;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        act_marker_nv.color.r = 1.0f;
        act_marker_nv.color.g = 0.0f;
        act_marker_nv.color.b = 0.0f;
        if(contact_f == true)
            act_marker_nv.color.a = 1.0;
        else
            act_marker_nv.color.a = 0;
        mutex_tac.unlock();

        act_marker_nv.header.frame_id = "frame";
        act_marker_nv.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        act_marker_nv.ns = "KukaRos";
        act_marker_nv.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        act_marker_nv.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        act_marker_nv.action = visualization_msgs::Marker::ADD;

        act_marker_nv.points.resize(2);
        act_marker_nv.points[0].x = cp_g(0);
        act_marker_nv.points[0].y = cp_g(1);
        act_marker_nv.points[0].z = cp_g(2);

        act_marker_nv.points[1].x = cendp_g(0);
        act_marker_nv.points[1].y = cendp_g(1);
        act_marker_nv.points[1].z = cendp_g(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        act_marker_nv.scale.x = .001;
        act_marker_nv.scale.y = .001;
        act_marker_nv.scale.z = .001;

        act_marker_nv.lifetime = ros::Duration();
        act_marker_nv_pub.publish(act_marker_nv);
    }

    //publish the desired contact position
    if ((des_ct_marker_pub.getNumSubscribers() >= 1)){
        visualization_msgs::Marker des_ct_marker;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        des_ct_marker.color.r = 0.0f;
        des_ct_marker.color.g = 1.0f;
        des_ct_marker.color.b = 0.0f;
        if(contact_f == true)
            des_ct_marker.color.a = 1.0;
        else
            des_ct_marker.color.a = 0;
        mutex_tac.unlock();

        des_ct_marker.header.frame_id = "/rh_ffdistal";
        des_ct_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        des_ct_marker.ns = "KukaRos";
        des_ct_marker.id = 0;
        des_ct_marker.frame_locked = true;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        des_ct_marker.type = visualization_msgs::Marker::CUBE;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        des_ct_marker.action = visualization_msgs::Marker::ADD;
        des_ct_marker.pose.position.x = ftt->get_Center_position(1)(0)/1000;
        des_ct_marker.pose.position.y = ftt->get_Center_position(1)(1)/1000;
        des_ct_marker.pose.position.z = ftt->get_Center_position(1)(2)/1000;
        des_ct_marker.pose.orientation.x = 0.0;
        des_ct_marker.pose.orientation.y = 0.0;
        des_ct_marker.pose.orientation.z = 0.0;
        des_ct_marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        des_ct_marker.scale.x = .002;
        des_ct_marker.scale.y = .002;
        des_ct_marker.scale.z = .002;

        des_ct_marker.lifetime = ros::Duration();
        des_ct_marker_pub.publish(des_ct_marker);
    }
    //publish the desired contact normal vector
    if(des_ct_nv_marker_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker des_ct_nv_marker;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        des_ct_nv_marker.color.r = 0.0f;
        des_ct_nv_marker.color.g = 1.0f;
        des_ct_nv_marker.color.b = 0.0f;
        if(contact_f == true)
            des_ct_nv_marker.color.a = 1.0;
        else
            des_ct_nv_marker.color.a = 0;
        mutex_tac.unlock();

        des_ct_nv_marker.header.frame_id = "/rh_ffdistal";
        des_ct_nv_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        des_ct_nv_marker.ns = "KukaRos";
        des_ct_nv_marker.id = 0;
        des_ct_nv_marker.frame_locked = true;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        des_ct_nv_marker.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        des_ct_nv_marker.action = visualization_msgs::Marker::ADD;

        des_ct_nv_marker.points.resize(2);
        des_ct_nv_marker.points[0].x = ftt->get_Center_position(1)(0)/1000;
        des_ct_nv_marker.points[0].y = ftt->get_Center_position(1)(1)/1000;
        des_ct_nv_marker.points[0].z = ftt->get_Center_position(1)(2)/1000;

        des_ct_nv_marker.points[1].x = ftt->get_Center_position(1)(0)/1000 + ftt->get_Center_nv(1)(0)/100;
        des_ct_nv_marker.points[1].y = ftt->get_Center_position(1)(1)/1000 + ftt->get_Center_nv(1)(1)/100;
        des_ct_nv_marker.points[1].z = ftt->get_Center_position(1)(2)/1000 + ftt->get_Center_nv(1)(2)/100;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        des_ct_nv_marker.scale.x = .001;
        des_ct_nv_marker.scale.y = .001;
        des_ct_nv_marker.scale.z = .001;

        des_ct_nv_marker.lifetime = ros::Duration();
        des_ct_nv_marker_pub.publish(des_ct_nv_marker);
    }


    //publish the gamma force vector
    if(gamma_force_marker_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker gamma_force_marker;
        mutex_tac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        gamma_force_marker.color.r = 1.0f;
        gamma_force_marker.color.g = 0.0f;
        gamma_force_marker.color.b = 0.0f;
        if(contact_f == true)
            gamma_force_marker.color.a = 1.0;
        else
            gamma_force_marker.color.a = 0;
        mutex_tac.unlock();

        gamma_force_marker.header.frame_id = "frame";
        gamma_force_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        gamma_force_marker.ns = "KukaRos";
        gamma_force_marker.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        gamma_force_marker.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        gamma_force_marker.action = visualization_msgs::Marker::ADD;

        gamma_force_marker.points.resize(2);
        gamma_force_marker.points[0].x = cp_g(0);
        gamma_force_marker.points[0].y = cp_g(1);
        gamma_force_marker.points[0].z = cp_g(2);

        gamma_force_marker.points[1].x = cendp_g_gamma(0);
        gamma_force_marker.points[1].y = cendp_g_gamma(1);
        gamma_force_marker.points[1].z = cendp_g_gamma(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        gamma_force_marker.scale.x = .001;
        gamma_force_marker.scale.y = .001;
        gamma_force_marker.scale.z = .001;

        gamma_force_marker.lifetime = ros::Duration();
        gamma_force_marker_pub.publish(gamma_force_marker);
    }

    // Publish the markerarray
    if ((marker_array_pub.getNumSubscribers() >= 1)||(marker_nvarray_pub.getNumSubscribers() >= 1)){
        visualization_msgs::MarkerArray marker_array_msg;
        visualization_msgs::MarkerArray marker_nvarray_msg;
        marker_array_msg.markers.resize(12);
        marker_nvarray_msg.markers.resize(12);
        Eigen::Vector3d taxel_g, taxel_temp,taxel_nv_g,nv_endp;
        taxel_g.setZero();
        taxel_temp.setZero();
        taxel_nv_g.setZero();
        nv_endp.setZero();
        for ( int i = 0; i < 12; i++)
        {
            //publish the taxel position
            marker_array_msg.markers[i].header.frame_id = "/frame";
            marker_array_msg.markers[i].header.stamp = ros::Time::now();
            marker_array_msg.markers[i].ns = "KukaRos";
            marker_array_msg.markers[i].id = i;
            marker_array_msg.markers[i].type = visualization_msgs::Marker::SPHERE;
            marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;

            mutex_tac.lock();
            local2global(ftt->data.fingertip_tac_position.at(i)/1000,\
                         left_rs->robot_orien["eef"],taxel_temp);
            taxel_g = left_rs->robot_position["eef"] + taxel_temp;
            mutex_tac.unlock();
            marker_array_msg.markers[i].pose.position.x = taxel_g(0);
            marker_array_msg.markers[i].pose.position.y = taxel_g(1);
            marker_array_msg.markers[i].pose.position.z = taxel_g(2);
            marker_array_msg.markers[i].pose.orientation.x = 0.0;
            marker_array_msg.markers[i].pose.orientation.y = 0.0;
            marker_array_msg.markers[i].pose.orientation.z = 0.0;
            marker_array_msg.markers[i].pose.orientation.w = 1.0;
            marker_array_msg.markers[i].scale.x = .002;
            marker_array_msg.markers[i].scale.y = .002;
            marker_array_msg.markers[i].scale.z = .002;
            marker_array_msg.markers[i].color.a = 1.0;
            marker_array_msg.markers[i].color.r = 0.0;
            marker_array_msg.markers[i].color.g = 1.0;
            marker_array_msg.markers[i].color.b = 0.0;

            //publish taxel normal vector
            marker_nvarray_msg.markers[i].header.frame_id = "frame";
            marker_nvarray_msg.markers[i].header.stamp = ros::Time::now();
            marker_nvarray_msg.markers[i].ns = "KukaRos";
            marker_nvarray_msg.markers[i].id = i;
            marker_nvarray_msg.markers[i].type = visualization_msgs::Marker::ARROW;
            marker_nvarray_msg.markers[i].action = visualization_msgs::Marker::ADD;

            mutex_tac.lock();
            local2global(ftt->data.fingertip_tac_nv.at(i)/100,\
                         left_rs->robot_orien["eef"],taxel_nv_g);
            nv_endp = taxel_g + taxel_nv_g;
            mutex_tac.unlock();
            marker_nvarray_msg.markers[i].points.resize(2);
            marker_nvarray_msg.markers[i].points[0].x = taxel_g(0);
            marker_nvarray_msg.markers[i].points[0].y = taxel_g(1);
            marker_nvarray_msg.markers[i].points[0].z = taxel_g(2);

            marker_nvarray_msg.markers[i].points[1].x = nv_endp(0);
            marker_nvarray_msg.markers[i].points[1].y = nv_endp(1);
            marker_nvarray_msg.markers[i].points[1].z = nv_endp(2);

            marker_nvarray_msg.markers[i].scale.x = .001;
            marker_nvarray_msg.markers[i].scale.y = .001;
            marker_nvarray_msg.markers[i].scale.z = .001;
            marker_nvarray_msg.markers[i].color.a = 1.0;
            marker_nvarray_msg.markers[i].color.r = 0.0;
            marker_nvarray_msg.markers[i].color.g = 1.0;
            marker_nvarray_msg.markers[i].color.b = 0.0;
        }
        marker_array_pub.publish(marker_array_msg);
        marker_nvarray_pub.publish(marker_nvarray_msg);
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
    rmt = NormalMode;
    ft_gama = new gamaFT;
    tool_vec_g.setZero();
    start_taccalib_rec = false;

    ftt = new FingertipTac(12);
    tn = teensy_finger;
    StopFlag = false;
    tac_index = 0.0;
    //declare the cb function
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_force(tip_force_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_tactile(tip_tactile_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_taxel_sliding(tip_taxel_sliding_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_taxel_rolling(tip_taxel_rolling_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_exploring(tip_exploring_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_moveto(moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_ftcalib(ftcalib_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_gamaftcalib(gamaftcalib_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_taccalib_rec(taccalib_rec_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_follow(tip_cablefollow_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_grav_comp_ctrl(tip_grav_comp_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_normal_ctrl(tip_normal_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_brake(brake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_nobrake(nobrake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_updateadmittance(updateadmittance_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);
    boost::function<void(boost::shared_ptr<std::string>)> checkb_markerarray(markerarray_cb);
    boost::function<void(boost::shared_ptr<std::string>)> slider_tacindex(tacindex_cb);

    std::string config_filename = selfpath + "/etc/left_arm_mid_param.xml";
    std::cout<<"config file name is: "<<config_filename<<std::endl;
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
    com_rsb->add_msg(rdtleftkuka);
    ft.setZero(6);
    estkukaforce.setZero();
    estkukamoment.setZero();
    cf_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(5,Average,Eigen::Vector3d(0,0,0));
    gama_f_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    gama_t_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    int len = 50;
    mtf = new MidTacFeature(len);
    cp_g.setZero();
    cendp_g.setZero();
    cendp_g_gamma.setZero();
    cnv_g.setZero();
    c_taxel_nv_l.setZero();
    c_taxel_p_l.setZero();
    c_taxel_nv_g.setZero();
    c_taxel_p_g.setZero();
    c_taxel_endp_g.setZero();
    //register cb function
    com_rsb->register_external("/foo/moveto",button_moveto);
    com_rsb->register_external("/foo/tip_force",button_tip_force);
    com_rsb->register_external("/foo/tip_tactile",button_tip_tactile);
    com_rsb->register_external("/foo/tip_taxel_sliding",button_tip_taxel_sliding);
    com_rsb->register_external("/foo/tip_taxel_rolling",button_tip_taxel_rolling);
    com_rsb->register_external("/foo/tip_exploring",button_tip_exploring);
    com_rsb->register_external("/foo/ftcalib",button_ftcalib);
    com_rsb->register_external("/foo/gamaftcalib",button_gamaftcalib);
    com_rsb->register_external("/foo/taccalib_rec",button_taccalib_rec);
    com_rsb->register_external("/foo/tip_follow",button_tip_follow);
    com_rsb->register_external("/foo/tip_grav_comp_ctrl",button_tip_grav_comp_ctrl);
    com_rsb->register_external("/foo/tip_normal_ctrl",button_tip_normal_ctrl);
    com_rsb->register_external("/foo/brake",button_brake);
    com_rsb->register_external("/foo/nobrake",button_nobrake);
    com_rsb->register_external("/foo/updateadmittance",button_updateadmittance);
    com_rsb->register_external("/foo/closeprog",button_closeprog);
    com_rsb->register_external("/foo/sa-marker",checkb_markerarray);
    com_rsb->register_external("/foo/tacindex",slider_tacindex);

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

    marker_shape = visualization_msgs::Marker::CUBE;
    marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 2);
    marker_array_pub = nh->advertise<visualization_msgs::MarkerArray>("vis_marker_array", 2);
    marker_nvarray_pub = nh->advertise<visualization_msgs::MarkerArray>("vis_marker_nv_array", 2);
    act_marker_pub = nh->advertise<visualization_msgs::Marker>("act_marker", 2);
    act_marker_nv_pub = nh->advertise<visualization_msgs::Marker>("act_marker_nv", 2);
    gamma_force_marker_pub = nh->advertise<visualization_msgs::Marker>("gamma_force_marker", 2);
    act_taxel_pub = nh->advertise<visualization_msgs::Marker>("act_taxel", 2);
    act_taxel_nv_pub = nh->advertise<visualization_msgs::Marker>("act_taxel_nv", 2);
    des_ct_marker_pub = nh->advertise<visualization_msgs::Marker>("des_ct_marker", 2);
    des_ct_nv_marker_pub = nh->advertise<visualization_msgs::Marker>("des_ct_nv_marker", 2);

    jsPub = nh->advertise<sensor_msgs::JointState> ("joint_states", 2);
    ros::spinOnce();

    br = new tf::TransformBroadcaster();

    std::cout<<"ros init finished"<<std::endl;
#endif
}

void get_ft_vis_info(){
    //this function is only for the computation of force vector visualization for gamma sensor.
    //located in the contact point detected by mid sensor
    bool contact_f = false;
    Eigen::Vector3d l2g_temp;
    l2g_temp.setZero();
    mutex_tac.lock();
    contact_f = ftt->isContact(ftt->data);
    if(contact_f == true){
        local2global(ft_gama->filtered_gama_f/100,\
                     left_rs->robot_orien["eef"],l2g_temp);
        cendp_g_gamma = cp_g + l2g_temp;
    }
    mutex_tac.unlock();
}
void get_mid_info(){
    //computingt the contact information from mid sensor--contact position,estimated
    //contact force vector, and line feature estimation
    bool contact_f = false;
    Eigen::Vector3d slope;
    Eigen::Vector3d l2g_temp;
    slope.setZero();
    l2g_temp.setZero();
    mutex_tac.lock();
    contact_f = ftt->isContact(ftt->data);
    if(contact_f == true){
        int taxId;
        taxId = ftt->est_ct_taxelId(ftt->data);
        ftt->contactarea = ftt->WhichArea(taxId);
        //estimate contact taxel position, normal vector
        for(int i = 0; i < 3; i ++){
            c_taxel_p_l(i) = ftt->data.fingertip_tac_position.at(taxId)(i);
            c_taxel_nv_l(i) = ftt->data.fingertip_tac_nv.at(taxId)(i);
        }
        local2global(c_taxel_p_l/1000,\
                     left_rs->robot_orien["eef"],l2g_temp);
        c_taxel_p_g = left_rs->robot_position["eef"] + l2g_temp;
        local2global(c_taxel_nv_l/100,\
                     left_rs->robot_orien["eef"],l2g_temp);
        c_taxel_endp_g = c_taxel_p_g + l2g_temp;
        //estimate contact point position, normal vector
        ftt->est_ct_info(ftt->data);
        local2global(ftt->pos/1000,\
                     left_rs->robot_orien["eef"],l2g_temp);
        cp_g = left_rs->robot_position["eef"] + l2g_temp;
        local2global(ftt->nv/100,\
                     left_rs->robot_orien["eef"],l2g_temp);
        cendp_g = cp_g + l2g_temp;


        //compute the slope of accumulated tactile linear feature
        slope = mtf->getSlope(cp_g.transpose());
        ftt->get_slope(left_rs->robot_orien["eef"].transpose()*slope);
        Tposition<<slope[0]<<","<<slope[1]<<","<<slope[2]<<","\
                          <<cp_g[0]<<","<<cp_g[1]<<","<<cp_g[2]<<std::endl;
    }
    else{
        ftt->slope_clear();
    }
    mutex_tac.unlock();
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
        //using mid for control, get contact information
        get_mid_info();
        //get ft visulization information
        get_ft_vis_info();
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
                left_ac_vec[i]->update_robot_reference(kuka_left_arm,left_task_vec[i],ftt);
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
    Tposition.open((data_f+std::string("position.txt")).c_str());
    Tft.open((data_f+std::string("ft.txt")).c_str());
    #ifdef HAVE_ROS
        ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
        ros::Subscriber	tacTipSub,GammaFtSub;
        if (argc > 1){
            std::cout<<"set hand by user"<<std::endl;
            std::string handname(argv[1]);
            tacTipSub=nh->subscribe("/"+handname+"/teensy/tactile", 1, // buffer size
                                    &recvTipTactile);
        }
        else{
            //default is left hand
            std::cout<<"use the left hand as default"<<std::endl;
            tacTipSub=nh->subscribe("/lh/teensy/tactile", 1, // buffer size
                                    &recvTipTactile);
        }
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
    #ifdef HAVE_ROS
    thrd_rospublisher.stop();
    #endif
    return 0;
}




