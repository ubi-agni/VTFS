
/*
 ============================================================================
 Name        : HingedToolManip.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : force servo for right arm + schunk grasp the hinged tool assuming that
 *             the hinged origin and axis direction has been estimated in advanced. The estimation
 *              can be done with fingertip flap the tool
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
#include <geometry_msgs/Vector3Stamped.h>
#include <reba_tactile_msgs/cp_info.h>
#include <std_msgs/Bool.h>
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
#include "visactcontroller.h"
#include "visservotask.h"
#include "RobotState.h"
#include "Util.h"
#include <fstream>
#include <mutex>
#include "gamaft.h"
#include "RebaType.h"
#include "msgcontenttype.h"

//define the gravity of the schunk hand and its accessory
#define Grav_Schunk_Acc 21.63
#define tool_grav 0.15*9.81
#define des_force 2

#ifdef HAVE_ROS
// ROS objects
tf::TransformBroadcaster *br;
sensor_msgs::JointState js_la, js_ra,js_schunk ;
ros::Publisher jsPub_la,jsPub_ra;
ros::Publisher jsPub_schunk;
ros::NodeHandle *nh;
ros::Publisher gamma_force_marker_pub, hingedtool_axis_marker_pub;
ros::Publisher reba_robot_end_eff_pub;
ros::Publisher p_ct_pub;
ros::Publisher desired_po_test_pub,desired_vec_test_pub;
#endif

//ptr for openkc component
ComOkc *com_okc_right;
//ptr for right kuka robot
Robot *kuka_right_arm;
//ptr for  right kuka controller vector
std::vector<ActController *> right_ac_vec;
//ptr for right kuka task vector
std::vector<Task *> right_task_vec;
//defined left arm task name
TaskNameT right_taskname;

ComRSB *com_rsb;
RsbDataType rdtschunkjs;

//right kuka controller parameters
ParameterManager* right_pm;
//right kuka internal model
RobotState *right_rs;

//ptr for FT sensor on the left arm
gamaFT *ft_gama;
Eigen::VectorXd ft;

//predefined pose of right arm should go
#define right_newP_x 0.0
#define right_newP_y 0.3
#define right_newP_z 0.15

#define right_newO_x 0.0
#define right_newO_y M_PI;
#define right_newO_z 0;


double initP_x,initP_y,initP_z;
double initO_x,initO_y,initO_z;
Eigen::Vector3d tool_vec_g;



//using mutex locking controller ptr while it is switching.
std::mutex mutex_act, mutex_force, mutex_ft, mutex_schunkjs_app;

ToolNameT tn;

bool StopFlag;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_f_filter;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_t_filter;

//define the robot control mode: normal mode vs psudo_gravity_compensation mode
RobotModeT right_rmt;
//define the hinged tool's axis direction (local frame)
Eigen::Vector3d local_hinged_axis_vec;

//for test control axis of hinge
//desired
Eigen::Matrix3d des_tm;
Eigen::Vector3d des_vec;
//cur
Eigen::Vector3d axis_end_vec;

//get schunk joint angle
std::vector<double> schunkjs;

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}


void sdh_moveto_cb(boost::shared_ptr<std::string> data){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = right_newP_x;
    p(1) = right_newP_y;
    p(2) = right_newP_z;

    Eigen::Vector3d desired_euler;
    Eigen::Matrix3d Ident;
    desired_euler.setZero();
    Ident.setIdentity();
    desired_euler(0) = 0;
    desired_euler(1) = -1*M_PI;
    desired_euler(2) = 0.5*M_PI;

    o = euler2axisangle(desired_euler,Ident);

    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = GLOBAL;
    right_task_vec.back()->set_desired_p_eigen(p);
    right_task_vec.back()->set_desired_o_ax(o);
    mutex_act.unlock();
    std::cout<<"kuka sdh self movement and move to new pose"<<std::endl;

}

void sdhaxisvec_moveto_cb(boost::shared_ptr<std::string> data){
	//an arbitary direction
    //des_vec = des_tm.col(1);
    //an specified directin
    des_vec(0) = 0;
    des_vec(1) = 1.0;
    des_vec(2) = 0;
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_ROTATEFOLLOW));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->set_desired_axis_dir(des_vec);
    right_task_vec.back()->set_primitive(ROTATE_TOWARDS_AXIS);
    mutex_act.unlock();
    std::cout<<"kuka sdh self movement and move to desired direction"<<std::endl;
}

void sdhmaintainF_cb(boost::shared_ptr<std::string> data){
	Eigen::Vector3d des_surf_nv,cur_axis;
    des_surf_nv.setZero();
    cur_axis.setZero();
    cur_axis(1) = 1.0;
    des_surf_nv(2) = 1.0;
	//generate contact frame
	Eigen::Matrix3d cf_tm;
	cf_tm.setZero();
	cf_tm.col(0) = des_surf_nv;
	cf_tm.col(2) = cur_axis;
	cf_tm.col(1) = cur_axis.cross(des_surf_nv);
    mutex_force.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->set_desired_axis_dir(des_vec);
    right_task_vec.back()->set_primitive(NOPRIM);
    
    
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.forcet = F_MAINTAIN;
    right_ac_vec.push_back(new ForceServoController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new ForceServoTask(right_taskname.forcet));
    right_task_vec.back()->mt = FORCE;
    right_task_vec.back()->mft = GLOBAL;
    right_task_vec.back()->set_desired_cf_kuka(des_force);
    right_task_vec.back()->set_desired_init_surf_nv(des_surf_nv);
    right_task_vec.back()->set_init_contact_frame(cf_tm);
    
    mutex_force.unlock();
    std::cout<<"maintain the contact force using F/T feedback"<<std::endl;
}

void sdhslideX_cb(boost::shared_ptr<std::string> data){
	Eigen::Vector3d des_surf_nv,cur_axis;
    des_surf_nv.setZero();
    cur_axis.setZero();
    cur_axis(1) = 1.0;
    des_surf_nv(2) = 1.0;
	//generate contact frame
	Eigen::Matrix3d cf_tm;
	cf_tm.setZero();
	cf_tm.col(0) = des_surf_nv;
	cf_tm.col(2) = cur_axis;
	cf_tm.col(1) = cur_axis.cross(des_surf_nv);
	mutex_force.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_LINEFOLLOW));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->set_desired_axis_dir(des_vec);
    right_task_vec.back()->set_init_contact_frame(cf_tm);
    right_task_vec.back()->set_primitive(SLIDINGY);
    

    right_taskname.forcet = F_MAINTAIN;
    right_ac_vec.push_back(new ForceServoController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new ForceServoTask(right_taskname.forcet));
    right_task_vec.back()->mt = FORCE;
    right_task_vec.back()->mft = GLOBAL;
    right_task_vec.back()->set_desired_cf_kuka(des_force);
    right_task_vec.back()->set_init_contact_frame(cf_tm);
    right_task_vec.back()->set_desired_init_surf_nv(des_surf_nv);
    
    mutex_force.unlock();
    std::cout<<"tool's hybrid control X"<<std::endl;
	}
void sdhslideY_cb(boost::shared_ptr<std::string> data){
	Eigen::Vector3d des_surf_nv,cur_axis;
    des_surf_nv.setZero();
    cur_axis.setZero();
    cur_axis(1) = 1.0;
    des_surf_nv(2) = 1.0;
	//generate contact frame
	Eigen::Matrix3d cf_tm;
	cf_tm.setZero();
	cf_tm.col(0) = des_surf_nv;
	cf_tm.col(2) = cur_axis;
	cf_tm.col(1) = cur_axis.cross(des_surf_nv);
	mutex_force.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_LINEFOLLOW));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->set_desired_axis_dir(des_vec);
    right_task_vec.back()->set_init_contact_frame(cf_tm);
    right_task_vec.back()->set_primitive(SLIDINGZ);
    

    right_taskname.forcet = F_MAINTAIN;
    right_ac_vec.push_back(new ForceServoController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new ForceServoTask(right_taskname.forcet));
    right_task_vec.back()->mt = FORCE;
    right_task_vec.back()->mft = GLOBAL;
    right_task_vec.back()->set_desired_cf_kuka(des_force);
    right_task_vec.back()->set_init_contact_frame(cf_tm);
    right_task_vec.back()->set_desired_init_surf_nv(des_surf_nv);
    
    mutex_force.unlock();
    std::cout<<"tool's hybrid control Y"<<std::endl;
	}

void sdhfoldtool_cb(boost::shared_ptr<std::string> data){
	Eigen::Vector3d des_surf_nv,cur_axis;
    des_surf_nv.setZero();
    cur_axis.setZero();
    cur_axis(1) = 1.0;
    des_surf_nv(2) = 1.0;
	//generate contact frame
	Eigen::Matrix3d cf_tm;
	cf_tm.setZero();
	cf_tm.col(0) = des_surf_nv;
	cf_tm.col(2) = cur_axis;
	cf_tm.col(1) = cur_axis.cross(des_surf_nv);
	mutex_force.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_ROTATEFOLLOW));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = LOCAL;
    right_task_vec.back()->set_desired_axis_dir(des_vec);
    right_task_vec.back()->set_primitive(ROTATE_AROUND_AXIS);
    right_task_vec.back()->set_init_contact_frame(cf_tm);
    

    right_taskname.forcet = F_MAINTAIN;
    right_ac_vec.push_back(new ForceServoController(*right_pm));
    right_ac_vec.back()->set_init_TM(right_rs->robot_orien["eef"]);
    right_task_vec.push_back(new ForceServoTask(right_taskname.forcet));
    right_task_vec.back()->mt = FORCE;
    right_task_vec.back()->mft = GLOBAL;
    right_task_vec.back()->set_desired_cf_kuka(des_force);
    right_task_vec.back()->set_desired_init_surf_nv(des_surf_nv);
    
    mutex_force.unlock();
    std::cout<<"fold the tool rotated by the axis"<<std::endl;
	}



void sdh_grav_comp_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to psudo_gravity_compasenstation control"<<std::endl;
    right_rmt = PsudoGravityCompensation;
}

void sdh_normal_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to normal control"<<std::endl;
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
    right_rmt = NormalMode;
}


void gamaftcalib_cb(boost::shared_ptr<std::string> data){

}


void brake_cb(boost::shared_ptr<std::string> data){
    com_okc_right->start_brake();
}

void nobrake_cb(boost::shared_ptr<std::string> data){
    com_okc_right->release_brake();
}


int counter_t = 0;
// receive FT Sensor data
void
recvFT(const geometry_msgs::WrenchStampedConstPtr& msg){
    mutex_force.lock();
//    std::cout <<"Fx"<<"\t"<<"Fy"<<"\t"<<"Fz"<<"\t"<<"Tx"<<"\t"<<"Ty"<<"\t"<<"Tz"<<std::endl;
//     if (counter >50){
//    std::cout << msg->wrench.force.x<<","<<msg->wrench.force.y<<","<<msg->wrench.force.z<<","<<msg->wrench.torque.x<<","<<msg->wrench.torque.y<<","<<msg->wrench.torque.z<<std::endl;
//         counter = 0;
//     }
//    else{
//        counter ++;
// }
    Eigen::Vector3d grav_tmp;
    grav_tmp.setZero();
    grav_tmp(2) = Grav_Schunk_Acc+tool_grav;
    ft_gama->raw_ft_f(0) = msg->wrench.force.x;
    ft_gama->raw_ft_f(1) = msg->wrench.force.y;
    ft_gama->raw_ft_f(2) = msg->wrench.force.z-Grav_Schunk_Acc;
    ft_gama->raw_ft_t(0) = msg->wrench.torque.x;
    ft_gama->raw_ft_t(1) = msg->wrench.torque.y;
    ft_gama->raw_ft_t(2) = msg->wrench.torque.z;
    //get rid of tool/pokingstick gravity--calib
    ft_gama->calib_ft_f = right_rs->robot_orien["eef"] * ft_gama->raw_ft_f + grav_tmp;
    ft_gama->calib_ft_t = ft_gama->raw_ft_t;
    //use smooth filter to get rid of noise.
    ft.head(3) = ft_gama->filtered_gama_f = gama_f_filter->push(ft_gama->calib_ft_f);
    //get rid of noise of no contacting
    if(ft_gama->filtered_gama_f.norm() <1.5)
	    ft.head(3).setZero();
    ft.tail(3) = ft_gama->filtered_gama_t = gama_t_filter->push(ft_gama->calib_ft_t);
    
    //counter_t ++;
    //if(counter_t%50==0){
        //counter_t = 0;
        //std::cout<<"estimated contact force: "<<ft_gama->filtered_gama_f(0)<<","<<ft_gama->filtered_gama_f(1)<<","<<ft_gama->filtered_gama_f(2)<<std::endl;
//}
    
    mutex_force.unlock();
}


void schunkJSviarsb(){
    mutex_schunkjs_app.lock();
    com_rsb->schunkjs_receive(schunkjs);
//     std::cout<<"schunk joint angle are ";
//     for(int i = 0; i < schunkjs.size(); i++){
//         std::cout<<schunkjs.at(i)<<",";
//     }
//     std::cout<<std::endl;
    mutex_schunkjs_app.unlock();
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
        js_la.position[i]=right_rs->JntPosition_mea[i];
        js_ra.position[i]=0;
    }
    
    mutex_schunkjs_app.lock();
    for(unsigned int i = 0; i < schunkjs.size(); i++){
        js_schunk.position[i] = schunkjs.at(i)*M_PI/180.0;
    }
    mutex_schunkjs_app.unlock();

    js_la.header.stamp=ros::Time::now();
    js_ra.header.stamp=ros::Time::now();
    js_schunk.header.stamp=ros::Time::now();

    //publish the gamma force vector
    if(gamma_force_marker_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker gamma_force_marker;
        // Set the color -- be sure to set alpha to something non-zero!
        gamma_force_marker.color.r = 1.0f;
        gamma_force_marker.color.g = 0.0f;
        gamma_force_marker.color.b = 0.0f;
        gamma_force_marker.color.a = 1.0;

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
        Eigen::Vector3d gama_f_normalized;
        gama_f_normalized = ft_gama->filtered_gama_f.normalized();

        gamma_force_marker.points.resize(2);
        gamma_force_marker.points[0].x = right_rs->robot_position["eef"](0);
        gamma_force_marker.points[0].y = right_rs->robot_position["eef"](1);
        gamma_force_marker.points[0].z = right_rs->robot_position["eef"](2);

        gamma_force_marker.points[1].x = right_rs->robot_position["eef"](0) + 0.1 * gama_f_normalized(0);
        gamma_force_marker.points[1].y = right_rs->robot_position["eef"](1) + 0.1 * gama_f_normalized(1);
        gamma_force_marker.points[1].z = right_rs->robot_position["eef"](2) + 0.1 * gama_f_normalized(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        gamma_force_marker.scale.x = .01;
        gamma_force_marker.scale.y = .01;
        gamma_force_marker.scale.z = .01;

        gamma_force_marker.lifetime = ros::Duration();
	    gamma_force_marker_pub.publish(gamma_force_marker);
    }
    
    //publish the director of the rotation axis
    if(hingedtool_axis_marker_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker hingedtool_axis_marker;
        // Set the color -- be sure to set alpha to something non-zero!
        hingedtool_axis_marker.color.r = 1.0f;
        hingedtool_axis_marker.color.g = 0.0f;
        hingedtool_axis_marker.color.b = 0.0f;
        hingedtool_axis_marker.color.a = 1.0;

        hingedtool_axis_marker.header.frame_id = "frame";
        hingedtool_axis_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        hingedtool_axis_marker.ns = "KukaRos";
        hingedtool_axis_marker.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        hingedtool_axis_marker.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        hingedtool_axis_marker.action = visualization_msgs::Marker::ADD;

        
        hingedtool_axis_marker.points.resize(2);
        hingedtool_axis_marker.points[0].x = right_rs->robot_position["eef"](0);
        hingedtool_axis_marker.points[0].y = right_rs->robot_position["eef"](1);
        hingedtool_axis_marker.points[0].z = right_rs->robot_position["eef"](2);

        
        axis_end_vec = right_rs->robot_position["eef"] + 0.1 * (right_rs->robot_orien["eef"] * local_hinged_axis_vec);
        hingedtool_axis_marker.points[1].x = axis_end_vec(0);
        hingedtool_axis_marker.points[1].y = axis_end_vec(1);
        hingedtool_axis_marker.points[1].z = axis_end_vec(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        hingedtool_axis_marker.scale.x = .01;
        hingedtool_axis_marker.scale.y = .01;
        hingedtool_axis_marker.scale.z = .01;

        hingedtool_axis_marker.lifetime = ros::Duration();
        hingedtool_axis_marker_pub.publish(hingedtool_axis_marker);
    }
    
    if(desired_po_test_pub.getNumSubscribers() >= 1){
		visualization_msgs::Marker desired_po_marker;
        // Set the color -- be sure to set alpha to something non-zero!
        desired_po_marker.color.r = 0.0f;
        desired_po_marker.color.g = 1.0f;
        desired_po_marker.color.b = 0.0f;
        desired_po_marker.color.a = 1.0;

        desired_po_marker.header.frame_id = "frame";
        desired_po_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        desired_po_marker.ns = "KukaRos";
        desired_po_marker.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        desired_po_marker.type = visualization_msgs::Marker::CUBE;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        desired_po_marker.action = visualization_msgs::Marker::ADD;

        
        desired_po_marker.pose.position.x = right_newP_x;
        desired_po_marker.pose.position.y = right_newP_y;
        desired_po_marker.pose.position.z = right_newP_z;
        
        desired_po_marker.pose.orientation.x = 0.0;
        desired_po_marker.pose.orientation.y = 0.0;
        desired_po_marker.pose.orientation.z = 0.0;
        desired_po_marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        desired_po_marker.scale.x = .01;
        desired_po_marker.scale.y = .01;
        desired_po_marker.scale.z = .01;

        desired_po_marker.lifetime = ros::Duration();
        desired_po_test_pub.publish(desired_po_marker);
		}
	if(desired_vec_test_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker vec_test_marker;
        // Set the color -- be sure to set alpha to something non-zero!
        vec_test_marker.color.r = 0.0f;
        vec_test_marker.color.g = 0.0f;
        vec_test_marker.color.b = 1.0f;
        vec_test_marker.color.a = 1.0;

        vec_test_marker.header.frame_id = "frame";
        vec_test_marker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        vec_test_marker.ns = "KukaRos";
        vec_test_marker.id = 0;
        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        vec_test_marker.type = visualization_msgs::Marker::ARROW;
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        vec_test_marker.action = visualization_msgs::Marker::ADD;

        
        vec_test_marker.points.resize(2);
        vec_test_marker.points[0].x = right_rs->robot_position["eef"](0);
        vec_test_marker.points[0].y = right_rs->robot_position["eef"](1);
        vec_test_marker.points[0].z = right_rs->robot_position["eef"](2);

       
        vec_test_marker.points[1].x = right_rs->robot_position["eef"](0) + 0.1 * des_vec(0);
        vec_test_marker.points[1].y = right_rs->robot_position["eef"](1) + 0.1 * des_vec(1);
        vec_test_marker.points[1].z = right_rs->robot_position["eef"](2) + 0.1 * des_vec(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        vec_test_marker.scale.x = .01;
        vec_test_marker.scale.y = .01;
        vec_test_marker.scale.z = .01;

        vec_test_marker.lifetime = ros::Duration();
        desired_vec_test_pub.publish(vec_test_marker);
    }
    
    reba_tactile_msgs::cp_info vec3_msg;
    vec3_msg.header.frame_id = vec3_msg.local_ct_position.header.frame_id = vec3_msg.global_ct_position.header.frame_id = vec3_msg.global_ct_nv.header.frame_id = vec3_msg.ct_pressure.header.frame_id = js_la.header.frame_id = "frame";
    vec3_msg.header.stamp = vec3_msg.local_ct_position.header.stamp = vec3_msg.global_ct_position.header.stamp = vec3_msg.global_ct_nv.header.stamp = vec3_msg.ct_pressure.header.stamp = js_la.header.stamp = ros::Time::now();
    
    //also publish the contact position via ROS (foo value)
    vec3_msg.global_ct_position.vector.x = 0;
    vec3_msg.global_ct_position.vector.y = 0;
    vec3_msg.global_ct_position.vector.z = 0;
    vec3_msg.local_ct_position.vector.x = 0;
    vec3_msg.local_ct_position.vector.y = 0;
    vec3_msg.local_ct_position.vector.z = 0;
    vec3_msg.global_ct_nv.vector.x = 0;
    vec3_msg.global_ct_nv.vector.y = 0;
    vec3_msg.global_ct_nv.vector.z = 1;
    vec3_msg.ct_pressure.vector.x = 0;
    vec3_msg.ct_pressure.vector.y = 0;
    vec3_msg.ct_pressure.vector.z = 0;
    p_ct_pub.publish(vec3_msg);
    
    // send a joint_state
    jsPub_la.publish(js_la);
    jsPub_ra.publish(js_ra);
    jsPub_schunk.publish(js_schunk);
    
    //publish the robot end-effector position
    geometry_msgs::Pose reba_robot_end_eff_val;
    
    reba_robot_end_eff_val.position.x = right_rs->robot_position["robot_eef"](0);
    reba_robot_end_eff_val.position.y = right_rs->robot_position["robot_eef"](1);
    reba_robot_end_eff_val.position.z = right_rs->robot_position["robot_eef"](2);
    Eigen::Quaterniond tmp_q;
    tmp_q = right_rs->robot_orien["robot_eef"];
    reba_robot_end_eff_val.orientation.x = tmp_q.x();
    reba_robot_end_eff_val.orientation.y = tmp_q.y();
    reba_robot_end_eff_val.orientation.z = tmp_q.z();
    reba_robot_end_eff_val.orientation.w = tmp_q.w();   
    reba_robot_end_eff_pub.publish(reba_robot_end_eff_val);
    
    //translated tool's frame
    tf::Matrix3x3 tfR;
    tf::Transform transform;
    tf::matrixEigenToTF (right_rs->robot_orien["eef"], tfR);
    transform.setOrigin( tf::Vector3(right_rs->robot_position["eef"](0), right_rs->robot_position["eef"](1), right_rs->robot_position["eef"](2)) );
    transform.setBasis(tfR);
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frame", "tool_frame"));
    //controlled desired frame only for visulaization
    des_tm = right_rs->robot_orien["eef"] * Eigen::AngleAxisd(1.0/3.0*M_PI, Eigen::Vector3d::UnitZ());
    tf::matrixEigenToTF (des_tm, tfR);
    transform.setOrigin( tf::Vector3(right_rs->robot_position["eef"](0), right_rs->robot_position["eef"](1), right_rs->robot_position["eef"](2)) );
    transform.setBasis(tfR);
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "frame", "desire_frame"));
    
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

    //find the path of config files
    std::string selfpath = get_selfpath();
    //select using normal control mode or psudogravity control mode
    right_rmt = NormalMode;
    //initialize FT sensor ptr
    ft_gama = new gamaFT;
    ft.setZero(6);
    tool_vec_g.setZero();
    axis_end_vec.setZero();
    
    //show toolname
    tn = hingedtool;
    StopFlag = false;
    //initialize the axis vec
    local_hinged_axis_vec.setZero();
    local_hinged_axis_vec(0) = -0.9472;
    local_hinged_axis_vec(1) = 0.0494;
    local_hinged_axis_vec(2) = -0.3166;
    
    des_tm.setZero();
    des_vec.setZero();
    
    //declare the cb function

    boost::function<void(boost::shared_ptr<std::string>)> button_sdh_moveto(sdh_moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_sdhaxisvec_moveto(sdhaxisvec_moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_sdhmaintainF(sdhmaintainF_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_sdhslideX(sdhslideX_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_sdhslideY(sdhslideY_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_sdhfoldtool(sdhfoldtool_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_gamaftcalib(gamaftcalib_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_sdh_grav_comp_ctrl(sdh_grav_comp_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_sdh_normal_ctrl(sdh_normal_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_brake(brake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_nobrake(nobrake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);

    //specify controller configure file in order to load it(right kuka + shadow hand)
    std::string config_filename = selfpath + "/etc/right_arm_param.xml";
    std::cout<<"right arm config file name is: "<<config_filename<<std::endl;
    //load controller parameters
    right_pm = new ParameterManager(config_filename);
    //initialize ptr to right kuka com okc
    com_okc_right = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);

    //connect kuka right
    com_okc_right->connect();

    //initialize the kuka robot and let it stay in the init pose
    kuka_right_arm = new KukaLwr(kuka_right,*com_okc_right,tn);

    //initialize the robot state
    right_rs = new RobotState(kuka_right_arm);

    //get the initialize state of kuka right
    kuka_right_arm->get_joint_position_act();
    kuka_right_arm->update_robot_state();
    right_rs->updated(kuka_right_arm);
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = GLOBAL;

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
    rdtschunkjs = SchunkJS;
    com_rsb->add_msg(rdtschunkjs);
    gama_f_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    gama_t_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));

    //register cb function
    com_rsb->register_external("/foo/sdhmoveto",button_sdh_moveto);
    com_rsb->register_external("/foo/sdhaxisvecmoveto",button_sdhaxisvec_moveto);
    com_rsb->register_external("/foo/sdhmaintainF",button_sdhmaintainF);
    com_rsb->register_external("/foo/sdhslideX",button_sdhslideX);
    com_rsb->register_external("/foo/sdhslideY",button_sdhslideY);
    com_rsb->register_external("/foo/sdhfoldtool",button_sdhfoldtool);
    com_rsb->register_external("/foo/gamaftcalib",button_gamaftcalib);
    com_rsb->register_external("/foo/sdh_grav_comp_ctrl",button_sdh_grav_comp_ctrl);
    com_rsb->register_external("/foo/sdh_normal_ctrl",button_sdh_normal_ctrl);
    com_rsb->register_external("/foo/closeprog",button_closeprog);

#ifdef HAVE_ROS
    std::string left_kuka_arm_name="la";
    std::string right_kuka_arm_name="ra";
    std::string left_schunk_hand_name ="lh";
    js_la.name.push_back(left_kuka_arm_name+"_arm_0_joint");
    js_la.name.push_back(left_kuka_arm_name+"_arm_1_joint");
    js_la.name.push_back(left_kuka_arm_name+"_arm_2_joint");
    js_la.name.push_back(left_kuka_arm_name+"_arm_3_joint");
    js_la.name.push_back(left_kuka_arm_name+"_arm_4_joint");
    js_la.name.push_back(left_kuka_arm_name+"_arm_5_joint");
    js_la.name.push_back(left_kuka_arm_name+"_arm_6_joint");
    js_ra.name.push_back(right_kuka_arm_name+"_arm_0_joint");
    js_ra.name.push_back(right_kuka_arm_name+"_arm_1_joint");
    js_ra.name.push_back(right_kuka_arm_name+"_arm_2_joint");
    js_ra.name.push_back(right_kuka_arm_name+"_arm_3_joint");
    js_ra.name.push_back(right_kuka_arm_name+"_arm_4_joint");
    js_ra.name.push_back(right_kuka_arm_name+"_arm_5_joint");
    js_ra.name.push_back(right_kuka_arm_name+"_arm_6_joint");
    
    //for schunk
    js_schunk.name.push_back(left_schunk_hand_name+"_sdh_knuckle_joint");
    js_schunk.name.push_back(left_schunk_hand_name+"_sdh_finger_22_joint");
    js_schunk.name.push_back(left_schunk_hand_name+"_sdh_finger_23_joint");
    js_schunk.name.push_back(left_schunk_hand_name+"_sdh_thumb_2_joint");
    js_schunk.name.push_back(left_schunk_hand_name+"_sdh_thumb_3_joint");
    js_schunk.name.push_back(left_schunk_hand_name+"_sdh_finger_12_joint");
    js_schunk.name.push_back(left_schunk_hand_name+"_sdh_finger_13_joint");

    js_la.position.resize(7);
    js_la.velocity.resize(7);
    js_la.effort.resize(7);

    js_ra.position.resize(7);
    js_ra.velocity.resize(7);
    js_ra.effort.resize(7);
    
    js_schunk.position.resize(7);
    js_schunk.velocity.resize(7);
    js_schunk.effort.resize(7);

    js_la.header.frame_id="frame_la";
    js_ra.header.frame_id="frame_ra";
    js_ra.header.frame_id="frame_lh";

    gamma_force_marker_pub = nh->advertise<visualization_msgs::Marker>("gamma_force_marker", 2);
    hingedtool_axis_marker_pub = nh->advertise<visualization_msgs::Marker>("hingedtool_axis_marker", 2);

    jsPub_la = nh->advertise<sensor_msgs::JointState> ("/la/joint_states", 2);
    jsPub_ra = nh->advertise<sensor_msgs::JointState> ("/ra/joint_states", 2);
    jsPub_schunk = nh->advertise<sensor_msgs::JointState> ("/lh/joint_states", 2);
    ros::spinOnce();

    br = new tf::TransformBroadcaster();

    std::cout<<"ros init finished"<<std::endl;
#endif
}

void get_ft_vis_info(){
    //this function is only for the computation of force vector visualization for gamma sensor.
    //located in the contact point detected by mid sensor
    
}


void run_rightarm(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc_right->data_available == true)&&(com_okc_right->controller_update == false)){
//        kuka_right_arm->update_robot_stiffness();
        kuka_right_arm->get_joint_position_act();
        kuka_right_arm->update_robot_state();
        right_rs->updated(kuka_right_arm);
        //using all kinds of controllers to update the reference
        for(unsigned int i = 0; i < right_ac_vec.size();i++){
            if(right_task_vec[i]->mt == JOINTS)
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i],(right_rs->robot_orien["eef"] * local_hinged_axis_vec).normalized(),right_rs);
            if(right_task_vec[i]->mt == FORCE){
                mutex_force.lock();
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i],ft,right_rs);
                mutex_force.unlock();
            }
        }
        //update with act_vec
        right_ac_vec[0]->llv.setZero();
        right_ac_vec[0]->lov.setZero();

        //use CBF to compute the desired joint angle rate
        kuka_right_arm->update_cbf_controller();
        kuka_right_arm->set_joint_command(right_rmt);
        com_okc_right->controller_update = true;
        com_okc_right->data_available = false;
    }
}


int main(int argc, char* argv[])
{
    #ifdef HAVE_ROS
        ros::init(argc, argv, "HingedToolManip",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
        ros::Subscriber	GammaFtSub;
        std::cout<<"Connect to ATI FT sensor"<<std::endl;
        GammaFtSub=nh->subscribe("/ft_sensor/wrench", 1, // buffer size
                                &recvFT);
    reba_robot_end_eff_pub = nh->advertise<geometry_msgs::Pose> ("reba_robot_end_eff", 1);
    //this is a foo pub in order to active the reba_estrot_node
    p_ct_pub = nh->advertise<reba_tactile_msgs::cp_info>("position_ct", 1);
    desired_po_test_pub = nh->advertise<visualization_msgs::Marker>("desired_po_test", 1);
    desired_vec_test_pub = nh->advertise<visualization_msgs::Marker>("desired_vec_test", 1);

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
    Timer thrd_rightkuka_ctrl(run_rightarm);
    thrd_rightkuka_ctrl.setSingleShot(false);
    thrd_rightkuka_ctrl.setInterval(Timer::Interval(1));
    thrd_rightkuka_ctrl.start(true);
    
    //start Schunk hand read thread
    Timer thrd_schunk_read(schunkJSviarsb);
    thrd_schunk_read.setSingleShot(false);
    thrd_schunk_read.setInterval(Timer::Interval(100));
    thrd_schunk_read.start(true);

    #ifdef HAVE_ROS
    //start ros publisher thread
    Timer thrd_rospublisher(ros_publisher);
    thrd_rospublisher.setSingleShot(false);
    thrd_rospublisher.setInterval(Timer::Interval(4));
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
    thrd_rightkuka_ctrl.stop();
    #ifdef HAVE_ROS
    thrd_rospublisher.stop();
    #endif
    return 0;
}




