

/*
 ============================================================================
 Name        : HapticExploring.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : use right arm + two poking stick exploring the object, then get the pointclouds of objects for Isy project
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
#include "RebaType.h"
#include "msgcontenttype.h"


#ifdef HAVE_ROS
// ROS objects
tf::TransformBroadcaster *br;
sensor_msgs::JointState js;
ros::Publisher jsPub;
ros::NodeHandle *nh;
ros::Publisher act_leftmarker_pub,act_rightmarker_pub,act_leftmarker_nv_pub,act_rightmarker_nv_pub;
#endif
std::ofstream Tposition;

//ptr for openkc component
ComOkc *com_okc_left;
//ptr for left kuka robot
Robot *kuka_left_arm;
//ptr for left kuka controller vector
std::vector<ActController *> left_ac_vec;
//ptr for left kuka task vector
std::vector<Task *> left_task_vec;
//defined left arm task name
TaskNameT left_taskname;


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
RsbDataType rdtleftkuka;

//left kuka controller parameters
ParameterManager* left_pm;
//right kuka controller parameters
ParameterManager* right_pm;
//left kuka internal model
RobotState *left_rs;
//right kuka internal model
RobotState *right_rs;

double initP_x,initP_y,initP_z;
double initO_x,initO_y,initO_z;
Eigen::VectorXd ft;

//using mutex locking controller ptr while it is switching.
std::mutex mutex_act, mutex_force,mutex_lefttac,mutex_righttac;
//initialize fingertip instance
FingertipTac *Leftftt,*Rightftt;
ToolNameT tn;

bool StopFlag;

//define the robot control mode: normal mode vs psudo_gravity_compensation mode
RobotModeT left_rmt,right_rmt;

//estimated contact point position, normal direction
Eigen::Vector3d left_cp_g,right_cp_g;
Eigen::Vector3d left_cendp_g,right_cendp_g;
Eigen::Vector3d left_cnv_g,right_cnv_g;
Eigen::Vector3d left_cp_nv,right_cp_nv;


void tip_startrecord_cb(boost::shared_ptr<std::string> data){
    std::cout<<"startrecord data"<<std::endl;
    Tposition<<left_cp_g(0)<<","<<left_cp_g(1)<<","<<left_cp_g(2)<<","<<\
               left_cp_nv(0)<<","<<left_cp_nv(1)<<","<<left_cp_nv(2)<<","\
              <<right_cp_g(0)<<","<<right_cp_g(1)<<","<<right_cp_g(2)<<","<<\
                             right_cp_nv(0)<<","<<right_cp_nv(1)<<","<<right_cp_nv(2)<<std::endl;
}
void tip_continuerecord_cb(boost::shared_ptr<std::string> data){
    std::cout<<"continuerecord data"<<std::endl;
}
void tip_stoprecord_cb(boost::shared_ptr<std::string> data){
    std::cout<<"stoprecord data"<<std::endl;
}

#ifdef HAVE_ROS
// receive Tactile data from UBI Fingertips
void
recvLeftTipTactile(const sr_robot_msgs::UBI0AllConstPtr& msg){
    // for first sensor each taxel
    //Todo: clear data and all estimated value;
    mutex_lefttac.lock();
    double press_val;
    Eigen::Vector3d pos_val,nv_val;
//    std::cout <<"0"<<"\t"<<"1"<<"\t"<<"2"<<"\t"<<"3"<<"\t"<<"4"<<"\t"<<"5"<<"\t"<<"6"<<"\t"<<"7"<<"\t"<<"8"<<"\t"<<"9"<<"\t"<<"10"<<"\t"<<"11"<<"\t"<<std::endl;
//    std::cout << std::fixed;
    ROS_ASSERT(Leftftt->data.fingertip_tac_pressure.size() == msg->tactiles[0].distal.size());
    for(size_t j = 0; j < msg->tactiles[0].distal.size(); ++j) {
        press_val= 1.0-(msg->tactiles[0].distal[j]/1023.0);
        if(press_val < MID_THRESHOLD) press_val = 0.0;
        Leftftt->data.fingertip_tac_pressure[j] = press_val;
//                std::cout<<std::setprecision(5)<<press_val<<"\t";
    }
//    std::cout<<std::endl;
    mutex_lefttac.unlock();
}

void
recvRightTipTactile(const sr_robot_msgs::UBI0AllConstPtr& msg){
    // for first sensor each taxel
    //Todo: clear data and all estimated value;
    mutex_righttac.lock();
    double press_val;
    Eigen::Vector3d pos_val,nv_val;
//    std::cout <<"0"<<"\t"<<"1"<<"\t"<<"2"<<"\t"<<"3"<<"\t"<<"4"<<"\t"<<"5"<<"\t"<<"6"<<"\t"<<"7"<<"\t"<<"8"<<"\t"<<"9"<<"\t"<<"10"<<"\t"<<"11"<<"\t"<<std::endl;
//    std::cout << std::fixed;
    ROS_ASSERT(Rightftt->data.fingertip_tac_pressure.size() == msg->tactiles[0].distal.size());
    for(size_t j = 0; j < msg->tactiles[0].distal.size(); ++j) {
        press_val= 1.0-(msg->tactiles[0].distal[j]/1023.0);
        if(press_val < MID_THRESHOLD) press_val = 0.0;
        Rightftt->data.fingertip_tac_pressure[j] = press_val;
//        std::cout<<std::setprecision(5)<<press_val<<"\t";
    }
//    std::cout<<std::endl;
    mutex_righttac.unlock();
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
        js.position[i]=right_rs->JntPosition_mea[i];
        js.position[i+7]=left_rs->JntPosition_mea[i];
    }

    js.header.stamp=ros::Time::now();
    bool leftcontact_f = false;
    mutex_lefttac.lock();
    leftcontact_f = Leftftt->isContact(Leftftt->data);
    mutex_lefttac.unlock();
    bool rightcontact_f = false;
    mutex_righttac.lock();
    rightcontact_f = Rightftt->isContact(Rightftt->data);
    mutex_righttac.unlock();

   //publish the actived position
    if ((act_leftmarker_pub.getNumSubscribers() >= 1)){
        visualization_msgs::Marker act_marker;
        mutex_lefttac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        act_marker.color.r = Leftftt->pressure;
        act_marker.color.g = 0.0f;
        act_marker.color.b = 0.0f;
        if(leftcontact_f == true)
            act_marker.color.a = 1.0;
        else
            act_marker.color.a = 0;
        mutex_lefttac.unlock();

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
        act_marker.pose.position.x = left_cp_g(0);
        act_marker.pose.position.y = left_cp_g(1);
        act_marker.pose.position.z = left_cp_g(2);
        act_marker.pose.orientation.x = 0.0;
        act_marker.pose.orientation.y = 0.0;
        act_marker.pose.orientation.z = 0.0;
        act_marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        act_marker.scale.x = .002;
        act_marker.scale.y = .002;
        act_marker.scale.z = .002;

        act_marker.lifetime = ros::Duration();
        act_leftmarker_pub.publish(act_marker);
    }

    //publish the actived position
     if ((act_rightmarker_pub.getNumSubscribers() >= 1)){
         visualization_msgs::Marker act_marker;
         mutex_righttac.lock();
         // Set the color -- be sure to set alpha to something non-zero!
         act_marker.color.r = Rightftt->pressure;
         act_marker.color.g = 0.0f;
         act_marker.color.b = 0.0f;
         if(rightcontact_f == true)
             act_marker.color.a = 1.0;
         else
             act_marker.color.a = 0;
         mutex_righttac.unlock();

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
         act_marker.pose.position.x = right_cp_g(0);
         act_marker.pose.position.y = right_cp_g(1);
         act_marker.pose.position.z = right_cp_g(2);
         act_marker.pose.orientation.x = 0.0;
         act_marker.pose.orientation.y = 0.0;
         act_marker.pose.orientation.z = 0.0;
         act_marker.pose.orientation.w = 1.0;

         // Set the scale of the marker -- 1x1x1 here means 1m on a side
         act_marker.scale.x = .002;
         act_marker.scale.y = .002;
         act_marker.scale.z = .002;

         act_marker.lifetime = ros::Duration();
         act_rightmarker_pub.publish(act_marker);
     }

    //publish the actived normal vector
    if(act_leftmarker_nv_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker act_marker_nv;
        mutex_lefttac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        act_marker_nv.color.r = 1.0f;
        act_marker_nv.color.g = 0.0f;
        act_marker_nv.color.b = 0.0f;
        if(leftcontact_f == true)
            act_marker_nv.color.a = 1.0;
        else
            act_marker_nv.color.a = 0;
        mutex_lefttac.unlock();

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
        act_marker_nv.points[0].x = left_cp_g(0);
        act_marker_nv.points[0].y = left_cp_g(1);
        act_marker_nv.points[0].z = left_cp_g(2);

        act_marker_nv.points[1].x = left_cendp_g(0);
        act_marker_nv.points[1].y = left_cendp_g(1);
        act_marker_nv.points[1].z = left_cendp_g(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        act_marker_nv.scale.x = .001;
        act_marker_nv.scale.y = .001;
        act_marker_nv.scale.z = .001;

        act_marker_nv.lifetime = ros::Duration();
        act_leftmarker_nv_pub.publish(act_marker_nv);
    }

    //publish the actived normal vector
    if(act_rightmarker_nv_pub.getNumSubscribers() >= 1){
        visualization_msgs::Marker act_marker_nv;
        mutex_righttac.lock();
        // Set the color -- be sure to set alpha to something non-zero!
        act_marker_nv.color.r = 1.0f;
        act_marker_nv.color.g = 0.0f;
        act_marker_nv.color.b = 0.0f;
        if(rightcontact_f == true)
            act_marker_nv.color.a = 1.0;
        else
            act_marker_nv.color.a = 0;
        mutex_righttac.unlock();

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
        act_marker_nv.points[0].x = right_cp_g(0);
        act_marker_nv.points[0].y = right_cp_g(1);
        act_marker_nv.points[0].z = right_cp_g(2);

        act_marker_nv.points[1].x = right_cendp_g(0);
        act_marker_nv.points[1].y = right_cendp_g(1);
        act_marker_nv.points[1].z = right_cendp_g(2);

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        act_marker_nv.scale.x = .001;
        act_marker_nv.scale.y = .001;
        act_marker_nv.scale.z = .001;

        act_marker_nv.lifetime = ros::Duration();
        act_rightmarker_nv_pub.publish(act_marker_nv);
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
    //find the path of config files
    std::string selfpath = get_selfpath();
    //select using normal control mode or psudogravity control mode
    left_rmt = NormalMode;
    right_rmt = NormalMode;
    //initialize the fingertip sensor ptr
    Leftftt = new FingertipTac(12);
    Rightftt = new FingertipTac(12);
    //show toolname
    tn = teensy_finger;
    StopFlag = false;
    ft.setZero(6);
    left_cp_nv.setZero();
    right_cp_nv.setZero();
    //declare the cb function
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_startrecord(tip_startrecord_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_continuerecord(tip_continuerecord_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tip_stoprecord(tip_stoprecord_cb);

    //specify controller configure file in order to load it(left kuka + mid fingertip)
    std::string config_filename = selfpath + "/etc/left_arm_mid_param.xml";
    std::cout<<"left arm config file name is: "<<config_filename<<std::endl;
    if(is_file_exist(config_filename.c_str()) == false){
        config_filename = "left_arm_mid_param.xml";
        if(is_file_exist(config_filename.c_str()) == false){
            std::cout<<"not find the tactile servo controller configure file"<<std::endl;
            exit(0);
        }
    }
    //initialize ptr to left kuka com okc
    com_okc_left = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    //load controller parameters
    left_pm = new ParameterManager(config_filename);


    //specify controller configure file in order to load it(right kuka + shadow hand)
    config_filename = selfpath + "/etc/right_arm_param.xml";
    std::cout<<"right arm config file name is: "<<config_filename<<std::endl;
    //load controller parameters
    right_pm = new ParameterManager(config_filename);
    //initialize ptr to right kuka com okc
    com_okc_right = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);

    //connect kuka left/right
    com_okc_left->connect();
    com_okc_right->connect();

    //initialize the kuka robot and let it stay in the init pose
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc_left,tn);
    kuka_right_arm = new KukaLwr(kuka_right,*com_okc_right,tn);

    //initialize the robot state
    left_rs = new RobotState(kuka_left_arm);
    right_rs = new RobotState(kuka_right_arm);

    //get the initialize state of kuka left
    kuka_left_arm->get_joint_position_act();
    kuka_left_arm->update_robot_state();
    left_rs->updated(kuka_left_arm);
    left_ac_vec.push_back(new ProActController(*left_pm));
    left_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    left_task_vec.back()->mt = JOINTS;
    left_task_vec.back()->mft = GLOBAL;
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


    //get the initialize state of kuka right
    kuka_right_arm->get_joint_position_act();
    kuka_right_arm->update_robot_state();
    right_rs->updated(kuka_right_arm);
    right_ac_vec.push_back(new ProActController(*right_pm));
    right_task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));
    right_task_vec.back()->mt = JOINTS;
    right_task_vec.back()->mft = GLOBAL;

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



    kuka_left_arm->setAxisStiffnessDamping(left_ac_vec.back()->pm.stiff_ctrlpara.axis_stiffness, \
                                           left_ac_vec.back()->pm.stiff_ctrlpara.axis_damping);

    kuka_right_arm->setAxisStiffnessDamping(right_ac_vec.back()->pm.stiff_ctrlpara.axis_stiffness, \
                                           right_ac_vec.back()->pm.stiff_ctrlpara.axis_damping);

    com_rsb = new ComRSB();
    rdtleftkuka = LeftKukaEff;
    com_rsb->add_msg(rdtleftkuka);
    left_cp_g.setZero();
    left_cendp_g.setZero();
    left_cnv_g.setZero();
    right_cp_g.setZero();
    right_cendp_g.setZero();
    right_cnv_g.setZero();
    //register cb function
    com_rsb->register_external("/foo/tipstartrecord",button_tip_startrecord);
    com_rsb->register_external("/foo/tipcontinuerecord",button_tip_continuerecord);
    com_rsb->register_external("/foo/tipstoprecord",button_tip_stoprecord);

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

    act_leftmarker_pub = nh->advertise<visualization_msgs::Marker>("left_act_marker", 2);
    act_leftmarker_nv_pub = nh->advertise<visualization_msgs::Marker>("left_act_marker_nv", 2);
    act_rightmarker_pub = nh->advertise<visualization_msgs::Marker>("right_act_marker", 2);
    act_rightmarker_nv_pub = nh->advertise<visualization_msgs::Marker>("right_act_marker_nv", 2);
    jsPub = nh->advertise<sensor_msgs::JointState> ("joint_states", 2);
    ros::spinOnce();
    br = new tf::TransformBroadcaster();
    std::cout<<"ros init finished"<<std::endl;
#endif
}

void get_mid_info(){
    //computingt the contact information from mid sensor--contact position,estimated
    //contact force vector, and line feature estimation
    bool leftcontact_f = false;
    bool rightcontact_f = false;
    Eigen::Vector3d slope;
    Eigen::Vector3d l2g_temp;
    slope.setZero();
    l2g_temp.setZero();
    mutex_lefttac.lock();
    leftcontact_f = Leftftt->isContact(Leftftt->data);
    if(leftcontact_f == true){
        //estimate contact point position, normal vector
        Leftftt->est_ct_info(Leftftt->data);
        local2global(Leftftt->pos/1000,\
                     left_rs->robot_orien["eef"],l2g_temp);
        left_cp_g = left_rs->robot_position["eef"] + l2g_temp;
        left_cp_nv = Leftftt->nv;
        local2global(Leftftt->nv/100,\
                     left_rs->robot_orien["eef"],l2g_temp);
        left_cendp_g = left_cp_g + l2g_temp;
    }
    mutex_lefttac.unlock();

    mutex_righttac.lock();
    rightcontact_f = Rightftt->isContact(Rightftt->data);
    if(rightcontact_f == true){
        //estimate contact point position, normal vector
        Rightftt->est_ct_info(Rightftt->data);
        local2global(Rightftt->pos/1000,\
                     right_rs->robot_orien["eef"],l2g_temp);
        right_cp_g = right_rs->robot_position["eef"] + l2g_temp;
        right_cp_nv = Rightftt->nv;
        local2global(Rightftt->nv/100,\
                     right_rs->robot_orien["eef"],l2g_temp);
        right_cendp_g = right_cp_g + l2g_temp;

    }
    mutex_righttac.unlock();
}


void run_leftarm(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc_left->data_available == true)&&(com_okc_left->controller_update == false)){
//        kuka_left_arm->update_robot_stiffness();
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        left_rs->updated(kuka_left_arm);
        //using mid for control, get contact information
        get_mid_info();
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
                mutex_lefttac.lock();
                left_ac_vec[i]->update_robot_reference(kuka_left_arm,left_task_vec[i],Leftftt);
                mutex_lefttac.unlock();
            }
        }
        //update with act_vec
        left_ac_vec[0]->llv.setZero();
        left_ac_vec[0]->lov.setZero();
        mutex_act.unlock();

        //use CBF to compute the desired joint angle rate
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command(left_rmt);
        com_okc_left->controller_update = true;
        com_okc_left->data_available = false;
    }
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
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i]);
            if(right_task_vec[i]->mt == FORCE){
                mutex_force.lock();
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i],ft,right_rs);
                mutex_force.unlock();
            }
            if(right_task_vec[i]->mt == TACTILE){
                mutex_lefttac.lock();
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i],Rightftt);
                mutex_lefttac.unlock();
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
    //for data recording
    std::string data_f ("/tmp/");
    Tposition.open((data_f+std::string("position.txt")).c_str());
    #ifdef HAVE_ROS
        ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
        ros::Subscriber	lefttacTipSub,righttacTipSub;
        std::cout<<"add call back for the left fingertip"<<std::endl;
        lefttacTipSub=nh->subscribe("/rh/teensy/tactile", 1, // buffer size
                                &recvLeftTipTactile);
        std::cout<<"add call back for the right fingertip"<<std::endl;
        righttacTipSub=nh->subscribe("/lh/teensy/tactile", 1, // buffer size
                                &recvRightTipTactile);

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
    Timer thrd_leftkuka_ctrl(run_leftarm);
    thrd_leftkuka_ctrl.setSingleShot(false);
    thrd_leftkuka_ctrl.setInterval(Timer::Interval(1));
    thrd_leftkuka_ctrl.start(true);

    //start kuka arm control thread
    Timer thrd_rightkuka_ctrl(run_rightarm);
    thrd_rightkuka_ctrl.setSingleShot(false);
    thrd_rightkuka_ctrl.setInterval(Timer::Interval(1));
    thrd_rightkuka_ctrl.start(true);

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
    thrd_leftkuka_ctrl.stop();
    thrd_rightkuka_ctrl.stop();
    #ifdef HAVE_ROS
    thrd_rospublisher.stop();
    #endif
    return 0;
}




