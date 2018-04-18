/*
 ============================================================================
 Name        : FTCalibTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : This is calibration test programe to use the two step
               bias setting procedure to esimate the gravity of tool 
               (hand/tool) from the readout the FT sensor.
               regarding the procedure, please refer to 
               https://projects.cit-ec.uni-bielefeld.de/projects/agni-grasplab/wiki/ATI_FT_sensor
 ============================================================================
 */

/*
 *
*/

//for ROS
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
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
#include "maniptool.h"


//define the gravity of the schunk hand and its accessory
#define Grav_Schunk_Acc 21.63
#define tool_grav 0.15*9.81

#ifdef HAVE_ROS
// ROS objects
ros::NodeHandle *nh;
#endif


bool StopFlag;

std::mutex mutex_force,mutex_act,mutex_tac,mutex_schunkjs_app;
//ptr for FT sensor on the left arm
gamaFT *ft_gama;


ComOkc *com_okc;
Robot *kuka_right_arm;
std::vector<ActController *> right_ac_vec;
std::vector<Task *> right_task_vec;
TaskNameT right_taskname;

ComRSB *com_rsb;
RsbDataType rdtrightkuka;
RsbDataType rdtschunkjs;

ParameterManager* pm;
RobotState *right_rs;

//tactool initialization
ManipTool *mt_ptr;
ToolNameT tn;
int counter;

double initP_x,initP_y,initP_z;
double initO_x,initO_y,initO_z;
//arm_payload gravity(hand+tool-in-hand or midfinger)
Eigen::Vector3d arm_payload_g;
Eigen::VectorXd ft;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_f_filter;
TemporalSmoothingFilter<Eigen::Vector3d>* gama_t_filter;
ros::Publisher jsPub_la, jsPub_ra;
ros::Publisher jsPub_schunk;
tf::TransformBroadcaster *br;
tf::TransformListener *listener;
sensor_msgs::JointState js_la, js_ra,js_schunk;
//get schunk joint angle
std::vector<double> schunkjs;
//define the robot control mode: normal mode vs psudo_gravity_compensation mode
RobotModeT rmt;

std::ofstream F_traj;

bool record_flag;

#ifdef HAVE_ROS
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
    //get rid of tool-pokingstick gravity
    ft_gama->calib_ft_f = right_rs->robot_orien["eef"] * ft_gama->raw_ft_f + grav_tmp;
    ft_gama->calib_ft_t = ft_gama->raw_ft_t;
    
    ft_gama->filtered_gama_f = gama_f_filter->push(ft_gama->calib_ft_f);
    ft_gama->filtered_gama_t = gama_t_filter->push(ft_gama->calib_ft_t);
    
    counter ++;
    if(counter%50==0){
        counter = 0;
        std::cout<<"estimated contact force: "<<ft_gama->filtered_gama_f(0)<<","<<ft_gama->filtered_gama_f(1)<<","<<ft_gama->filtered_gama_f(2)<<std::endl;
}
    
    mutex_force.unlock();
}

#endif

void schunkJSviarsb(){
    mutex_schunkjs_app.lock();
    com_rsb->schunkjs_receive(schunkjs);
//     std::cout<<"received schunk joint angle are ";
//     for(int i = 0; i < schunkjs.size(); i++){
//         std::cout<<schunkjs.at(i)<<",";
//     }
//     std::cout<<std::endl;
    mutex_schunkjs_app.unlock();
}

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}
void tactool_grav_comp_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to psudo_gravity_compasenstation control"<<std::endl;
    rmt = PsudoGravityCompensation;
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
    rmt = NormalMode;
}

void moveto_cb(boost::shared_ptr<std::string> data){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) =  0.1;
    p(1) =  right_rs->robot_position["eef"](1);
    p(2) = right_rs->robot_position["eef"](2);;

    o(0) = 0.0;
    o(1) = M_PI;
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

void record_cb(boost::shared_ptr<std::string> data){
    record_flag = true;
    std::cout<<"start record data"<<std::endl;
}

void run(){
#ifdef HAVE_ROS
    ros::Rate r(500);
    while (ros::ok())
    {
      r.sleep();
      ros::spinOnce();
    }
#endif
}


void run_leftarm(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){

//        kuka_right_arm->update_robot_stiffness();
        kuka_right_arm->get_joint_position_act();
        kuka_right_arm->update_robot_state();
        right_rs->updated(kuka_right_arm);

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

void ros_publisher(){
    
//     tf::StampedTransform transform_rev;
//     try{
//         listener.lookupTransform("/vicon/WiperTool_4small_markers/wiper_frame", "/world",
//                                   ros::Time(0), transform_rev);
//     }
//     catch (tf::TransformException &ex) {
//          ROS_ERROR("%s",ex.what());
//          ros::Duration(1.0).sleep();
//       }
//     tf::Transform transform_new(transform_rev.getRotation(), tf::Vector3(transform_rev.getOrigin().getX(), transform_rev.getOrigin().getY(), transform_rev.getOrigin().getZ()+0.04));
//     br->sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), "world", "new_viconmarker_frame"));
    
    //prepare joint state data

   for(unsigned int i=0 ; i< 7;++i){
        //there is a arm name changed because the confliction between openkc and kukas in rviz
        js_la.position[i]=right_rs->JntPosition_mea[i];
        js_ra.position[i]=0;
    }
    mutex_schunkjs_app.lock();
//     std::cout<<"ros pub schunk js size is "<<schunkjs.size()<<" value are ";
    for(unsigned int i = 0; i < schunkjs.size(); i++){
        js_schunk.position[i] = schunkjs.at(i)*M_PI/180.0;
//         std::cout<<js_schunk.position[i]<<",";
    }
//     std::cout<<std::endl;
    mutex_schunkjs_app.unlock();
    
    js_la.header.stamp=js_schunk.header.stamp=js_ra.header.stamp=ros::Time::now();

    // send a joint_state
    jsPub_la.publish(js_la);
    jsPub_ra.publish(js_ra);
    jsPub_schunk.publish(js_schunk);
    
    tf::Matrix3x3 tfR;
    tf::Transform transform;
    Quaterniond eef_q;
    tf::matrixEigenToTF (right_rs->robot_orien["eef"], tfR);
    eef_q = right_rs->robot_orien["eef"];
    transform.setOrigin( tf::Vector3(right_rs->robot_position["eef"](0), right_rs->robot_position["eef"](1), right_rs->robot_position["eef"](2)) );
    transform.setBasis(tfR);
    br->sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "right_kuka_eef"));
    
    tf::StampedTransform transform_rev;
    try{
        listener->waitForTransform("/world","new_viconmarker_frame", ros::Time(0), ros::Duration(10.0) );
        listener->lookupTransform("/world", "new_viconmarker_frame",
                                  ros::Time(0), transform_rev);
    }
    catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
      }
    
    if(record_flag == true){
        F_traj<<right_rs->JntPosition_mea[0]<<","<<right_rs->JntPosition_mea[1]<<","<<right_rs->JntPosition_mea[2]<<","<<right_rs->JntPosition_mea[3]<<","<<right_rs->JntPosition_mea[4]<<","<<right_rs->JntPosition_mea[5]<<","<<right_rs->JntPosition_mea[6]<<","<<right_rs->robot_position["eef"](0)<<","<<right_rs->robot_position["eef"](1)<<","<<right_rs->robot_position["eef"](2)<<","<<eef_q.w()<<","<<eef_q.x()<<","<<eef_q.y()<<","<<eef_q.z()<<","<<transform_rev.getOrigin().getX()<<","<<transform_rev.getOrigin().getY()<<","<<transform_rev.getOrigin().getZ()<<","<<transform_rev.getRotation().getW()<<","<<transform_rev.getRotation().getX()<<","<<transform_rev.getRotation().getY()<<","<<transform_rev.getRotation().getZ()<<","<<ft_gama->filtered_gama_f(0)<<","<<ft_gama->filtered_gama_f(1)<<","<<ft_gama->filtered_gama_f(2)<<std::endl;
    }
}

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
    arm_payload_g.setZero();
    tn = none;
    StopFlag = false;
    record_flag = false;

//    init_tool_pose.p.setZero();
//    init_tool_pose.o.setZero();
//    init_tool_pose.rel_o.setZero();
    //declare the cb function
    boost::function<void(boost::shared_ptr<std::string>)> button_moveto(moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_record(record_cb);
//     boost::function<void(boost::shared_ptr<std::string>)> button_ftcalib(ftcalib_cb);
//     boost::function<void(boost::shared_ptr<std::string>)> button_gamaftcalib(gamaftcalib_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_grav_comp_ctrl(tactool_grav_comp_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_tactool_normal_ctrl(tactool_normal_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);

    std::string config_filename = selfpath + "/etc/right_arm_param.xml";
    if(is_file_exist(config_filename.c_str()) == false){
        config_filename = "right_arm_param.xml";
        if(is_file_exist(config_filename.c_str()) == false){
            std::cout<<"not find the tactile servo controller configure file"<<std::endl;
            exit(0);
        }
    }

    pm = new ParameterManager(config_filename);
    com_okc = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_right_arm = new KukaLwr(kuka_right,*com_okc,tn);
    right_rs = new RobotState(kuka_right_arm);
    kuka_right_arm->get_joint_position_act();
    kuka_right_arm->update_robot_state();
    right_rs->updated(kuka_right_arm);
    //initialize hand-hold manipulation tool--here is a tactile brush
    mt_ptr = new ManipTool(right_rs);
    mt_ptr ->mtt = Tacbrush;
    mt_ptr->ts.dof_num = 0;
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
    rdtrightkuka = RightKukaEff;
    rdtschunkjs = SchunkJS;
    com_rsb->add_msg(rdtrightkuka);
    com_rsb->add_msg(rdtschunkjs);
    ft.setZero(6);
    
    gama_f_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    gama_t_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    //register cb function
    com_rsb->register_external("/foo/moveto",button_moveto);
//     com_rsb->register_external("/foo/ftcalib",button_ftcalib);
//     com_rsb->register_external("/foo/gamaftcalib",button_gamaftcalib);
    com_rsb->register_external("/foo/record",button_record);
    com_rsb->register_external("/foo/tactool_grav_comp_ctrl",button_tactool_grav_comp_ctrl);
    com_rsb->register_external("/foo/tactool_normal_ctrl",button_tactool_normal_ctrl);
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
    js_schunk.header.frame_id="frame_lh";

    jsPub_la = nh->advertise<sensor_msgs::JointState> ("/la/joint_states", 2);
    jsPub_ra = nh->advertise<sensor_msgs::JointState> ("/ra/joint_states", 2);
    jsPub_schunk = nh->advertise<sensor_msgs::JointState> ("/lh/joint_states", 2);
    ros::spinOnce();

    br = new tf::TransformBroadcaster();
    listener = new tf::TransformListener();
    std::cout<<"ros init finished"<<std::endl;
#endif
}

int main(int argc, char* argv[])
{

    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);
    StopFlag = false;
    #ifdef HAVE_ROS
        ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
        ros::Subscriber GammaFtSub;
        std::cout<<"Connect to ATI FT sensor"<<std::endl;
        GammaFtSub=nh->subscribe("/ft_sensor/wrench", 1, // buffer size 
                                &recvFT);
    #endif
    std::string data_f ("/tmp/");
    F_traj.open((data_f+std::string("traj.txt")).c_str());
    init();
    //start a timer thread
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(1));
    thrd_getMsg.start(true);

    //start kuka arm control thread
    Timer thrd_kuka_ctrl(run_leftarm);
    thrd_kuka_ctrl.setSingleShot(false);
    thrd_kuka_ctrl.setInterval(Timer::Interval(1));
    thrd_kuka_ctrl.start(true);
    
        //start Schunk hand read thread
    Timer thrd_schunk_read(schunkJSviarsb);
    thrd_schunk_read.setSingleShot(false);
    thrd_schunk_read.setInterval(Timer::Interval(100));
    thrd_schunk_read.start(true);
    
    #ifdef HAVE_ROS
    //start ros publisher thread
    Timer thrd_rospublisher(ros_publisher);
    thrd_rospublisher.setSingleShot(false);
    thrd_rospublisher.setInterval(Timer::Interval(20));
    thrd_rospublisher.start(true);
    #endif
    
    
    counter = 0;
    while(!StopFlag){
    }

    //stop ros
    #ifdef HAVE_ROS
    ros::shutdown();
    delete nh;
    #endif

    //stop the timer
    thrd_kuka_ctrl.stop();
    #ifdef HAVE_ROS
    thrd_getMsg.stop();
    thrd_rospublisher.stop();
    #endif
    return 0;
}



