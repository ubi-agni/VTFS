/*
 ============================================================================
 Name        : MyrmexServoing.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : use myrmex and right arm for tactile servoing
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
#define TAC_F 0.3

#ifdef HAVE_ROS
// ROS objects
tf::TransformBroadcaster *br;
sensor_msgs::JointState js;
ros::Publisher jsPub;
ros::NodeHandle *nh;
#endif

ComOkc *com_okc;
Robot *kuka_right_arm;
std::vector<ActController *> right_ac_vec;
std::vector<Task *> right_task_vec;
TaskNameT right_taskname;

ComRSB *com_rsb;
RsbDataType rdtrightkuka;
RsbDataType rdtrighttac;
myrmex_msg right_myrmex_msg;
ParameterManager* pm;
RobotState *right_rs;
gamaFT *ft_gama;
PCAFeature *pcaf;

TacSensorType TacST;

#define newP_x 0.2
#define newP_y 0.3
#define newP_z 0.30

#define newO_x 0.0
#define newO_y -M_PI/2;
#define newO_z 0.0;


double initP_x,initP_y,initP_z;
double initO_x,initO_y,initO_z;


//using mutex locking controller ptr while it is switching.
std::mutex mutex_act, mutex_force,mutex_tac,mutex_ft;
uint32_t marker_shape;
ToolNameT tn;

bool StopFlag;

//define the robot control mode: normal mode vs psudo_gravity_compensation mode
RobotModeT rmt;


void tactileviarsb(){
    //via network--RSB, the contact information are obtained.
    mutex_tac.lock();
    com_rsb->tactile_receive(right_myrmex_msg,"rightmyrmex");
//    std::cout<<"tactile force output "<<right_myrmex_msg.cf<<std::endl;
//    std::cout<<"myrmex readout "<<right_myrmex_msg.cogx<<","<<right_myrmex_msg.cogy<<std::endl;
    mutex_tac.unlock();
}

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}


void tactile_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.tact = CONTACT_FORCE_TRACKING;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for maintain contact"<<std::endl;
}

void taxel_sliding_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.tact = CONTACT_POINT_FORCE_TRACKING;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}


void twist_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.tact = Z_ORIEN_TRACKING;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for sliding to the desired point"<<std::endl;
}


void taxel_rolling_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    right_ac_vec.clear();
    right_task_vec.clear();
    right_taskname.tact = COVER_OBJECT_SURFACE;
    right_ac_vec.push_back(new TacServoController(*pm));
    right_ac_vec.back()->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task_vec.push_back(new TacServoTask(right_taskname.tact));
    right_task_vec.back()->mt = TACTILE;
    right_task_vec.back()->set_desired_cf_myrmex(TAC_F);
    mutex_act.unlock();
    std::cout<<"tactile servoing for rolling to the desired point"<<std::endl;
}


void grav_comp_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to psudo_gravity_compasenstation control"<<std::endl;
    rmt = PsudoGravityCompensation;
}

void normal_ctrl_cb(boost::shared_ptr<std::string> data){
    std::cout<<"switch to normal control"<<std::endl;
    rmt = NormalMode;
}



void moveto_cb(boost::shared_ptr<std::string> data){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();

    //get start point position in cartesian space
    p(0) =  0.1;
    p(1) =  0.3;
    p(2) = 0.3;

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


void brake_cb(boost::shared_ptr<std::string> data){
    com_okc->start_brake();
}

void nobrake_cb(boost::shared_ptr<std::string> data){
    com_okc->release_brake();
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
    TacST = Myrmex;
    StopFlag = false;
    //declare the cb function
    boost::function<void(boost::shared_ptr<std::string>)> button_tactile(tactile_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_taxel_sliding(taxel_sliding_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_taxel_rolling(taxel_rolling_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_twist(twist_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_moveto(moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_grav_comp_ctrl(grav_comp_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_normal_ctrl(normal_ctrl_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_brake(brake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_nobrake(nobrake_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);

    std::string config_filename = selfpath + "/etc/right_arm_param.xml";
//    std::cout<<"config name is "<<config_filename<<std::endl;
    if(is_file_exist(config_filename.c_str()) == false){
        config_filename = "right_arm_param.xml";
        if(is_file_exist(config_filename.c_str()) == false){
            std::cout<<"not find the tactile servo controller configure file"<<std::endl;
            exit(0);
        }
    }

    pm = new ParameterManager(config_filename,TacST);
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
    rdtrightkuka = RightKukaEff;
    rdtrighttac = RightMyrmex;
    com_rsb->add_msg(rdtrightkuka);
    com_rsb->add_msg(rdtrighttac);
    //register cb function
    com_rsb->register_external("/foo/moveto",button_moveto);
    com_rsb->register_external("/foo/tactile",button_tactile);
    com_rsb->register_external("/foo/taxel_sliding",button_taxel_sliding);
    com_rsb->register_external("/foo/taxel_rolling",button_taxel_rolling);
    com_rsb->register_external("/foo/twist",button_twist);
    com_rsb->register_external("/foo/grav_comp_ctrl",button_grav_comp_ctrl);
    com_rsb->register_external("/foo/normal_ctrl",button_normal_ctrl);
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

    jsPub = nh->advertise<sensor_msgs::JointState> ("joint_states", 2);
    ros::spinOnce();

    br = new tf::TransformBroadcaster();

    std::cout<<"ros init finished"<<std::endl;
#endif
}



void run_rightarm(){
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
            if(right_task_vec[i]->mt == TACTILE){
                mutex_tac.lock();
                right_ac_vec[i]->update_robot_reference(kuka_right_arm,right_task_vec[i],&right_myrmex_msg);
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
        ros::init(argc, argv, "MyrmexServo",ros::init_options::NoSigintHandler);
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
    #ifdef HAVE_ROS
    thrd_rospublisher.stop();
    #endif
    return 0;
}




