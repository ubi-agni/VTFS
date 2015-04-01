

/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : kuka left finger follow the kuka right palm movement while they are contacted.
 ============================================================================
 */

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

#include <ICLUtils/Mutex.h>

std::ofstream TDataRecord;

HSplit gui;
ComOkc *com_okc_left;
ComOkc *com_okc_right;
ComRSB *com_rsb;
Robot *kuka_left_arm;
ActController *left_ac;
Task *left_task;
TaskNameT left_taskname;

Robot *kuka_right_arm;
ActController *right_ac;
Task *right_task;
TaskNameT right_taskname;

RsbDataType rdtlefttac,rdtleftkuka;
myrmex_msg left_myrmex_msg;

RsbDataType rdtrighttac,rdtrightkuka;
myrmex_msg right_myrmex_msg;

ParameterManager *left_pm, *right_pm;

#define left_newP_x 0.10
#define left_newP_y 0.3
#define left_newP_z 0.30

#define left_newO_x 0.0
#define left_newO_y M_PI/2;
#define left_newO_z 0.0;

#define right_newP_x -0.10
#define right_newP_y 0.3
#define right_newP_z 0.30

#define right_newO_x 0.0
#define right_newO_y -M_PI/2;
#define right_newO_z 0.0;

//using mutex locking controller ptr while it is switching.
Mutex mutex_leftact,mutex_rightact,mutex_tactile;


void contact_cb(void){
    mutex_leftact.lock();
    delete left_ac;
    delete left_task;
    left_taskname.tact = CONTACT_FORCE_TRACKING;
    left_ac = new TacServoController(*left_pm);
    left_ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task = new TacServoTask(left_taskname.tact);
    left_task->mt = TACTILE;
    left_task->set_desired_cf_myrmex(0.15);
    mutex_leftact.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}
void moveto_cb(void){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = (-1)*left_newP_x;
    p(1) = left_newP_y;
    p(2) = left_newP_z+0.05;

    o(0) = left_newO_x;
    o(1) = left_newO_y;
    o(2) = left_newO_z;
    mutex_leftact.lock();
    delete left_ac;
    delete left_task;
    left_ac = new ProActController(*left_pm);
    left_task = new KukaSelfCtrlTask(RP_NOCONTROL);
    left_task->mt == JOINTS;
    left_task->mft = GLOBAL;
    left_task->set_desired_p_eigen(p);
    left_task->set_desired_o_ax(o);
    mutex_leftact.unlock();
    std::cout<<"left robot self movement and move to new pose"<<std::endl;

    p(0) = (-1)*right_newP_x;
    p(1) = right_newP_y;
    p(2) = right_newP_z+0.05;

    o(0) = right_newO_x;
    o(1) = right_newO_y;
    o(2) = right_newO_z;
    mutex_rightact.lock();
    delete right_ac;
    delete right_task;
    right_ac = new ProActController(*left_pm);
    right_task = new KukaSelfCtrlTask(RP_NOCONTROL);
    right_task->mt == JOINTS;
    right_task->mft = GLOBAL;
    right_task->set_desired_p_eigen(p);
    right_task->set_desired_o_ax(o);
    mutex_rightact.unlock();
    std::cout<<"right robot self movement and move to new pose"<<std::endl;


}

void back_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    delete left_ac;
    delete left_task;
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac = new TacServoController(*left_pm);
    left_ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task = new TacServoTask(left_taskname.tact);
    left_task->mt = TACTILE;
    left_task->set_desired_cp_myrmex(cp);
    mutex_leftact.unlock();
    std::cout<<"left arm switch to following mode"<<std::endl;

    mutex_rightact.lock();
    delete right_ac;
    delete right_task;
    right_taskname.prot = RLYP;
    right_ac = new ProActController(*right_pm);
    right_ac->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task = new KukaSelfCtrlTask(right_taskname.prot);
    right_task ->mft = LOCAL;
    right_task ->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}

void forward_cb(void){
//    double cp[2];
//    cp[0] = 8;
//    cp[1] = 8;
//    mutex_leftact.lock();
//    delete left_ac;
//    delete left_task;
//    left_taskname.tact = COVER_OBJECT_SURFACE;
//    left_ac = new TacServoController(*left_pm);
//    left_ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
//    left_task = new TacServoTask(left_taskname.tact);
//    left_task->mt = TACTILE;
//    left_task->set_desired_cp_myrmex(cp);
//    mutex_leftact.unlock();
//    std::cout<<"switch rolling task"<<std::endl;

    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    delete left_ac;
    delete left_task;
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac = new TacServoController(*left_pm);
    left_ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task = new TacServoTask(left_taskname.tact);
    left_task->mt = TACTILE;
    left_task->set_desired_cp_myrmex(cp);
    mutex_leftact.unlock();
    std::cout<<"left arm switch to following mode"<<std::endl;

    mutex_rightact.lock();
    delete right_ac;
    delete right_task;
    right_taskname.prot = RLYN;
    right_ac = new ProActController(*right_pm);
    right_ac->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task = new KukaSelfCtrlTask(right_taskname.prot);
    right_task ->mft = LOCAL;
    right_task ->mt = JOINTS;
    mutex_rightact.unlock();
    std::cout<<"right arm switch to open motion mode"<<std::endl;
}

void idle_cb(){
//    double cp[2];
//    cp[0] = 8;
//    cp[1] = 8;
//    mutex_leftact.lock();
//    delete left_ac;
//    delete left_task;
//    left_taskname.tact = Z_ORIEN_TRACKING;
//    left_ac = new TacServoController(*left_pm);
//    left_ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
//    left_task = new TacServoTask(left_taskname.tact);
//    left_task->mt = TACTILE;
//    left_task->set_desired_cp_myrmex(cp);
//    mutex_leftact.unlock();
//    std::cout<<"switch twisting task"<<std::endl;

    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_leftact.lock();
    delete left_ac;
    delete left_task;
    left_taskname.tact = SENSING_POLE_TRACKING;
    left_ac = new TacServoController(*left_pm);
    left_ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    left_task = new TacServoTask(left_taskname.tact);
    left_task->mt = TACTILE;
    left_task->set_desired_cp_myrmex(cp);
    mutex_leftact.unlock();
    std::cout<<"left arm switch to following mode"<<std::endl;

    mutex_rightact.lock();
    delete right_ac;
    delete right_task;
    right_taskname.prot = RP_NOCONTROL;
    right_ac = new ProActController(*right_pm);
    right_ac->set_init_TM(kuka_right_arm->get_cur_cart_o());
    right_task = new KukaSelfCtrlTask(right_taskname.prot);
    right_task ->mft = LOCAL;
    right_task ->mt = JOINTS;
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
    left_ac->set_pm(*left_pm);
    left_taskname.tact = CONTACT_FORCE_TRACKING;
    left_ac->updateTacServoCtrlParam(left_taskname.tact);
    mutex_leftact.unlock();
}


void run_leftarm(){
    if((com_okc_left->data_available == true)&&(com_okc_left->controller_update == false)){
        //for left arm
//        kuka_left_arm->update_robot_stiffness();
        //only call for this function, the ->jnt_position_act is updated
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        //using all kinds of controllers to update the reference
        mutex_leftact.lock();
        if(left_task->mt == JOINTS)
            left_ac->update_robot_reference(kuka_left_arm,left_task);
        if(left_task->mt == TACTILE){
            mutex_tactile.lock();
            left_ac->update_robot_reference(kuka_left_arm,left_task,&right_myrmex_msg);
            mutex_tactile.unlock();
        }
        //update with act_vec
        left_ac->llv.setZero();
        left_ac->lov.setZero();
        mutex_leftact.unlock();
        //use CBF to compute the desired joint angle rate
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command();
        com_okc_left->controller_update = true;
    }

}

void run_rightarm(){
    if((com_okc_right->data_available == true)&&(com_okc_right->controller_update == false)){
        //for right arm
//        kuka_right_arm->update_robot_stiffness();
        //only call for this function, the ->jnt_position_act is updated
        kuka_right_arm->get_joint_position_act();
        kuka_right_arm->update_robot_state();
        //using all kinds of controllers to update the reference
        mutex_rightact.lock();
        if(right_task->mt == JOINTS)
            right_ac->update_robot_reference(kuka_right_arm,right_task);
        if(right_task->mt == TACTILE){
            mutex_tactile.lock();
            right_ac->update_robot_reference(kuka_right_arm,right_task,&right_myrmex_msg);
            mutex_tactile.unlock();
        }
        right_ac->llv.setZero();
        right_ac->lov.setZero();
        mutex_rightact.unlock();
        //use CBF to compute the desired joint angle rate
        kuka_right_arm->update_cbf_controller();
        kuka_right_arm->set_joint_command();
        com_okc_right->controller_update = true;
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
    std::cout<<"right contact information "<<right_myrmex_msg.cf<<","<<right_myrmex_msg.cogx<<","<<right_myrmex_msg.cogy<<","<<right_myrmex_msg.contactflag<<std::endl;
    TDataRecord<<right_myrmex_msg.cf<<","<<right_myrmex_msg.cogx<<","<<right_myrmex_msg.cogy<<","<<right_myrmex_msg.contactflag<<std::endl;
    mutex_tactile.unlock();
}

void init(){
    gui << (VBox()
        << Button("Moveto").handle("moveto_task")
        << Button("Contact").handle("contact_task")
        << Button("LeftBrake").handle("leftbrake_task")
        << Button("RightBrake").handle("rightbrake_task")
        << Button("LeftNoBrake").handle("leftnobrake_task")
        << Button("RightNoBrake").handle("rightnobrake_task")
        << Button("Back").handle("back_task")
        << Button("Forward").handle("forward_task")
        << Button("Idle").handle("idle_task")
        << Button("UAdmittanceParam").handle("uadmittanceparam_task")
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
    gui["idle_task"].registerCallback(utils::function(idle_cb));
    gui["uadmittanceparam_task"].registerCallback(utils::function(updateadmittance_cb));

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

    Eigen::Vector3d p_left,o_left,p_right,o_right;
    p_left.setZero();
    o_left.setZero();
    p_right.setZero();
    o_right.setZero();

    left_ac = new ProActController(*left_pm);
    left_task = new KukaSelfCtrlTask(RP_NOCONTROL);
    p_left(0) = (-1)*left_newP_x;
    p_left(1) = left_newP_y;
    p_left(2) = left_newP_z;
    o_left(0) = left_newO_x;
    o_left(1) = left_newO_y;
    o_left(2) = left_newO_z;
    left_task->set_desired_p_eigen(p_left);
    left_task->set_desired_o_ax(o_left);
    kuka_left_arm->setAxisStiffnessDamping(left_ac->pm.stiff_ctrlpara.axis_stiffness, \
                                           left_ac->pm.stiff_ctrlpara.axis_damping);
    right_ac = new ProActController(*right_pm);
    right_task = new KukaSelfCtrlTask(RP_NOCONTROL);

    p_right(0) = (-1) * right_newP_x;
    p_right(1) = right_newP_y;
    p_right(2) = right_newP_z;
    o_right(0) = right_newO_x;
    o_right(1) = right_newO_y;
    o_right(2) = right_newO_z;
    right_task->set_desired_p_eigen(p_right);
    right_task->set_desired_o_ax(o_right);
    kuka_right_arm->setAxisStiffnessDamping(right_ac->pm.stiff_ctrlpara.axis_stiffness, \
                                           right_ac->pm.stiff_ctrlpara.axis_damping);
    std::cout<<"finish init"<<std::endl;
}

int main(int argc, char* argv[])
{
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
    return ICLApp(argc,argv,"-input|-i(2)",init,run_leftarm,run_rightarm,tactileviarsb).exec();
}


