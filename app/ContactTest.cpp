
/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : kuka safely contact test
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

#include <ICLUtils/Mutex.h>

HSplit gui;
ComOkc *com_okc;
Robot *kuka_left_arm;
ActController *ac;
Task *task;
TaskNameT taskname;
ComRSB *com_rsb;
RsbDataType rdtlefttac,rdtleftkuka;
myrmex_msg left_myrmex_msg;
ParameterManager* pm;

#define newP_x 0.28
#define newP_y 0.3
#define newP_z 0.30

#define newO_x 0.0
#define newO_y M_PI/2;
#define newO_z 0.0;

//using mutex locking controller ptr while it is switching.
Mutex mutex_act,mutex_tactile;


void contact_cb(void){
    mutex_act.lock();
    delete ac;
    delete task;
    taskname.tact = CONTACT_FORCE_TRACKING;
    ac = new TacServoController(*pm);
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task = new TacServoTask(taskname.tact);
    task->mt = TACTILE;
    task->set_desired_cf_myrmex(0.05);
    mutex_act.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}
void moveto_cb(void){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = (-1)*newP_x;
    p(1) = newP_y;
    p(2) = newP_z;

    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    mutex_act.lock();
    delete ac;
    delete task;
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    task->mt == JOINTS;
    task->mft = GLOBAL;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    mutex_act.unlock();
    std::cout<<"robot self movement and move to new pose"<<std::endl;
}

void sliding_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_act.lock();
    delete ac;
    delete task;
    taskname.tact = CONTACT_POINT_TRACKING;
    ac = new TacServoController(*pm);
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task = new TacServoTask(taskname.tact);
    task->mt = TACTILE;
    task->set_desired_cp_myrmex(cp);
    mutex_act.unlock();
    std::cout<<"switch sliding task"<<std::endl;
}

void rolling_cb(void){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_act.lock();
    delete ac;
    delete task;
    taskname.tact = COVER_OBJECT_SURFACE;
    ac = new TacServoController(*pm);
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task = new TacServoTask(taskname.tact);
    task->mt = TACTILE;
    task->set_desired_cp_myrmex(cp);
    mutex_act.unlock();
    std::cout<<"switch rolling task"<<std::endl;
}

void twisting_cb(){
    double cp[2];
    cp[0] = 8;
    cp[1] = 8;
    mutex_act.lock();
    delete ac;
    delete task;
    taskname.tact = Z_ORIEN_TRACKING;
    ac = new TacServoController(*pm);
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task = new TacServoTask(taskname.tact);
    task->mt = TACTILE;
    task->set_desired_cp_myrmex(cp);
    mutex_act.unlock();
    std::cout<<"switch twisting task"<<std::endl;
}

void brake_cb(void){
    com_okc->start_brake();
}

void nobrake_cb(void){
    com_okc->release_brake();
}

void updateadmittance_cb(){
//    mutex_act.lock();
//    taskname.tact = Z_ORIEN_TRACKING;
//    ac->update_controller_para();
//    ac->updateTacServoCtrlParam(taskname.tact);
//    mutex_act.unlock();
}


void run(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
//        kuka_left_arm->update_robot_stiffness();
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        //using all kinds of controllers to update the reference
        mutex_act.lock();
        if(task->mt == JOINTS)
            ac->update_robot_reference(kuka_left_arm,task);
        if(task->mt == TACTILE){
            mutex_tactile.lock();
            ac->update_robot_reference(kuka_left_arm,task,&left_myrmex_msg);
            mutex_tactile.unlock();
        }
        ac->llv.setZero();
        ac->lov.setZero();
        mutex_act.unlock();
        //use CBF to compute the desired joint angle rate
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command();
        com_okc->controller_update = true;
    }
}

void tactileviarsb(){
    //via network--RSB, the contact information are obtained.
    static FPSLimiter fpsl2(500);
    fpsl2.wait();
    mutex_tactile.lock();
    com_rsb->tactile_receive(left_myrmex_msg,"leftmyrmex");
//    std::cout<<"contact information "<<left_myrmex_msg.cf<<","<<left_myrmex_msg.cogx<<","<<left_myrmex_msg.cogy<<","<<left_myrmex_msg.contactflag<<std::endl;
    mutex_tactile.unlock();
}

void init(){
    gui << (VBox()
        << Button("Moveto").handle("moveto_task")
        << Button("Contact").handle("contact_task")
        << Button("Slidng").handle("sliding_task")
        << Button("Rolling").handle("rolling_task")
        << Button("Twisting").handle("twisting_task")
        << Button("Brake").handle("brake_task")
        << Button("NoBrake").handle("nobrake_task")
        << Button("UAdmittanceParam").handle("uadmittanceparam_task")
        )
    << Show();
    gui["contact_task"].registerCallback(utils::function(contact_cb));
    gui["moveto_task"].registerCallback(utils::function(moveto_cb));
    gui["sliding_task"].registerCallback(utils::function(sliding_cb));
    gui["rolling_task"].registerCallback(utils::function(rolling_cb));
    gui["twisting_task"].registerCallback(utils::function(twisting_cb));
    gui["brake_task"].registerCallback(utils::function(brake_cb));
    gui["nobrake_task"].registerCallback(utils::function(nobrake_cb));
    gui["uadmittanceparam_task"].registerCallback(utils::function(updateadmittance_cb));
    pm = new ParameterManager("left_arm_param.xml");
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc);
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    kuka_left_arm->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, \
                                           ac->pm.stiff_ctrlpara.axis_damping);
//    std::cout<<"before start rsb........................................................................"<<std::endl;
    com_rsb = new ComRSB();
    rdtlefttac = LeftMyrmex;
    rdtleftkuka = LeftKukaEff;
    com_rsb->add_msg(rdtlefttac);
    com_rsb->add_msg(rdtleftkuka);
//    std::cout<<" after start rsb..............................................................................."<<std::endl;
}

int main(int argc, char* argv[])
{
    return ICLApp(argc,argv,"-input|-i(2)",init,run,tactileviarsb).exec();
}

