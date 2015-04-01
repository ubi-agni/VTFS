


/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : left kuka using estimated force feedback to follow the unknown object surface
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
#include "ControllerModule/forceservocontroller.h"
#include "TaskModule/forceservotask.h"
//#include "ControllerModule/CtrlParam.h"

#include <ICLUtils/Mutex.h>
#include "RobotModule/RobotState.h"
//#include "UtilModule/TemporalSmoothingFilter.h"

HSplit gui;
ComOkc *com_okc;
Robot *kuka_left_arm;
ActController *ac;
Task *task;
TaskNameT taskname;
ComRSB *com_rsb;
RsbDataType rdtleftkuka;
ParameterManager* pm;
RobotState *left_rs;
kuka_msg left_kuka_msg;


#define newP_x -0.1
#define newP_y 0.3
#define newP_z 0.30

#define newO_x 0.0
#define newO_y M_PI/2;
#define newO_z 0.0;

//using mutex locking controller ptr while it is switching.
Mutex mutex_act, mutex_force;

//estimated force/torque from fri
Eigen::Vector3d estkukaforce,estkukamoment;
Vec filtered_force;
Eigen::VectorXd ft;
TemporalSmoothingFilter<Vec>* cf_filter;

void contact_cb(void){
    mutex_act.lock();
    delete ac;
    delete task;
    taskname.forcet = F_MAINTAIN;
    ac = new ForceServoController(*pm);
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task = new ForceServoTask(taskname.forcet);
    task->mt = FORCE;
    task->set_desired_cf_kuka(3);
    mutex_act.unlock();
    std::cout<<"maintain the force"<<std::endl;
}

void follow_cb(void){
    mutex_act.lock();
    delete ac;
    delete task;
    taskname.forcet = F_CURVETRACKING;
    ac = new ForceServoController(*pm);
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task = new ForceServoTask(taskname.forcet);
    task->mt = FORCE;
    task->set_desired_cf_kuka(3);
    mutex_act.unlock();
    std::cout<<"curve tracking"<<std::endl;
}


void moveto_cb(void){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = newP_x;
    p(1) = newP_y;
    p(2) = newP_z+0.1;

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

void ftcalib_cb(void){
    kuka_left_arm->calibForce(1000);
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
        left_rs->updated(kuka_left_arm);
        kuka_left_arm->getTcpFtCalib(estkukaforce);
        filtered_force = cf_filter->push(Vec(estkukaforce(0),estkukaforce(1),estkukaforce(2),1));
        std::cout<<"force are "<<estkukaforce(0)<<","<<estkukaforce(1)<<","<<estkukaforce(2)<<std::endl;
        mutex_force.lock();
        for(int i = 0; i < 3; i++){
            ft(i) = filtered_force[i];
            ft(i+3) = estkukamoment(i);
        }
        mutex_force.unlock();
        //using all kinds of controllers to update the reference
        mutex_act.lock();
        if(task->mt == JOINTS)
            ac->update_robot_reference(kuka_left_arm,task);
        if(task->mt == FORCE){
            mutex_force.lock();
            ac->update_robot_reference(kuka_left_arm,task,ft,left_rs);
            mutex_force.unlock();
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

void ftsending(void){
    //via network--RSB, the contact information are obtained.
    static FPSLimiter fpsl2(500);
    fpsl2.wait();
    mutex_force.lock();
    left_kuka_msg.p = kuka_left_arm->get_cur_cart_p();
    left_kuka_msg.o = kuka_left_arm->get_cur_cart_o();
    left_kuka_msg.ft.setZero(6);
    left_kuka_msg.ft = ft;
    com_rsb->kuka_msg_send(left_kuka_msg,"leftkuka");
    mutex_force.unlock();
}

void init(){
    gui << (VBox()
        << Button("Moveto").handle("moveto_task")
        << Button("ftCalib").handle("ftcalib_task")
        << Button("Contact").handle("contact_task")
        << Button("Follow").handle("follow_task")
        << Button("Brake").handle("brake_task")
        << Button("NoBrake").handle("nobrake_task")
        << Button("UAdmittanceParam").handle("uadmittanceparam_task")
        )
    << Show();
    gui["contact_task"].registerCallback(utils::function(contact_cb));
    gui["moveto_task"].registerCallback(utils::function(moveto_cb));
    gui["ftcalib_task"].registerCallback(utils::function(ftcalib_cb));
    gui["follow_task"].registerCallback(utils::function(follow_cb));
    gui["brake_task"].registerCallback(utils::function(brake_cb));
    gui["nobrake_task"].registerCallback(utils::function(nobrake_cb));
    gui["uadmittanceparam_task"].registerCallback(utils::function(updateadmittance_cb));
    pm = new ParameterManager("left_arm_param.xml");
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc);
    left_rs = new RobotState(kuka_left_arm);
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = newP_x;
    p(1) = newP_y;
    p(2) = newP_z;

    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    kuka_left_arm->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, \
                                           ac->pm.stiff_ctrlpara.axis_damping);
    com_rsb = new ComRSB();
    rdtleftkuka = LeftKukaEff;
    com_rsb->add_msg(rdtleftkuka);
    ft.setZero(6);
    estkukaforce.setZero();
    estkukamoment.setZero();
    cf_filter = new TemporalSmoothingFilter<Vec>(5,Average,Vec(0,0,0,0));
}

int main(int argc, char* argv[])
{
    return ICLApp(argc,argv,"-input|-i(2)",init,run,ftsending).exec();
}

