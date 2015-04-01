

/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : combine cover the object and kuka openforward movement test
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
#include <vector>                //used for task and controller vector

HSplit gui;
ComOkc *com_okc;
Robot *kuka_left_arm;
std::vector<ActController *> ac_vec;
std::vector<Task *> task_vec;
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
    ac_vec.clear();
    task_vec.clear();
    taskname.tact = CONTACT_FORCE_TRACKING;
    ac_vec.push_back(new TacServoController(*pm));
    ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task_vec.push_back(new TacServoTask(taskname.tact));
    task_vec.back()->mt = TACTILE;
    task_vec.back()->set_desired_cf_myrmex(0.1);
    mutex_act.unlock();
    std::cout<<"switch task and controller"<<std::endl;
}

void exploring_cb(void){
    mutex_act.lock();
    ac_vec.clear();
    task_vec.clear();

    taskname.tact = COVER_OBJECT_SURFACE;
    ac_vec.push_back(new TacServoController(*pm));
    ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task_vec.push_back(new TacServoTask(taskname.tact));
    task_vec.back()->mt = TACTILE;
    task_vec.back()->set_desired_cf_myrmex(0.1);

    taskname.prot = RLYP;
    ac_vec.push_back(new ProActController(*pm));
    ac_vec.back()->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task_vec.push_back(new KukaSelfCtrlTask(taskname.prot));
    task_vec.back()->mft = LOCAL;
    task_vec.back()->mt = JOINTS;
    mutex_act.unlock();
    std::cout<<"switch exploring and composite controllers"<<std::endl;
}


void brake_cb(void){
    com_okc->start_brake();
}

void nobrake_cb(void){
    com_okc->release_brake();
}


void run(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
//        kuka_left_arm->update_robot_stiffness();

        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        //using all kinds of controllers to update the reference
        mutex_act.lock();
        for(int i = 0; i < ac_vec.size(); i++){
            if(task_vec[i]->mt == JOINTS){
                ac_vec[i]->update_robot_reference(kuka_left_arm,task_vec[i]);
            }
            if(task_vec[i]->mt == TACTILE)
                ac_vec[i]->update_robot_reference(kuka_left_arm,task_vec[i],&left_myrmex_msg);
        }
        ac_vec[0]->llv.setZero();
        ac_vec[0]->lov.setZero();
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
        << Button("Contact").handle("contact_task")
        << Button("Exploroing").handle("exploring_task")
        << Button("Brake").handle("brake_task")
        << Button("NoBrake").handle("nobrake_task")
        )
    << Show();
    gui["contact_task"].registerCallback(utils::function(contact_cb));
    gui["exploring_task"].registerCallback(utils::function(exploring_cb));
    gui["brake_task"].registerCallback(utils::function(brake_cb));
    gui["nobrake_task"].registerCallback(utils::function(nobrake_cb));
    pm = new ParameterManager("left_arm_param.xml");
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc);
    ac_vec.push_back(new ProActController(*pm));
    task_vec.push_back(new KukaSelfCtrlTask(RP_NOCONTROL));

    kuka_left_arm->setAxisStiffnessDamping(ac_vec[0]->pm.stiff_ctrlpara.axis_stiffness, \
                                           ac_vec[0]->pm.stiff_ctrlpara.axis_damping);
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

