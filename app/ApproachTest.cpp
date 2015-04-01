
/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Kuka LWR self movement
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

#include "ComModule/ComOkc.h"
#include "RobotModule/KukaLwr.h"
#include "RobotModule/Robot.h"
#include "ControllerModule/proactcontroller.h"
#include "TaskModule/kukaselfctrltask.h"

//load the parameter which are stored in xml file
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"
#include "boost/foreach.hpp"
#include "ControllerModule/CtrlParam.h"
#include "ControllerModule/parametermanager.h"

#include <ICLUtils/Mutex.h>
HSplit gui;
ComOkc *com_okc;
Robot *kuka_left_arm;
ActController *ac;
Task *task;
ParameterManager* pm;


#define initP_x 0.28
#define initP_y 0.3
#define initP_z 0.30

#define initO_x 0.0
#define initO_y M_PI/2;
#define initO_z 0.0;

//using mutex locking controller ptr while it is switching.
Mutex mutex_act;
void moveto_cb(void){
    Eigen::Vector3d p;
    Eigen::Vector3d o;
    p.setZero();
    o.setZero();
    p(0) = -1 * initP_x;
    p(1) = initP_y;
    p(2) = initP_z;
    o(0) = initO_x;
    o(1) = initO_y;
    o(2) = initO_z;
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = GLOBAL;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    mutex_act.unlock();
    std::cout<<"Approach to predefined point"<<std::endl;
}

void global_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = GLOBAL;
    task->curtaskname.prot = RP_NOCONTROL;
    mutex_act.unlock();
    std::cout<<"Switch to global movment task"<<std::endl;
}

void local_cb(void){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RP_NOCONTROL;
    mutex_act.unlock();
    std::cout<<"Switch to local movment task"<<std::endl;
}

void lxp_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLXP;
    mutex_act.unlock();
    std::cout<<"Switch to lxp movment task"<<std::endl;
}
void lyp_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLYP;
    mutex_act.unlock();
    std::cout<<"Switch to lyp movment task"<<std::endl;
}
void lzp_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLZP;
    mutex_act.unlock();
    std::cout<<"Switch to lzp movment task"<<std::endl;
}
void rxp_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRXP;
    mutex_act.unlock();
    std::cout<<"Switch to rxp movment task"<<std::endl;
}
void ryp_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRYP;
    mutex_act.unlock();
    std::cout<<"Switch to ryp movment task"<<std::endl;
}
void rzp_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRZP;
    mutex_act.unlock();
    std::cout<<"Switch to rzp movment task"<<std::endl;
}
void lxn_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLXN;
    mutex_act.unlock();
    std::cout<<"Switch to lxn movment task"<<std::endl;
}
void lyn_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLYN;
    mutex_act.unlock();
    std::cout<<"Switch to lyn movment task"<<std::endl;
}
void lzn_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLZN;
    mutex_act.unlock();
    std::cout<<"Switch to lzn movment task"<<std::endl;
}
void rxn_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRXN;
    mutex_act.unlock();
    std::cout<<"Switch to rxn movment task"<<std::endl;
}
void ryn_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRYN;
    mutex_act.unlock();
    std::cout<<"Switch to ryn movment task"<<std::endl;
}
void rzn_cb(void){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRZN;
    mutex_act.unlock();
    std::cout<<"Switch to rzn movment task"<<std::endl;
}


void run(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
//        kuka_left_arm->update_robot_stiffness();
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        //using controller to update the reference
        mutex_act.lock();
        ac->update_robot_reference(kuka_left_arm,task);
        mutex_act.unlock();
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command();
        com_okc->controller_update = true;
    }
}


void init(){
    gui << (VBox()
        << Button("moveto").handle("moveto_task")
        << Button("local").handle("local_task")
        << Button("global").handle("global_task")
        )
        << (VBox()
        << Button("LXP").handle("lxp_task")
        << Button("LYP").handle("lyp_task")
        << Button("LZP").handle("lzp_task")
        << Button("RXP").handle("rxp_task")
        << Button("RYP").handle("ryp_task")
        << Button("RZP").handle("rzp_task")
        )
        << (VBox()
        << Button("LXN").handle("lxn_task")
        << Button("LYN").handle("lyn_task")
        << Button("LZN").handle("lzn_task")
        << Button("RXN").handle("rxn_task")
        << Button("RYN").handle("ryn_task")
        << Button("RZN").handle("rzn_task")
        )
    << Show();
    gui["moveto_task"].registerCallback(utils::function(moveto_cb));
    gui["local_task"].registerCallback(utils::function(local_cb));
    gui["global_task"].registerCallback(utils::function(global_cb));

    gui["lxp_task"].registerCallback(utils::function(lxp_cb));
    gui["lyp_task"].registerCallback(utils::function(lyp_cb));
    gui["lzp_task"].registerCallback(utils::function(lzp_cb));
    gui["rxp_task"].registerCallback(utils::function(rxp_cb));
    gui["ryp_task"].registerCallback(utils::function(ryp_cb));
    gui["rzp_task"].registerCallback(utils::function(rzp_cb));

    gui["lxn_task"].registerCallback(utils::function(lxn_cb));
    gui["lyn_task"].registerCallback(utils::function(lyn_cb));
    gui["lzn_task"].registerCallback(utils::function(lzn_cb));
    gui["rxn_task"].registerCallback(utils::function(rxn_cb));
    gui["ryn_task"].registerCallback(utils::function(ryn_cb));
    gui["rzn_task"].registerCallback(utils::function(rzn_cb));

    pm = new ParameterManager("left_arm_param.xml");
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc);
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    task->mt = JOINTS;
    task->mft = GLOBAL;
    kuka_left_arm->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, ac->pm.stiff_ctrlpara.axis_damping);
//    std::cout<<"init part is finished"<<std::endl;
}

int main(int argc, char* argv[])
{
    return ICLApp(argc,argv,"-input|-i(2)",init,run).exec();
}
