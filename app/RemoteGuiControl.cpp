/*
 ============================================================================
 Name        : RemoteGuiControl.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Using remote created icl gui control kuka movement example
 ============================================================================
 */
#include <stdlib.h>
#include <iostream>
#include <mutex>

#include "Timer.h"
#include "comrsb.h"
#include "msgcontenttype.h"
#include "ComOkc.h"
#include "KukaLwr.h"
#include "Robot.h"
#include "proactcontroller.h"

#define newP_x -0.1
#define newP_y 0.4
#define newP_z 0.30

#define newO_x 0.0
#define newO_y M_PI/2;
#define newO_z 0.0;
bool StopFlag;

using namespace rsb;
ComRSB *com_rsb;
ComOkc *com_okc;
Robot *kuka_left_arm;
ActController *ac;
ParameterManager* pm;
Task *task;
RobotState *rs;
std::mutex mutex_act;

void function_slider(boost::shared_ptr<std::string> data) {
     std::cout<<"slide "<<*data<<std::endl;

}

void function_button(boost::shared_ptr<std::string> data) {
    Eigen::Vector3d p;
    Eigen::Vector3d o;
    p.setZero();
    o.setZero();
    std::cout<<"robot self movement and move to new pose"<<std::endl;
    std::cout<<"click botton "<<*data<<std::endl;
    p(0) = newP_x;
    p(1) = newP_y;
    p(2) = newP_z+0.2;
    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = GLOBAL;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    mutex_act.unlock();

}

void stop_button_cb(boost::shared_ptr<std::string> data) {
    std::cout<<"click stop botton "<<*data<<std::endl;
    StopFlag = true;

}

void run(){
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){
        kuka_left_arm->get_joint_position_act();
        kuka_left_arm->update_robot_state();
        //using controller to update the reference
        ac->update_robot_reference(kuka_left_arm,task);
        kuka_left_arm->update_cbf_controller();
        kuka_left_arm->set_joint_command();
        com_okc->controller_update = true;
        com_okc->data_available = false;
    }
}

int main(int argc, char** argv) {
    StopFlag = false;
    //define the callback function
    boost::function<void(boost::shared_ptr<std::string>)> fun_slider(function_slider);
    boost::function<void(boost::shared_ptr<std::string>)> fun_button(function_button);
    boost::function<void(boost::shared_ptr<std::string>)> stop_button(stop_button_cb);

    pm = new ParameterManager("left_arm_param.xml");
    com_okc = new ComOkc(kuka_left,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_left_arm = new KukaLwr(kuka_left,*com_okc);
    std::cout<<"init part is finished"<<std::endl;
    ac = new ProActController(*pm);

    rs = new RobotState(kuka_left_arm);

    Eigen::Vector3d p_left,o_left,p_right,o_right;
    p_left.setZero();
    o_left.setZero();
    p_right.setZero();
    o_right.setZero();

    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    task->mt = JOINTS;
    task->mft = GLOBAL;
    p_left(0) = newP_x;
    p_left(1) = newP_y;
    p_left(2) = newP_z;
    o_left(0) = newO_x;
    o_left(1) = newO_y;
    o_left(2) = newO_z;
    task->set_desired_p_eigen(p_left);
    task->set_desired_o_ax(o_left);
    kuka_left_arm->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, \
                                           ac->pm.stiff_ctrlpara.axis_damping);

    com_rsb = new ComRSB();
    //register callbackfunction
    com_rsb->register_external("/foo/impbutton",fun_button);
    com_rsb->register_external("/foo/slider",fun_slider);
    com_rsb->register_external("/foo/stopbutton",stop_button);
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(1));
    thrd_getMsg.start(true);

    while(!StopFlag){
    }
    thrd_getMsg.stop();
    return 1;
}
