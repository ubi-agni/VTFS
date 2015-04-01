

/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Kuka Cartesian impedance mode for kuka movement
 ============================================================================
 */


#include <ICLQt/Common.h>
#include <ICLUtils/Random.h>
#include <ICLUtils/FPSLimiter.h>
#include <UtilModule/VisTool.h>

#include <iostream>
#include <thread>
#include <unistd.h>
#include <termios.h>

#include "ComModule/comrsb.h"
#include "ComModule/ComOkc.h"
#include "RobotModule/KukaLwr.h"
#include "RobotModule/Robot.h"
#include "ControllerModule/proactcontroller.h"
#include "TaskModule/kukaselfctrltask.h"
#include "ControllerModule/tacservocontroller.h"
#include "TaskModule/tacservotask.h"
//#include "ControllerModule/CtrlParam.h"
#include "UtilModule/Timer.h"
#include <fstream>
#include "UtilModule/Util.h"
#include "RobotModule/RobotState.h"

std::ofstream stiffness_data;
ComOkc *com_okc;
Robot *kuka_lwr;
ActController *ac;
Task *task;
TaskNameT taskname;
ParameterManager* pm;
RobotState *kuka_lwr_rs;

GUI gui;
VisTool *visTool = 0;


#ifdef DJALLIL_CONF
#define newP_x 0.28
#define newP_y 0.5
#define newP_z 0.50

#define newO_x 0.0
#define newO_y 0.0
#define newO_z 0.0;
#else
#define newP_x 0.2
#define newP_y 0.3
#define newP_z 0.30
#define newO_x 0.0
#define newO_y -M_PI/2;
#define newO_z 0.0;
#endif

#define SAMPLEFREQUENCE 4

char inp;

RobotModeT rmt;

bool stiffflag,startflag;

double t_t;



void moveto_cb(void){
    Eigen::Vector3d p,o;
    p.setZero();
    o.setZero();
    p(0) = newP_x-0.1;
    p(1) = newP_y;
    p(2) = newP_z;

    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    delete ac;
    delete task;
    for(int i = 0; i < 7; i++){
        pm->stiff_ctrlpara.axis_stiffness[i] = 1000;
        pm->stiff_ctrlpara.axis_damping[i] = 0.7;
    }
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    task->mt = JOINTS;
    task->mft = GLOBAL;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    rmt = NormalMode;
    startflag = false;
    //    std::cout<<"robot self movement and move to new pose"<<std::endl;
}

void movexyz_cb(){
    Eigen::Vector3d cp, p, o;
    double x,y,z;

    cp.setZero();
    p.setZero();
    o.setZero();

    cp = kuka_lwr->get_cur_cart_p();
    o = tm2axisangle(kuka_lwr->get_cur_cart_o());

    //todo init xyz by slider.
    x = gui["tx"];
    y = gui["ty"];
    z = gui["tz"];
    p(0) = cp(0) + x/1000.0;
    p(1) = cp(1) + y/1000.0;
    p(2) = cp(2) + z/1000.0;

    delete ac;
    delete task;
    for(int i = 0; i < 7; i++){
        pm->stiff_ctrlpara.axis_stiffness[i] = 1000;
        pm->stiff_ctrlpara.axis_damping[i] = 0.7;
    }
    //std::cout<<"change the stiffness"<<std::endl;
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    task->mt = JOINTS;
    task->mft = LOCALP2P;
    task->velocity_p2p(0) = x/1000.0;
    task->velocity_p2p(1) = y/1000.0;
    task->velocity_p2p(2) = z/1000.0;
    task->set_initial_p_eigen(cp);
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    rmt = NormalMode;
    //    std::cout<<"robot self movement and move to new pose"<<std::endl;
}

void psudog_cb(void){
    rmt = PsudoGravityCompensation;
}

void print_pf(void){
    Eigen::Vector3d p,f,t;
    Eigen::Matrix3d o;

    p = kuka_lwr->get_cur_cart_p();
    o = kuka_lwr->get_cur_cart_o();
    kuka_lwr->get_eef_ft(f,t);
    //    std::cout<<"position "<<p[0]<<","<<p[1]<<","<<p[2]<<std::endl;
    //    std::cout<<"orientation "<<std::endl;std::cout<<o<<std::endl;
    //    std::cout<<"force "<<f[0]<<","<<f[1]<<","<<f[2]<<std::endl;
    //    std::cout<<"torque "<<t[0]<<","<<t[1]<<","<<t[2]<<std::endl;
}
Point32f curr(10,50);
Eigen::Vector3d initP;

void scart_cb(void){
    //    Eigen::VectorXd cp_stiff,cp_damping,extft;
    com_okc->start_brake();
    //    switch_stiff_cb1();
    kuka_lwr->switch2cpcontrol();
    com_okc->release_brake();
    stiffflag = true;
    visTool->setPropertyValue("pos.curr",curr);
//    initP[0] = kuka_lwr->pose_frombase[3];
//    initP[1] = kuka_lwr->pose_frombase[7];
//    initP[2] = kuka_lwr->pose_frombase[11];
    initP.setZero();
    initP = kuka_lwr_rs->robot_position["eef"];
    startflag = true;
    //    cp_stiff.setZero(6);
    //    cp_damping.setZero(6);
    //    extft.setZero(6);
    //    cp_stiff[0] = 2000;
    //    cp_stiff[1] = 200;
    //    cp_stiff[2] = 2000;
    //    cp_stiff[3] = 200;
    //    cp_stiff[4] = 200;
    //    cp_stiff[5] = 200;
    //    cp_damping[0] = 0.7;
    //    cp_damping[1] = 0.7;
    //    cp_damping[2] = 0.7;
    //    cp_damping[3] = 0.7;
    //    cp_damping[4] = 0.7;
    //    cp_damping[5] = 0.7;
    //    extft[1] = 20;
    //    kuka_lwr->update_robot_cp_stiffness(cp_stiff,cp_damping);
    //    kuka_lwr->update_robot_cp_exttcpft(extft);
    //    std::cout<<"change stiffness"<<std::endl;
}

void sjnt_cb(void){
    //    Eigen::VectorXd cp_stiff,cp_damping,extft;
    com_okc->start_brake();
    stiffflag = false;
    //    switch_stiff_cb1();
    kuka_lwr->switch2jntcontrol();
    com_okc->release_brake();

}




void run_ctrl(){
//    if(pa("-no-robot")){
//        Thread::msleep(1000);
//        return;
//    }
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){

        //        //kuka_lwr->update_robot_stiffness(pm);
        Eigen::VectorXd cp_stiff,cp_damping,extft;
        Eigen::Vector3d vel;
        vel.setZero();
        extft.setZero(6);
        kuka_lwr_rs->updated(kuka_lwr);
        kuka_lwr->get_joint_position_act();
        kuka_lwr->get_joint_position_mea();
        kuka_lwr->update_robot_state();
        vel = kuka_lwr->get_cur_vel();
        if(stiffflag ==true){
            cp_stiff.setZero(6);
            cp_damping.setZero(6);
            cp_stiff[0] = 2000;
            cp_stiff[1] = 200;
            cp_stiff[2] = 0;
            cp_stiff[3] = 200;
            cp_stiff[4] = 200;
            cp_stiff[5] = 200;
            cp_damping[0] = 0.7;
            cp_damping[1] = 0.7;
            cp_damping[2] = 0.7;
            cp_damping[3] = 0.7;
            cp_damping[4] = 0.7;
            cp_damping[5] = 0.7;
            if(fabs(vel[0])>0.02)
                extft[1] = 20*vel[0];
            else{
                extft[1] = 0;
            }
            t_t = t_t + 0.01;
            extft[1] = 20*sin(t_t);
            if(t_t>6.28){t_t = 0.0;}
//            extft[1] = 0;
            //            std::cout<<"vel and force "<<vel[0]<<","<<extft[1]<<std::endl;
            kuka_lwr->update_robot_cp_stiffness(cp_stiff,cp_damping);
            kuka_lwr->update_robot_cp_exttcpft(extft);
        }
        //using all kinds of controllers to update the reference
        if(task->mt == JOINTS)
            ac->update_robot_reference(kuka_lwr,task);
        ac->llv.setZero();
        ac->lov.setZero();
        //use CBF to compute the desired joint angle rate
        kuka_lwr->update_cbf_controller();
        kuka_lwr->set_joint_command(rmt);
        com_okc->controller_update = true;

    }
}




void start_cb(){


}

void run_vis(){
//  if(!pa("-run-test")) {
//    Thread::msleep(1000);
//    return;
//  }

  
  if(startflag == true){
      Eigen::Vector3d tmp_p;
      tmp_p.setZero();
      tmp_p = kuka_lwr_rs->robot_position["eef"]-initP;
        curr.x = (-1)*(tmp_p[0])*1000*((double)pa("-g",0)-(double)pa("-s",0)) \
                /(float)gui["tbar"];
        curr.y = (double)pa("-g",1) + (tmp_p[1])*1000*((double)pa("-g",0)-(double)pa("-s",0))/2.0 \
                /(float)gui["tbar"];
        std::cout<<"x,y "<<curr.x<<","<<curr.y<<std::endl;
        //      curr.x += 1;
//      curr.y += gaussRandom(0,2);
//      if(curr.y > 80) curr.y *= 0.9;
//      if(curr.y < 20) curr.y *= 1.1;

      visTool->setPropertyValue("pos.curr",curr);
  }
  static FPSLimiter limiter(30);
  limiter.wait();
}

void init(){
//    if(!pa("-no-robot")){}
    pm = new ParameterManager("right_arm_param.xml");
    com_okc = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_lwr = new KukaLwr(kuka_right,*com_okc);
    kuka_lwr_rs = new RobotState(kuka_lwr);
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
    kuka_lwr->setAxisStiffnessDamping(ac->pm.stiff_ctrlpara.axis_stiffness, \
                                      ac->pm.stiff_ctrlpara.axis_damping);
    rmt = NormalMode;
    //tHello.setSingleShot(false);
    //tHello.setInterval(Timer::Interval(SAMPLEFREQUENCE));
    //tHello.start(true);
    stiffflag = false;
    startflag = false;
    t_t = 0.0;


    gui << VBox().handle("box")
        <<(VBox()
         << Button("moveto").handle("moveto_task")
         <<(HBox()
          << Slider(-500,500,100,false).label("x_move(mm)").handle("tx")
          << Slider(-500,500,100,false).label("y_move(mm)").handle("ty")
          << Slider(-500,500,100,false).label("z_move(mm)").handle("tz")
             )
         << Button("movexyz").handle("movexyz_task")
         << Button("psudoG").handle("psudog_task")
         << Button("sJNT").handle("sjnt_task")
         << Button("sCART").handle("scart_task")
         << Button("start").handle("start_task")
         << Slider(0,500,100,false).label("bar_move(mm)").handle("tbar")

         )
        <<  Show();
    gui["moveto_task"].registerCallback(utils::function(moveto_cb));
    gui["movexyz_task"].registerCallback(utils::function(movexyz_cb));
    gui["psudog_task"].registerCallback(utils::function(psudog_cb));
    gui["sjnt_task"].registerCallback(utils::function(sjnt_cb));
    gui["scart_task"].registerCallback(utils::function(scart_cb));
    gui["start_task"].registerCallback(utils::function(start_cb));
    visTool = new VisTool;
    BoxHandle box = gui["box"];
    box.add(visTool->getRootWidget(),"myVisTool");

    float maxX = pa("-v",0), maxY = pa("-v",1);
    
    if(pa("-props")){
      visTool->loadProperties(pa("-props"));
    }
    
    visTool->setPropertyValue("viewport.maxX",maxX);
    visTool->setPropertyValue("viewport.maxY",maxY);
    
    visTool->setPropertyValue("pos.left",Point32f(pa("-s",0), pa("-s",1)));
    visTool->setPropertyValue("pos.right",Point32f(pa("-g",0), pa("-g",1)));
    
    visTool->setPropertyValue("margin",pa("-margin").as<float>());

    visTool->setPropertyValue("sizes.trace",2);
    gui.getRootWidget()->setGeometry(QRect(pa("-geom",0),
                                           pa("-geom",1),
                                           pa("-geom",2),
                                           pa("-geom",3)));
    if(pa("-record")){
      visTool->getWidget()->startRecording(*pa("-record",0), *pa("-record",1));
    }

}

int main(int n, char **args){
    return ICLApp(n,args,"-viewport|-v(w=100,h=100) "
                  "-start-pos|-s(x=10,y=50) -goal-pos|-g(x=90,y=50) "
                  "-margin|-m(margin=30) -props(xmlfile) -record(device,info) "
                  "-window-geomery|-geom(x=100,y=100,w=800,h=400)",init,run_ctrl, run_vis).exec();
}
