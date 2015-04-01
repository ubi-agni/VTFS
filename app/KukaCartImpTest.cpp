
/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Kuka Cartesian impedance mode for kuka movement
 ============================================================================
 */


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

std::ofstream stiffness_data;
ComOkc *com_okc;
Robot *kuka_lwr;
ActController *ac;
Task *task;
TaskNameT taskname;
ParameterManager* pm;
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

bool stiffflag;

double t_t;

int getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

//The function we want to make the thread run.
void keypresscap(void)
{	char inp_tmp;
    while(inp != 'e'){
        inp_tmp = getch();
            if ( inp_tmp != '\n' ) {
            inp = inp_tmp;
//            std::cout<<"in sub task loop, you input char "<<inp<<std::endl;
            }
    }
}

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
    std::cout<<"change the stiffness"<<std::endl;
    ac = new ProActController(*pm);
    task = new KukaSelfCtrlTask(RP_NOCONTROL);
    task->mt = JOINTS;
    task->mft = GLOBAL;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    rmt = NormalMode;
//    std::cout<<"robot self movement and move to new pose"<<std::endl;
}

void movein_xyz(float x, float y, float z){
    Eigen::Vector3d cp, p, o;

    cp.setZero();
    p.setZero();
    o.setZero();

    cp = kuka_lwr->get_cur_cart_p();
    o = tm2axisangle(kuka_lwr->get_cur_cart_o());

    p(0) = cp(0) + x;
    p(1) = cp(1) + y;
    p(2) = cp(2) + z;

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
    task->velocity_p2p(0) = x/1.0;
    task->velocity_p2p(1) = y/1.0;
    task->velocity_p2p(2) = z/1.0;
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


void switch_stiff_cb(void){
//    Eigen::VectorXd cp_stiff,cp_damping,extft;
    com_okc->start_brake();
//    switch_stiff_cb1();
    kuka_lwr->switch2cpcontrol();
    com_okc->release_brake();
    stiffflag = true;
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

void switch_jnt_cb(void){
//    Eigen::VectorXd cp_stiff,cp_damping,extft;
    com_okc->start_brake();
    stiffflag = false;
//    switch_stiff_cb1();
    kuka_lwr->switch2jntcontrol();
    com_okc->release_brake();

}

Timer tHello([]()
{
//    Eigen::Vector3d p,f,t;
//    Eigen::Matrix3d o;

//    p = kuka_lwr->get_cur_cart_p();
//    o = kuka_lwr->get_cur_cart_o();
//    kuka_lwr->get_eef_ft(f,t);

//    Eigen::MatrixXd Kcp;
//    Eigen::Matrix<double, 6, 7> J_eigen;
//    Kcp.setZero(6,6);
//    Kcp(0,0) = 0.01;
//    Kcp(1,1) = 5000;
//    Kcp(2,2) = 5000;
//    Kcp(3,3) = 300;
//    Kcp(4,4) = 300;
//    Kcp(5,5) = 300;

//    Eigen::MatrixXd K_axis;
//    K_axis.setZero(7,7);


//    J_eigen = kuka_lwr->Jac_kdl.data;
//    K_axis = J_eigen.transpose() * Kcp * J_eigen;

//    for(int i = 0; i < 7; i++){
//        if(K_axis(i,i)>2000) K_axis(i,i) = 2000;
//        if(K_axis(i,i)<0.01) K_axis(i,i) = 0.01;
//        pm->stiff_ctrlpara.axis_stiffness[i] = K_axis(i,i);
//        pm->stiff_ctrlpara.axis_damping[i] = 0.7;
//    }
//    stiffness_data<<K_axis(0,0)<<","<<K_axis(1,1)<<","<<K_axis(2,2)<<","<<K_axis(3,3)<<","<<K_axis(4,4)<<","<<K_axis(5,5)<<std::endl;
//    std::cout<<"change the stiffness"<<std::endl;
//    std::cout<<"stiffness are "<<std::endl;std::cout<<K_axis<<std::endl;
//    std::cout<<"Jacobian are "<<std::endl;std::cout<<J_eigen<<std::endl;
});


void run(){
    //only call for this function, the ->jnt_position_act is updated
    if((com_okc->data_available == true)&&(com_okc->controller_update == false)){

//        //kuka_lwr->update_robot_stiffness(pm);
        Eigen::VectorXd cp_stiff,cp_damping,extft;
        Eigen::Vector3d vel;
        vel.setZero();
        extft.setZero(6);
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
                 extft[1] = 0;
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

void init(){
    pm = new ParameterManager("right_arm_param.xml");
    com_okc = new ComOkc(kuka_right,OKC_HOST,OKC_PORT);
    com_okc->connect();
    kuka_lwr = new KukaLwr(kuka_right,*com_okc);
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
    tHello.setSingleShot(false);
    tHello.setInterval(Timer::Interval(SAMPLEFREQUENCE));
    tHello.start(true);
    stiffflag = false;
    t_t = 0.0;
}

int main(int argc, char* argv[])
{

    int sinCounter = 0;
    float teta = 0;
    bool sinOn = false;
    double step = 0.1;
    std::thread t1(keypresscap);
    stiffness_data.open("/tmp/stiff.txt");
    inp = 'f';
    init();
    while(inp != 'e' && inp != EOF){
        switch (inp){

        case 'g':
            psudog_cb();
            inp = '\n';
            break;
        case 's':
            switch_stiff_cb();
            inp = '\n';
            break;
        case 'j':
            switch_jnt_cb();
            inp = '\n';
            break;
        case 'A':
           movein_xyz(0.0, 0.0, 0.1);
           inp = '\n';
           break;
        case 'B':
           movein_xyz(0.0, 0.0, -0.1);
           inp = '\n';
           break;
        case 'C':
            movein_xyz(-0.1, 0.0, 0.0);
            inp = '\n';
            break;
        case 'D':
            movein_xyz(0.1, 0.0, 0.0);
            inp = '\n';
            break;
        case 'l':
            sinOn = !sinOn;
            inp = '\n';
            break;
        case '\n':
            break;
        default:

            break;
        }
        if(sinOn){
            sinCounter = sinCounter + 1;
            if (sinCounter == 250){
                teta = teta + 0.01;
                movein_xyz(-0.1 * sin(teta), 0.0, 0.0);
                sinCounter = 0;
                //std::cout<<"teta = "<< sin(teta)<<std::endl;
            }
        }
        run();
        usleep(20);
}
    std::cout<<"main function is end "<<std::endl;
    tHello.stop();
    t1.join();
    std::cout<<"keypress thread is end "<<std::endl;
}

