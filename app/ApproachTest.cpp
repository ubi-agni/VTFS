
/*
 ============================================================================
 Name        : ApproachTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Kuka LWR self movement
 ============================================================================
 */

#include "ComOkc.h"
#include "KukaLwr.h"
#include "Robot.h"
#include "proactcontroller.h"
#include "kukaselfctrltask.h"
#include "comrsb.h"
#include "Timer.h"
#include "msgcontenttype.h"

//load the parameter which are stored in xml file
#include "boost/property_tree/ptree.hpp"
#include "boost/property_tree/xml_parser.hpp"
#include "boost/foreach.hpp"
#include "CtrlParam.h"
#include "parametermanager.h"
#include <mutex>
#include "RebaType.h"
#include "Util.h"

using namespace rsb;
ComRSB *com_rsb;
ComOkc *com_okc;
Robot *kuka_left_arm;
ActController *ac;
Task *task;
ParameterManager* pm;
RobotState *rs;
//using mutex locking controller ptr while it is switching.
std::mutex mutex_act;
bool StopFlag;


#define newP_x -0.1
#define newP_y 0.4
#define newP_z 0.30

#define newO_x 0.0
#define newO_y M_PI/2;
#define newO_z 0.0;


void moveto_cb(boost::shared_ptr<std::string> data){
    Eigen::Vector3d p;
    Eigen::Vector3d o;
    p.setZero();
    o.setZero();
    p(0) = newP_x;
    p(1) = newP_y;
    p(2) = newP_z;
    o(0) = newO_x;
    o(1) = newO_y;
    o(2) = newO_z;
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = GLOBAL;
    task->set_desired_p_eigen(p);
    task->set_desired_o_ax(o);
    mutex_act.unlock();
    std::cout<<"Approach to predefined point"<<std::endl;
}

void global_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    task->mt = JOINTS;
    task->mft = GLOBAL;
    task->curtaskname.prot = RP_NOCONTROL;
    mutex_act.unlock();
    std::cout<<"Switch to global movment task"<<std::endl;
}

void local_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RP_NOCONTROL;
    mutex_act.unlock();
    std::cout<<"Switch to local movment task"<<std::endl;
}

void lxp_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLXP;
    mutex_act.unlock();
    std::cout<<"Switch to lxp movment task"<<std::endl;
}
void lyp_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLYP;
    mutex_act.unlock();
    std::cout<<"Switch to lyp movment task"<<std::endl;
}
void lzp_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLZP;
    mutex_act.unlock();
    std::cout<<"Switch to lzp movment task"<<std::endl;
}
void rxp_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRXP;
    mutex_act.unlock();
    std::cout<<"Switch to rxp movment task"<<std::endl;
}
void ryp_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRYP;
    mutex_act.unlock();
    std::cout<<"Switch to ryp movment task"<<std::endl;
}
void rzp_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRZP;
    mutex_act.unlock();
    std::cout<<"Switch to rzp movment task"<<std::endl;
}
void lxn_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLXN;
    mutex_act.unlock();
    std::cout<<"Switch to lxn movment task"<<std::endl;
}
void lyn_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLYN;
    mutex_act.unlock();
    std::cout<<"Switch to lyn movment task"<<std::endl;
}
void lzn_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RLZN;
    mutex_act.unlock();
    std::cout<<"Switch to lzn movment task"<<std::endl;
}
void rxn_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRXN;
    mutex_act.unlock();
    std::cout<<"Switch to rxn movment task"<<std::endl;
}
void ryn_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRYN;
    mutex_act.unlock();
    std::cout<<"Switch to ryn movment task"<<std::endl;
}
void rzn_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RRZN;
    mutex_act.unlock();
    std::cout<<"Switch to rzn movment task"<<std::endl;
}

void stop_cb(boost::shared_ptr<std::string> data){
    mutex_act.lock();
    ac->set_init_TM(kuka_left_arm->get_cur_cart_o());
    task->mt = JOINTS;
    task->mft = LOCAL;
    task->curtaskname.prot = RP_NOCONTROL;
    mutex_act.unlock();
    std::cout<<"Switch to stop movment task"<<std::endl;
}

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
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
        ac->llv.setZero();
        ac->lov.setZero();
        com_okc->controller_update = true;
        com_okc->data_available = false;
    }
}

int main(int argc, char** argv) {
    //define cb function
    boost::function<void(boost::shared_ptr<std::string>)> button_moveto(moveto_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_localcb(local_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_globalcb(global_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_lxp(lxp_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_lyp(lyp_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_lzp(lzp_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_rxp(rxp_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_ryp(ryp_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_rzp(rzp_cb);

    boost::function<void(boost::shared_ptr<std::string>)> button_lxn(lxn_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_lyn(lyn_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_lzn(lzn_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_rxn(rxn_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_ryn(ryn_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_rzn(rzn_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_stop(stop_cb);
    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);

    std::string selfpath = get_selfpath();
    std::string config_filename = selfpath + "/etc/left_arm_mid_param.xml";
    std::cout<<"config file name is: "<<config_filename<<std::endl;
    if(is_file_exist(config_filename.c_str()) == false){
        config_filename = "left_arm_mid_param.xml";
        if(is_file_exist(config_filename.c_str()) == false){
            std::cout<<"not find the kuka controller configure file"<<std::endl;
            exit(0);
        }
    }

    StopFlag = false;
    pm = new ParameterManager(config_filename);
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
    //register cb function
    com_rsb->register_external("/foo/moveto",button_moveto);
    com_rsb->register_external("/foo/localcb",button_localcb);
    com_rsb->register_external("/foo/globalcb",button_globalcb);
    com_rsb->register_external("/foo/lxp",button_lxp);
    com_rsb->register_external("/foo/lyp",button_lyp);
    com_rsb->register_external("/foo/lzp",button_lzp);
    com_rsb->register_external("/foo/rxp",button_rxp);
    com_rsb->register_external("/foo/ryp",button_ryp);
    com_rsb->register_external("/foo/rzp",button_rzp);

    com_rsb->register_external("/foo/lxn",button_lxn);
    com_rsb->register_external("/foo/lyn",button_lyn);
    com_rsb->register_external("/foo/lzn",button_lzn);
    com_rsb->register_external("/foo/rxn",button_rxn);
    com_rsb->register_external("/foo/ryn",button_ryn);
    com_rsb->register_external("/foo/rzn",button_rzn);
    com_rsb->register_external("/foo/stopmovement",button_stop);
    com_rsb->register_external("/foo/closeprog",button_closeprog);

    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(1));
    thrd_getMsg.start(true);

    //main thread is hanging
    while(!StopFlag){
    }


    //stop robot components
    //todo

    //stop the timer
    thrd_getMsg.stop();
    return 0;
}
