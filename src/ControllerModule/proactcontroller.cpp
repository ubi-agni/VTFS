#include "proactcontroller.h"
#include <cstddef> //for std::ptrdiff_t;

ProActController::ProActController(ParameterManager &p) : ActController(p)
{
    initProServoCtrlParam(RLXP);
    initProServoCtrlParam(RLYP);
    initProServoCtrlParam(RLZP);
    initProServoCtrlParam(RRXP);
    initProServoCtrlParam(RRYP);
    initProServoCtrlParam(RRZP);
    initProServoCtrlParam(RLXN);
    initProServoCtrlParam(RLYN);
    initProServoCtrlParam(RLZN);
    initProServoCtrlParam(RRXN);
    initProServoCtrlParam(RRYN);
    initProServoCtrlParam(RRZN);
    initProServoCtrlParam(RP_NOCONTROL);
    initProServoCtrlParam(RP_LINEFOLLOW);
    initProServoCtrlParam(RP_ROTATEFOLLOW);
    initProServoCtrlParam(RP_BOTHFOLLOW);
    llv_pro.setZero();
    lov_pro.setZero();
    lv_pro.setZero(6);
    delta_ag = 0; 
    delta_ag_int = 0;
}

void ProActController::set_pm(ParameterManager &p){
    pm = p;
    std::cout<<"you are update pm in proproception controller"<<std::endl;
}


void ProActController::update_controller_para(Eigen::Vector3d vel,PROTaskNameT tnt){
    for(int i = 0; i < 3; i++)
        Kpp[tnt](i,i) = vel(i);
}

void ProActController::update_controller_para(std::pair<Eigen::Vector3d,double>& r_ax,PROTaskNameT tnt){
    std::ptrdiff_t index;
    Eigen::Vector3d ax;
    ax.setZero();
    ax = r_ax.first;
    for(int i = 0; i < 3; i++)
          ax(i) = fabs(ax(i));
    ax.maxCoeff(&index);
    for(int i = 3; i < 6; i++)
        Kpp[tnt](i,i) = 0.0;
    if(r_ax.first(index)>0)
        Kpp[tnt](index+3,index+3) = r_ax.second;
    else
        Kpp[tnt](index+3,index+3) = (-1) * r_ax.second;
    std::cout<<"coe......................................"<<Kpp[tnt](3,3)<<","<<Kpp[tnt](4,4)<<","<<Kpp[tnt](5,5)<<","<<index+3<<std::endl;
}

void ProActController::update_controller_para(Eigen::Vector3d vel,Eigen::Vector3d r_vel,PROTaskNameT tnt){
    for(int i = 0; i < 3; i++){
        Kpp[tnt](i,i) = vel(i);
        Kpp[tnt](i+3,i+3) = r_vel(i);
    }
}


void ProActController::update_controller_para_stiffness(){
}


void ProActController::initProServoCtrlParam(PROTaskNameT tnt){
    Kpp[tnt].setZero(6, 6);
    psm[tnt].setZero(6, 6);
    Kop[tnt].setIdentity(3,3);
    Kpp[tnt] = pm.pro_task_ctrl_param[tnt].kpp;
    psm[tnt] = pm.pro_task_ctrl_param[tnt].psm;
}

void ProActController::updateProServoCtrlParam(PROTaskNameT tnt){
    Kpp[tnt] = pm.pro_task_ctrl_param[tnt].kpp;
    psm[tnt] = pm.pro_task_ctrl_param[tnt].psm;
}

void ProActController::get_desired_lv(Robot *robot, Task *t){
    Eigen::VectorXd identity_v;
    identity_v.setOnes(6);
    KukaSelfCtrlTask tst(t->curtaskname.prot);
    tst = *(KukaSelfCtrlTask*)t;
//    std::cout<<"kpp and psm"<<tst.curtaskname.prot<<std::endl;
//    std::cout<<"old prot"<<t->curtaskname.prot<<std::endl;
//    std::cout<<Kpp[tst.curtaskname.prot]<<std::endl;
//    std::cout<<std::endl;
    lv_pro = Kpp[tst.curtaskname.prot] * psm[tst.curtaskname.prot] * identity_v;
    llv_pro = lv_pro.head(3);
    lov_pro = Kop[tst.curtaskname.prot] * lv_pro.tail(3);
    limit_vel(get_llv_limit(),llv_pro,lov_pro);
}

void ProActController::update_robot_reference(Robot *robot){
    Eigen::Vector3d p_cur,p_target;
    Eigen::Vector3d o_cur,o_target;
    p_cur.setZero();
    o_cur.setZero();
    p_cur = robot->get_cur_cart_p();
    o_cur = tm2axisangle(robot->get_cur_cart_o());
    p_target = p_cur;
    o_target = o_cur;
    for (int i=0; i < 3; i++){
        cart_command[i] = p_target(i);
        cart_command[i+3] = o_target(i);
    }
    for(int i = 0; i < 6; i++)
        robot->set_cart_command(cart_command);
}

void ProActController::get_desired_lv(Robot *robot, Task *t,Eigen::Vector3d cur_dir,RobotState* rs){
	Eigen::VectorXd identity_v;
    identity_v.setOnes(6);
    Eigen::Vector3d tmp_rot_axis;
    tmp_rot_axis.setZero();
    Eigen::Vector3d normalized_cur_dir;
    normalized_cur_dir = cur_dir.normalized();
    KukaSelfCtrlTask tst(t->curtaskname.prot);
    tst = *(KukaSelfCtrlTask*)t;
//    std::cout<<"kpp and psm"<<tst.curtaskname.prot<<std::endl;
//    std::cout<<"old prot"<<t->curtaskname.prot<<std::endl;
//    std::cout<<"Kpp "<<Kpp[tst.curtaskname.prot]<<std::endl;
//    std::cout<<"Kop "<<Kop[tst.curtaskname.prot]<<std::endl;
//    std::cout<<std::endl;

    //test for one axis
    identity_v(0) = 0;
    identity_v(2) = 0;
    Eigen::Matrix3d Rel;
    Rel.setIdentity();
    Rel = rs->robot_orien["eef"].transpose() * tst.contact_frame;
    std::cout<<"contact frame is "<<std::endl;
    std::cout<<tst.contact_frame<<std::endl;
    
    lv_pro.head(3) = Kpp[tst.curtaskname.prot].block(0,0,3,3) * psm[tst.curtaskname.prot].block(0,0,3,3) * Rel * identity_v.head(3);
    std::cout<<"local motion in proprioception "<<lv_pro(0)<<","<<lv_pro(1)<<","<<lv_pro(2)<<std::endl;
    std::cout<<"c to g "<<std::endl;
    std::cout<<tst.contact_frame * identity_v.head(3)<<std::endl;
    std::cout<<"c to e "<<std::endl;
    std::cout<<Rel * identity_v.head(3)<<std::endl;
    llv_pro = lv_pro.head(3);
    global2local(normalized_cur_dir.cross(tst.desired_axis_dir),\
                     rs->robot_orien["eef"],tmp_rot_axis);
    delta_ag = (1.0-fabs(normalized_cur_dir.dot(tst.desired_axis_dir)));
    delta_ag_int =  delta_ag_int + delta_ag;                
    lv_pro.tail(3) = delta_ag * tmp_rot_axis + 0.001 * delta_ag_int * tmp_rot_axis;
//    std::cout<<"local rotation axis is "<<lv_pro(3)<<","<<lv_pro(4)<<","<<lv_pro(5)<<std::endl;
    lov_pro = Kop[tst.curtaskname.prot] * lv_pro.tail(3);
    limit_vel(get_llv_limit(),llv_pro,lov_pro);
}


void ProActController::update_robot_reference(Robot *robot, Task *t, Eigen::Vector3d cur_dir, RobotState* rs){
    Eigen::Vector3d p_target,o_target;
    p_target.setZero();
    o_target.setZero();
    if(t->mft == GLOBAL){
        p_target = t->get_desired_p_eigen();
        o_target = t->get_desired_o_ax();
    }
    if(t->mft == LOCAL){
        get_desired_lv(robot,t,cur_dir,rs);
//        limit_eef_euler(get_euler_limit());
//        std::cout<<"lv before in proservo "<<llv<<std::endl;
//        std::cout<<lov<<std::endl;
        llv = llv + llv_pro;
        lov = lov + lov_pro;
//        std::cout<<"lv after in proservo "<<llv<<std::endl;
//        std::cout<<lov<<std::endl;
        local_to_global(robot->get_cur_cart_p(),robot->get_cur_cart_o(),llv,\
                        lov,p_target,o_target);
    }
    if(t->mft == LOCALP2P){
        //checking the whether moved distance is the same like desired move distance
        if((robot->get_cur_cart_p()-t->get_initial_p_eigen()).norm()<\
            (t->get_desired_p_eigen()-t->get_initial_p_eigen()).norm()){
            p_target = robot->get_cur_cart_p() + t->velocity_p2p;\
            std::cout<<"p cur are "<<robot->get_cur_cart_p()<<std::endl;
            std::cout<<"vel "<<t->velocity_p2p<<std::endl;
        }
        else{
            p_target = robot->get_cur_cart_p();
        }
        std::cout<<"in local p2p mode"<<std::endl;
        o_target = t->get_desired_o_ax();
    }
    for (int i=0; i < 3; i++){
        cart_command[i] = p_target(i);
        cart_command[i+3] = o_target(i);
    }
    for(int i = 0; i < 6; i++)
        robot->set_cart_command(cart_command);
}


void ProActController::update_robot_reference(Robot *robot, Task *t){
    Eigen::Vector3d p_target,o_target;
    p_target.setZero();
    o_target.setZero();
    if(t->mft == GLOBAL){
        p_target = t->get_desired_p_eigen();
        o_target = t->get_desired_o_ax();
    }
    if(t->mft == LOCAL){
        get_desired_lv(robot,t);
//        limit_eef_euler(get_euler_limit());
//        std::cout<<"lv before in proservo "<<llv<<std::endl;
//        std::cout<<lov<<std::endl;
        llv = llv + llv_pro;
        lov = lov + lov_pro;
//        std::cout<<"lv after in proservo "<<llv<<std::endl;
//        std::cout<<lov<<std::endl;
        local_to_global(robot->get_cur_cart_p(),robot->get_cur_cart_o(),llv,\
                        lov,p_target,o_target);
    }
    if(t->mft == LOCALP2P){
        //checking the whether moved distance is the same like desired move distance
        if((robot->get_cur_cart_p()-t->get_initial_p_eigen()).norm()<\
            (t->get_desired_p_eigen()-t->get_initial_p_eigen()).norm()){
            p_target = robot->get_cur_cart_p() + t->velocity_p2p;\
            std::cout<<"p cur are "<<robot->get_cur_cart_p()<<std::endl;
            std::cout<<"vel "<<t->velocity_p2p<<std::endl;
        }
        else{
            p_target = robot->get_cur_cart_p();
        }
        std::cout<<"in local p2p mode"<<std::endl;
        o_target = t->get_desired_o_ax();
    }
    for (int i=0; i < 3; i++){
        cart_command[i] = p_target(i);
        cart_command[i+3] = o_target(i);
    }
    for(int i = 0; i < 6; i++)
        robot->set_cart_command(cart_command);
}



void ProActController::get_joint_position(){
}

void ProActController::get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov){
    lv = llv_pro;
    ov = lov_pro;
}
