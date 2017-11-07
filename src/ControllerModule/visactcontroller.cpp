#include "visactcontroller.h"
#include <cstddef> //for std::ptrdiff_t;

VisActController::VisActController(ParameterManager &p) : ActController(p)
{
    initVisServoCtrlParam(NV_VERT_ALIGN);
    llv_vis.setZero();
    lov_vis.setZero();
    lv_vis.setZero(6);
    counter = 0;
}

void VisActController::set_pm(ParameterManager &p){
    pm = p;
    std::cout<<"you are update pm in proproception controller"<<std::endl;
}



void VisActController::update_controller_para_stiffness(){
}


void VisActController::initVisServoCtrlParam(VISTaskNameT tnt){
    Kpp[tnt].setZero(6, 6);
    vsm[tnt].setZero(6, 6);
    Kop[tnt].setIdentity(3,3);
    Kpp[tnt] = pm.vis_task_ctrl_param[tnt].kpp;
    Kop[tnt] = pm.vis_task_ctrl_param[tnt].kop;
    vsm[tnt] = pm.vis_task_ctrl_param[tnt].vsm;
}

void VisActController::updateVisServoCtrlParam(VISTaskNameT tnt){
    Kpp[tnt] = pm.vis_task_ctrl_param[tnt].kpp;
    Kop[tnt] = pm.vis_task_ctrl_param[tnt].kop;
    vsm[tnt] = pm.vis_task_ctrl_param[tnt].vsm;
}

void VisActController::get_desired_lv(Robot *robot, Task *t,Eigen::Vector3d nv_v,Eigen::Vector3d tip_nv, RobotState* rs){
    Eigen::VectorXd identity_v;
    Eigen::Vector3d tmp_rot;
    tmp_rot.setZero();
    identity_v.setOnes(6);
    VisuoServoTask tst(t->curtaskname.vist);
    tst = *(VisuoServoTask*)t;
//     std::cout<<"kop "<<tst.curtaskname.vist<<std::endl;
// //    std::cout<<"old vist"<<t->curtaskname.vist<<std::endl;
//     std::cout<<Kop[tst.curtaskname.vist]<<std::endl;
//     std::cout<<std::endl;
    lv_vis.head(3).setZero();
    lv_vis.tail(3).setZero();
    counter++;
    if(counter >=100){
        std::cout<<"nv_v "<<nv_v(0)<<","<<nv_v(1)<<","<<nv_v(2)<<std::endl;
        std::cout<<"tip_nv "<<tip_nv(0)<<","<<tip_nv(1)<<","<<tip_nv(2)<<std::endl;
        std::cout<<"dot product "<<nv_v.dot(tip_nv)<<std::endl;
        counter = 0;
    }
    if(nv_v.dot(tip_nv) < 0){
        //fingertip should rotate from tip_nv to surface_nv
        global2local(tip_nv.cross(nv_v),\
                     rs->robot_orien["eef"],tmp_rot);
    }else{
        //fingertip should rotate from surface_nv to tip_nv
        global2local(nv_v.cross(tip_nv),\
                     rs->robot_orien["eef"],tmp_rot);
    }
//     lv_vis(5) = -0.1*nv_v.dot(tip_nv);
    lv_vis.tail(3) = 0.02* tmp_rot;
    
    llv_vis = lv_vis.head(3);
    lov_vis = Kop[tst.curtaskname.vist] * lv_vis.tail(3);
    limit_vel(get_llv_limit(),llv_vis,lov_vis);
}

void VisActController::update_robot_reference(Robot *robot, Task *t,Eigen::Vector3d nv_v,Eigen::Vector3d tip_nv, RobotState* rs){
    Eigen::Vector3d o_target,p_target;
    p_target.setZero();
    o_target.setZero();
    get_desired_lv(robot,t,nv_v,tip_nv,rs);
    llv = llv + llv_vis;
    lov = lov + lov_vis;
//     std::cout<<"lov: "<<lov(0)<<","<<lov(1)<<","<<lov(2)<<std::endl;
    local_to_global(robot->get_cur_cart_p(),robot->get_cur_cart_o(),llv,\
                    lov,p_target,o_target);
    for (int i=0; i < 3; i++){
        cart_command[i] = p_target(i);
        cart_command[i+3] = o_target(i);
    }
    for(int i = 0; i < 6; i++)
        robot->set_cart_command(cart_command);
}


void VisActController::get_joint_position(){
}

void VisActController::get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov){
    lv = llv_vis;
    ov = lov_vis;
}
