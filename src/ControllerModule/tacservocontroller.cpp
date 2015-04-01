#include "tacservocontroller.h"
#include "TaskModule/tacservotask.h"
#include <sys/stat.h>     //create folder for data record

 void TacServoController::initTacServoCtrlParam(TACTaskNameT tnt){
     Kpp[tnt].setZero(6, 6);
     Kpi[tnt].setZero(6, 6);
     Kpd[tnt].setZero(6, 6);
     Kop[tnt].setZero(3, 3);
     sm[tnt].setZero(6, 6);
     tjkm[tnt].setZero(6, 6);
     Kpp[tnt] = pm.tac_task_ctrl_param[tnt].kpp;
     Kpi[tnt] = pm.tac_task_ctrl_param[tnt].kpi;
     Kpd[tnt] = pm.tac_task_ctrl_param[tnt].kpd;
     Kop[tnt] = pm.tac_task_ctrl_param[tnt].kop;
     sm[tnt] = pm.tac_task_ctrl_param[tnt].tsm;
     tjkm[tnt] = pm.tac_task_ctrl_param[tnt].ttjkm;
 }

 void TacServoController::set_pm(ParameterManager &p){
     pm = p;
     std::cout<<"you are update pm in tactile servo controller"<<std::endl;
 }

 void TacServoController::update_controller_para(Eigen::Vector3d, PROTaskNameT){
 }

 void TacServoController::update_controller_para_stiffness(){
 }


 void TacServoController::updateTacServoCtrlParam(TACTaskNameT tnt){
     Kpp[tnt] = pm.tac_task_ctrl_param[tnt].kpp;
     Kpi[tnt] = pm.tac_task_ctrl_param[tnt].kpi;
     Kpd[tnt] = pm.tac_task_ctrl_param[tnt].kpd;
     Kop[tnt] = pm.tac_task_ctrl_param[tnt].kop;
     sm[tnt] = pm.tac_task_ctrl_param[tnt].tsm;
     tjkm[tnt] = pm.tac_task_ctrl_param[tnt].ttjkm;
 }

TacServoController::TacServoController(ParameterManager &p) : ActController(p)
{
    deltais.setZero(6);
    deltais_int.setZero(6);
    deltais_old.setZero(6);
    delta_obj_int.setZero(3);
    delta_obj_old.setZero(3);
    delta_obj_int_o.setZero(3);
    delta_obj_old_o.setZero(3);
    deltape.setZero(6);

    initTacServoCtrlParam(CONTACT_POINT_TRACKING);
    initTacServoCtrlParam(CONTACT_FORCE_TRACKING);
    initTacServoCtrlParam(CONTACT_POINT_FORCE_TRACKING);
    initTacServoCtrlParam(SENSING_POLE_TRACKING);
    initTacServoCtrlParam(Z_ORIEN_TRACKING);
    initTacServoCtrlParam(LINEAR_TRACKING);
    initTacServoCtrlParam(COVER_OBJECT_SURFACE);
    initTacServoCtrlParam(OBJECT_SURFACE_EXPLORING);
    llv_tac.setZero();
    lov_tac.setZero();
//    if(mkdir("/dev/shm/debug",0777)==-1)//creating a directory
//    {
//        std::cout<<"creat folder failed"<<std::endl;
//    }
//    int temp;
//    if ((temp = mkdir("/dev/shm/debug",0777)) != 0) {
//        fprintf(stderr, "ERROR %d: unable to mkdir; %s\n", errno, strerror(errno));
//    }
//    std::string data_f ("/dev/shm/debug/");
//    ctrl_debug.open((data_f+std::string("ctrl_debug.txt")).c_str());
}


void TacServoController::update_robot_reference(Robot *robot){
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

void TacServoController::get_desired_lv(Robot *robot, Task *t, myrmex_msg *tacfb){
    TacServoTask tst(t->curtaskname.tact);
    tst = *(TacServoTask*)t;
    double desired_cp[2];
    double desiredf;
    //robot current state
    tst.get_desired_cp_myrmex(desired_cp);
    tst.get_desired_cf_myrmex(desiredf);
//    std::cout<<"desired contact p in myrmex "<<desired_cp[0]<<","<<desired_cp[1]<<std::endl;
    if(tacfb->contactflag == true){
        deltais(1) = tacfb->cogx - desired_cp[0];
        deltais(0) = tacfb->cogy - desired_cp[1];
        deltais(5) = M_PI/2 - tacfb->lineorien;
    }
    else{
        deltais(0) = 0;
        deltais(1) = 0;
        deltais(5) = 0;
        deltais_int.setZero();
    }
    deltais(2) =  desiredf - tacfb->cf;
    //!this two value can be updated by other feedback in future
    deltais(3) = 0;
    deltais(4) = 0;
    deltais_int = deltais_int + deltais;
//    std::cout<<"desiredis "<<deltais<<std::endl;
//    std::cout<<"current task name "<<tst.curtaskname.tact<<std::endl;
//    std::cout<<"kop: "<<std::endl;
//    std::cout<<Kop[tst.curtaskname.tact]<<std::endl;
//    std::cout<<"kpp: "<<std::endl;
//    std::cout<<Kpp[tst.curtaskname.tact]<<std::endl;
//    std::cout<<"tjkm: "<<std::endl;
//    std::cout<<tjkm[tst.curtaskname.tact]<<std::endl;
//    std::cout<<"sm: "<<std::endl;
//    std::cout<<sm[tst.curtaskname.tact]<<std::endl;

    deltape = Kpp[tst.curtaskname.tact] * tjkm[tst.curtaskname.tact] * sm[tst.curtaskname.tact] * deltais + \
            Kpi[tst.curtaskname.tact] * tjkm[tst.curtaskname.tact] * sm[tst.curtaskname.tact] * deltais_int + \
            Kpd[tst.curtaskname.tact] * tjkm[tst.curtaskname.tact] * sm[tst.curtaskname.tact] * (deltais - deltais_old);
//    std::cout<<"deltapa are "<<deltape<<std::endl;
    llv_tac = deltape.head(3);
    lov_tac = Kop[tst.curtaskname.tact] * deltape.tail(3);
    limit_vel(get_llv_limit(),llv_tac,lov_tac);
    deltais_old = deltais;
}

void TacServoController::update_robot_reference(Robot *robot, Task *t, myrmex_msg *tacfb){
    Eigen::Vector3d o_target,p_target;
    p_target.setZero();
    o_target.setZero();
    get_desired_lv(robot,t,tacfb);
//    std::cout<<"lv before in tacservo "<<llv<<std::endl;
//    std::cout<<lov<<std::endl;
    llv = llv + llv_tac;
    lov = lov + lov_tac;
//    std::cout<<"lv after in tacservo "<<llv<<std::endl;
//    std::cout<<lov<<std::endl;
    local_to_global(robot->get_cur_cart_p(),robot->get_cur_cart_o(),llv,\
                    lov,p_target,o_target);
    for (int i=0; i < 3; i++){
        cart_command[i] = p_target(i);
        cart_command[i+3] = o_target(i);
    }
    for(int i = 0; i < 6; i++)
        robot->set_cart_command(cart_command);
}

void TacServoController::get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov){
    lv = llv_tac;
    ov = lov_tac;
}

