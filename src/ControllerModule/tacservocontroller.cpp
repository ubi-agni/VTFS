#include "ControllerModule/tacservocontroller.h"
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
void TacServoController::get_desired_lv(Robot *robot, Task *t, FingertipTac *midfb){
    TacServoTask tst(t->curtaskname.tact);
    tst = *(TacServoTask*)t;
    int act_id;
    double desiredf;
    Eigen::Vector3d desired_cp,desired_nv;
    //robot current state
    tst.get_desired_taxel_mid(act_id);
    tst.get_desired_cf_mid(desiredf);
    /*strategy I*/
    //There is no generalized sensor frame can be defined, the transformation
    //between the deviation of tactile info and finger-eef frame infinite tiny
    //movement have to be defined according to the contact area.
     /*strategy II*/
//    if(midfb->isContact(midfb->data) == true){
//        desired_cp = midfb->data.fingertip_tac_position.at(act_id);
//        deltais(0) = desired_cp[0] - midfb->pos[0];
//        deltais(2) = desired_cp[2] - midfb->pos[2];
//    }
//    else{
//        deltais(0) = 0;
//        deltais(2) = 0;
//        deltais_int.setZero();
//    }
//    deltais(1) =  desiredf - midfb->pressure;
//    //!this two value can be updated by other feedback in future
//    deltais(3) = 0;
//    deltais(4) = 0;
//    deltais(5) = 0;


    if(midfb->isContact(midfb->data) == true){
        Eigen::Vector3d ctc_nv;
        Eigen::Vector3d des_nv_normaize,ctc_nv_normalize;
        desired_nv.setZero();
        des_nv_normaize.setZero();
        ctc_nv_normalize.setZero();
        double deltaf;
        deltaf = desiredf - midfb->pressure;
        ctc_nv.setZero();
        ctc_nv = midfb->nv;
        if(t->curtaskname.tact == CONTACT_FORCE_TRACKING){
            deltais(0) = deltaf*ctc_nv(0);
            deltais(1) = deltaf*ctc_nv(1);
            deltais(2) = deltaf*ctc_nv(2);
        }
        if(t->curtaskname.tact == CONTACT_POINT_TRACKING){
            if(t->tft == TAXEL_NUM){
                desired_cp = midfb->data.fingertip_tac_position.at(act_id);
                desired_nv = midfb->data.fingertip_tac_nv.at(act_id);
            }
            else{
                t->get_desired_position_mid(desired_cp);
                t->get_desired_nv_mid(desired_nv);
            }

            des_nv_normaize = desired_nv.normalized();
            ctc_nv_normalize = ctc_nv.normalized();
            std::cout<<"dot product is"<<std::endl;
            std::cout<<des_nv_normaize.dot(ctc_nv_normalize)<<std::endl;
            if(des_nv_normaize.dot(ctc_nv_normalize)>=0.5){
                deltais(0) = midfb->pos[0] - desired_cp(0);
                deltais(1) = midfb->pos[1] - desired_cp(1);
                deltais(2) = midfb->pos[2] - desired_cp(2);
            }
            else{
                deltais.setZero();
            }
        }
        if(t->curtaskname.tact == CONTACT_POINT_FORCE_TRACKING){
            if(t->tft == TAXEL_NUM){
                desired_cp = midfb->data.fingertip_tac_position.at(act_id);
                desired_nv = midfb->data.fingertip_tac_nv.at(act_id);
            }
            else{
                t->get_desired_position_mid(desired_cp);
                t->get_desired_nv_mid(desired_nv);
            }
            des_nv_normaize = desired_nv.normalized();
            ctc_nv_normalize = ctc_nv.normalized();
            std::cout<<"dot product is"<<std::endl;
            std::cout<<des_nv_normaize.dot(ctc_nv_normalize)<<std::endl;
            if(des_nv_normaize.dot(ctc_nv_normalize)>=0.5){
                deltais(0) = midfb->pos[0] - desired_cp(0);
                deltais(1) = midfb->pos[1] - desired_cp(1);
                deltais(2) = midfb->pos[2] - desired_cp(2);
            }
            else{
                deltais.setZero();
            }
            std::cout<<"deltais before"<<std::endl;
            std::cout<<deltais<<std::endl;
            std::cout<<"fdev before"<<std::endl;
            std::cout<<deltaf*ctc_nv<<std::endl;
            std::cout<<"f "<<desiredf<<","<<midfb->pressure<<std::endl;
            for(int i = 0; i < 3; i++){
                deltais(i) = deltais(i) + (100.0)*deltaf*ctc_nv(i);
            }
            std::cout<<"deltais after"<<std::endl;
            std::cout<<deltais<<std::endl;
        }
        if((t->curtaskname.tact == COVER_OBJECT_SURFACE)||(t->curtaskname.tact == LINEAR_TRACKING)){
            if(t->tft == TAXEL_NUM){
                desired_cp = midfb->data.fingertip_tac_position.at(act_id);
            }
            else{
                t->get_desired_position_mid(desired_cp);
            }
            deltais(0) = desired_cp(0) - midfb->pos[0];
            deltais(1) = 0;
            deltais(2) = desired_cp(2) - midfb->pos[2];
            std::cout<<"deltais before"<<std::endl;
            std::cout<<deltais<<std::endl;
            std::cout<<"fdev before"<<std::endl;
            std::cout<<deltaf*ctc_nv<<std::endl;
            for(int i = 0; i < 3; i++){
                deltais(i) = deltais(i) + (50.0)*deltaf*ctc_nv(i);
            }
            std::cout<<"deltais after"<<std::endl;
            std::cout<<deltais<<std::endl;
        }
    }
    else{
        deltais(0) = 0;
        deltais(1) =  0;
        deltais(2) = 0;
        deltais_int.setZero();
    }
    //!this two value can be updated by other feedback in future
    deltais(3) = 0;
    deltais(4) = 0;
    deltais(5) = 0;


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
    if(midfb->isContact(midfb->data) == true){
        Eigen::Vector3d rot_nv;
        Eigen::Vector3d ctc_nv;
        Eigen::Vector3d des_nv_normaize,ctc_nv_normalize;
        rot_nv.setZero();
        desired_nv.setZero();
        des_nv_normaize.setZero();
        ctc_nv_normalize.setZero();
        if((t->curtaskname.tact == CONTACT_POINT_TRACKING)||(t->curtaskname.tact == CONTACT_POINT_FORCE_TRACKING)||(t->curtaskname.tact == COVER_OBJECT_SURFACE)||(t->curtaskname.tact == LINEAR_TRACKING)){
            ctc_nv.setZero();
            ctc_nv = midfb->nv;
            if(t->tft == TAXEL_NUM){
                desired_nv = midfb->data.fingertip_tac_nv.at(act_id);
            }
            else{
                t->get_desired_nv_mid(desired_nv);
            }
            //rolling components
            rot_nv = desired_nv.cross(ctc_nv);
            des_nv_normaize = desired_nv.normalized();
            ctc_nv_normalize = ctc_nv.normalized();
            lov_tac = lov_tac + 0.05*(1-(des_nv_normaize.dot(ctc_nv_normalize)))*rot_nv;
            //twist component, only use x z component of slope to compute the deviation angle between current estimated linear and z axis
            double comp_x, comp_z,comp_y,l_feature;
            comp_x = midfb->slope(0);
            comp_y = midfb->slope(1);
            comp_z = midfb->slope(2);
            l_feature = comp_x / comp_z;
            if((fabs(comp_x)>= 1e-5)&&(fabs(comp_y)>= 1e-5)&&(fabs(comp_z)>= 1e-5)){
                lov_tac(1) = lov_tac(1) + (-0.01) * l_feature;
            }


        }
    }
//    std::cout<<"lov_tac are "<<lov_tac(0)<<","<<lov_tac(1)<<","<<lov_tac(2)<<std::endl;
    limit_vel(get_llv_limit(),llv_tac,lov_tac);
    deltais_old = deltais;
}

void TacServoController::update_robot_reference(Robot *robot, Task *t,FingertipTac *midfb){
    Eigen::Vector3d o_target,p_target;
    p_target.setZero();
    o_target.setZero();
    get_desired_lv(robot,t,midfb);
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
