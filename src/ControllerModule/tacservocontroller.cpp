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
    initTacServoCtrlParam(LEARN_TACTOOL_CONTACT);
    initTacServoCtrlParam(LEARN_TACTOOL_SLIDING);
    initTacServoCtrlParam(LEARN_TACTOOL_ROLLING);
    llv_tac.setZero();
    lov_tac.setZero();
    cdet_ptr = new ContactDetector();
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
    //robot desired state(contact taxel and contact pressrue)
    tst.get_desired_taxel_mid(act_id);
    tst.get_desired_cf_mid(desiredf);
    /*strategy I*/
    //There is no generalized sensor frame can be defined, the transformation
    //between the deviation of tactile info and finger-eef frame infinite tiny
    //movement have to be defined according to the contact area.
     /*strategy II*/
    Eigen::Vector3d ctc_nv,ctc_nv_normalize;
    Eigen::Vector3d des_nv_normalize;
    ctc_nv.setZero();
    ctc_nv_normalize.setZero();
    desired_nv.setZero();
    des_nv_normalize.setZero();
    if(midfb->isContact(midfb->data) == true){
        double deltaf;
        deltaf = desiredf - midfb->pressure;
        std::cout<<"deltaf "<<deltaf<<std::endl;
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

            des_nv_normalize = desired_nv.normalized();
            ctc_nv_normalize = ctc_nv.normalized();
            std::cout<<"dot product is"<<std::endl;
            std::cout<<des_nv_normalize.dot(ctc_nv_normalize)<<std::endl;
            if(des_nv_normalize.dot(ctc_nv_normalize)>=0.5){
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
            des_nv_normalize = desired_nv.normalized();
            ctc_nv_normalize = ctc_nv.normalized();
            std::cout<<"dot product is"<<std::endl;
            std::cout<<des_nv_normalize.dot(ctc_nv_normalize)<<std::endl;
            if(des_nv_normalize.dot(ctc_nv_normalize)>=0.5){
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
            deltais(0) =  midfb->pos[0] - desired_cp(0);
            deltais(1) = 0;
            deltais(2) = midfb->pos[2] - desired_cp(2);
            for(int i = 0; i < 3; i++){
                deltais(i) = deltais(i);
            }
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
    if(midfb->isContact(midfb->data) == true){
        if((t->curtaskname.tact == COVER_OBJECT_SURFACE)||(t->curtaskname.tact == LINEAR_TRACKING)){
            if(midfb->isContact(midfb->data) == true){
                double deltaf;
                deltaf = desiredf - midfb->pressure;
                for(int i = 0; i < 3; i++){
                    llv_tac(i) =  deltape(i) + (0.025)*deltaf*ctc_nv(i);
                }
            }
            else{
                llv_tac.setZero();
            }
        }
        if((t->curtaskname.tact == CONTACT_FORCE_TRACKING)||(t->curtaskname.tact == CONTACT_POINT_FORCE_TRACKING)){
            llv_tac = deltape.head(3);
        }
    }
    else{
        llv_tac.setZero();
    }

    lov_tac = Kop[tst.curtaskname.tact] * deltape.tail(3);
    if(midfb->isContact(midfb->data) == true){
        std::cout<<"before the normal direction control "<<lov_tac(0)<<","<<lov_tac(1)<<","<<lov_tac(2)<<std::endl;
        Eigen::Vector3d rot_nv;
        rot_nv.setZero();
        if((t->curtaskname.tact == CONTACT_POINT_TRACKING)||(t->curtaskname.tact == CONTACT_POINT_FORCE_TRACKING)||\
                (t->curtaskname.tact == COVER_OBJECT_SURFACE)||(t->curtaskname.tact == LINEAR_TRACKING)){
            ctc_nv = midfb->nv;
            if(t->tft == TAXEL_NUM){
                desired_nv = midfb->data.fingertip_tac_nv.at(act_id);
            }
            else{
                t->get_desired_nv_mid(desired_nv);
            }
            //rolling components
            rot_nv = desired_nv.cross(ctc_nv);
            std::cout<<"desured_nv "<<desired_nv(0)<<","<<desired_nv(1)<<","<<desired_nv(2)<<std::endl;
            std::cout<<"ctc_nv "<<ctc_nv(0)<<","<<ctc_nv(1)<<","<<ctc_nv(2)<<std::endl;
            std::cout<<"rot_nv "<<rot_nv(0)<<","<<rot_nv(1)<<","<<rot_nv(2)<<std::endl;
            des_nv_normalize = desired_nv.normalized();
            ctc_nv_normalize = ctc_nv.normalized();
            lov_tac = lov_tac + 0.1*(1-(des_nv_normalize.dot(ctc_nv_normalize)))*rot_nv;
            std::cout<<"after the normal direction control "<<lov_tac(0)<<","<<lov_tac(1)<<","<<lov_tac(2)<<std::endl;
            //twist component, only use x z component of slope to compute the deviation angle between current estimated linear and z axis
            //only superimposing twist motion while following the cable.
            if(t->curtaskname.tact == LINEAR_TRACKING){
                double comp_x, comp_z,comp_y,l_feature;
                comp_x = midfb->slope(0);
                comp_y = midfb->slope(1);
                comp_z = midfb->slope(2);
                l_feature = atan(comp_x / comp_z);
                if((fabs(comp_x)>= 1e-5)&&(fabs(comp_y)>= 1e-5)&&(fabs(comp_z)>= 1e-5)){
                    lov_tac(1) = lov_tac(1) + (-0.05) * l_feature;
                }
            }
        }
    }
    else{

    }
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
    if(midfb->isContact(midfb->data) == true){
        if((t->curtaskname.tact == COVER_OBJECT_SURFACE)){
        }
    }

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
        deltais(5) = 0- tacfb->lineorien;
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
    std::cout<<"o_target "<<o_target(0)<<","<<o_target(1)<<","<<o_target(2)<<std::endl;
    for(int i = 0; i < 6; i++)
        robot->set_cart_command(cart_command);
}

void TacServoController::get_desired_lv(ManipTool *mt, Robot *robot, Task *t,myrmex_msg *tacfb){
    TacServoTask tst(t->curtaskname.tact);
    double desired_cp[2];
    tst = *(TacServoTask*)t;
    double desiredf;
    double desiredorien;
    //robot current state
    tst.get_desired_cf_myrmex(desiredf);
    tst.get_desired_cp_myrmex(desired_cp);
    tst.get_desired_orien_myrmex(desiredorien);
    if(tacfb->contactflag == true){
        if (t->emt == LINEAREXPLORE){
            deltais(0) = tst.dir_x;
            deltais(1) = tst.dir_y;
        }
        else{
            deltais(0) = (tacfb->cogy - desired_cp[0]);
            deltais(1) = (tacfb->cogx - desired_cp[1]);
        }
        deltais(2) =  desiredf - tacfb->cf;
//        std::cout<<"desiredf and current f are "<<desiredf<<","<<tacfb->cf<<std::endl;
        deltais(5) = desiredorien - tacfb->lineorien;
//        std::cout<<"deisred and current line direction"<<desiredorien<<",,,,,"<<tacfb->lineorien<<std::endl;
    }
    else{
        deltais(0) = 0;
        deltais(1) = 0;
        deltais(2) =  desiredf - tacfb->cf;
        deltais(5) = 0;
        deltais_int.setZero();
    }

    //!this two value can be updated by other feedback in future
    deltais(3) = 0;
    deltais(4) = 0;
    deltais_int = deltais_int + deltais;
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
    if (t->emt == LINEAREXPLORE){
        llv_tac = mt->ts.tac_sensor_cfm_local* deltape.head(3);
    }
    else{
        if(robot->toolname == none)
            llv_tac = mt->ts.tac_sensor_cfm_local*deltape.head(3);
        if(robot->toolname == tactool)
            llv_tac =  deltape.head(3);
    }
    if (t->emt == ROTATEEXPLORE){
        deltape.tail(3) = tst.desired_pose_range;
    }
//    std::cout<<"rotation stimulous"<<deltape.tail(3)<<std::endl;
    lov_tac = Kop[tst.curtaskname.tact] * deltape.tail(3);
    limit_vel(get_llv_limit(),llv_tac,lov_tac);
//    std::cout<<"local rot vel "<<lov_tac(0)<<","<<lov_tac(1)<<","<<lov_tac(2)<<std::endl;
    deltais_old = deltais;
}

void TacServoController::update_robot_reference(ManipTool *mt, Robot *robot, Task *t,myrmex_msg *tacfb){
    TacServoTask tst(t->curtaskname.tact);
    tst = *(TacServoTask*)t;
    Eigen::Vector3d o_target,p_target;
    p_target.setZero();
    o_target.setZero();
    get_desired_lv(mt,robot,t,tacfb);
    limit_eef_euler(t->get_desired_rotation_range());
//    std::cout<<"lv before in tacservo "<<llv<<std::endl;
//    std::cout<<"ov before in tacservo "<<lov<<std::endl;
    if(tacfb->contactflag == true){
        if((t->emt == LINEAREXPLORE)||(t->emt == ROTATEEXPLORE)){
            if(cdet_ptr->isContactArea(tacfb->cogx,tacfb->cogy) == true){
                llv = llv + llv_tac;
                lov = lov + lov_tac;
            }
            else if(cdet_ptr->isSlideOK(tacfb->cogx,tacfb->cogy,tst.dir_x,tst.dir_y) == true){
                llv = llv + llv_tac;
                lov = lov + lov_tac;
            }
            else{
                llv = llv;
                lov = lov;
            }
        }
        else{
            llv = llv + llv_tac;
            lov = lov + lov_tac;
        }
    }
    //temporary set like this in order to avoid the init contact failure while the controller is switching
    else{
        llv = llv + llv_tac;
        lov = lov + lov_tac;
    }
//    std::cout<<"lv after in tacservo "<<llv<<std::endl;
//    std::cout<<"ov after in tacservo "<<lov<<std::endl;
    local_to_global(robot->get_cur_cart_p(),robot->get_cur_cart_o(),llv,\
                    lov,p_target,o_target);
    for (int i=0; i < 3; i++){
        cart_command[i] = p_target(i);
        cart_command[i+3] = o_target(i);
    }
//    std::cout<<"o_target "<<o_target(0)<<","<<o_target(1)<<","<<o_target(2)<<std::endl;
    for(int i = 0; i < 6; i++)
        robot->set_cart_command(cart_command);
}



void TacServoController::get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov){
    lv = llv_tac;
    ov = lov_tac;
}

