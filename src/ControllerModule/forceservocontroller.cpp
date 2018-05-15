#include "ControllerModule/forceservocontroller.h"
#include "TaskModule/forceservotask.h"

void ForceServoController::initForceServoCtrlParam(FORCETaskNameT fnt){
    Kpp[fnt].setZero(6, 6);
    Kpi[fnt].setZero(6, 6);
    Kpd[fnt].setZero(6, 6);
    Kop[fnt].setZero(3, 3);
    sm[fnt].setZero(6, 6);
    tjkm[fnt].setZero(6, 6);
    Kpp[fnt] = pm.force_task_ctrl_param[fnt].kpp;
    Kpi[fnt] = pm.force_task_ctrl_param[fnt].kpi;
    Kpd[fnt] = pm.force_task_ctrl_param[fnt].kpd;
    Kop[fnt] = pm.force_task_ctrl_param[fnt].kop;
    sm[fnt] = pm.force_task_ctrl_param[fnt].tsm;
    tjkm[fnt] = pm.force_task_ctrl_param[fnt].ttjkm;

}

void ForceServoController::updateForceServoCtrlParam(FORCETaskNameT fnt){
    Kpp[fnt] = pm.force_task_ctrl_param[fnt].kpp;
    Kpi[fnt] = pm.force_task_ctrl_param[fnt].kpi;
    Kpd[fnt] = pm.force_task_ctrl_param[fnt].kpd;
    Kop[fnt] = pm.force_task_ctrl_param[fnt].kop;
    sm[fnt] = pm.force_task_ctrl_param[fnt].tsm;
    tjkm[fnt] = pm.force_task_ctrl_param[fnt].ttjkm;
}

ForceServoController::ForceServoController(ParameterManager &p) : ActController(p)
{
    deltais.setZero(6);
    deltais_int.setZero(6);
    deltais_old.setZero(6);
    delta_obj_int.setZero(3);
    delta_obj_old.setZero(3);
    delta_obj_int_o.setZero(3);
    delta_obj_old_o.setZero(3);
    deltape.setZero(6);

    initForceServoCtrlParam(F_MAINTAIN);
    initForceServoCtrlParam(F_CURVETRACKING);
    llv_f.setZero();
    lov_f.setZero();
    lv_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    vel_rec2.open("/tmp/data/vel_rec.txt");
}
void ForceServoController::get_desired_lv(Robot *robot, Task *t, Eigen::VectorXd ft, RobotState* rs){
    ForceServoTask tst(t->curtaskname.forcet);
    tst = *(ForceServoTask*)t;
    Eigen::Vector3d rs_lv,v_ratio;
    Eigen::Vector3d est_v_g;
    Eigen::Vector3d filtered_lv;
    est_v_g.setZero();
    rs_lv.setZero();
    v_ratio.setZero();
    //get the desired contact force
    double desiredf;
    tst.get_desired_cf_kuka(desiredf);
    
    if(tst.mft == GLOBAL){
		Eigen::VectorXd delta_g;
		delta_g.setZero(6);
        delta_g.head(3) = (-1) * (desiredf * tst.desired_surf_nv - ft.head(3));
		delta_g(3) = 0;
		delta_g(4) = 0;
		delta_g(5) = 0;
		deltais.head(3) = rs->robot_orien["eef"].transpose() * delta_g.head(3);
		deltais(3) = 0;
		deltais(4) = 0;
		deltais(5) = 0;
		}
	else{
		deltais(0) = 1;
		deltais(1) = 1;
		deltais(2) =  desiredf - ft(2);
		//!this two value can be updated by other feedback in future
		deltais(3) = 0;
		deltais(4) = 0;
		deltais(5) = 0;
		}
    std::cout<<"current force "<<ft(0)<<","<<ft(1)<<","<<ft(2)<<","<<std::endl;
//    std::cout<<"desiredis "<<deltais(0)<<","<<deltais(1)<<","<<deltais(2)<<","<<std::endl;
//    std::cout<<"current task name "<<tst.curtaskname.forcet<<std::endl;
//    std::cout<<"kop: "<<std::endl;
//    std::cout<<Kop[tst.curtaskname.forcet]<<std::endl;
//    std::cout<<"kpp: "<<std::endl;
//    std::cout<<Kpp[tst.curtaskname.forcet]<<std::endl;
//    std::cout<<"tjkm: "<<std::endl;
//    std::cout<<tjkm[tst.curtaskname.forcet]<<std::endl;
//    std::cout<<"sm: "<<std::endl;
//    std::cout<<sm[tst.curtaskname.forcet]<<std::endl;

    deltape = Kpp[tst.curtaskname.forcet] * sm[tst.curtaskname.forcet] * deltais + \
            Kpi[tst.curtaskname.forcet] * sm[tst.curtaskname.forcet] * deltais_int + \
            Kpd[tst.curtaskname.forcet] * sm[tst.curtaskname.forcet] * (deltais - deltais_old);
    llv_f = deltape.head(3);
    est_v_g = rs->EstRobotEefLVel_Ref(robot);

    rs_lv = rs->robot_orien["eef"].transpose()*rs->EstRobotEefLVel_Ref(robot);
    filtered_lv = lv_filter->push(rs_lv);

    if((filtered_lv[0] == 0)&&(filtered_lv[1] == 0)){
        v_ratio.setZero();
    }else{
        v_ratio(2) = atan(filtered_lv[1]/filtered_lv[0]);
        v_ratio(1) = atan((-1)*filtered_lv[2]/filtered_lv[0]);
        v_ratio(0) = atan(filtered_lv[2]/filtered_lv[1]);
    }
    lov_f = Kop[tst.curtaskname.forcet] * v_ratio;
    lov_f(0) = 0.0;
    lov_f(1) = 0.0;
    lov_f(2) = 0.0;
    //~ std::cout<<lov_f(0)<<","<<lov_f(1)<<","<<lov_f(2)<<std::endl;
    //~ vel_rec2<<est_v_g(0)<<","<<est_v_g(1)<<","<<est_v_g(2)<<","\
           //~ <<filtered_lv[0]<<","<<filtered_lv[1]<<","<<filtered_lv[2]<<","\
          //~ <<v_ratio(0)<<","<<v_ratio(1)<<","<<v_ratio(2)<<","\
         //~ <<llv_f(0)<<","<<llv_f(1)<<","<<llv_f(2)<<","\
        //~ <<lov_f(0)<<","<<lov_f(1)<<","<<lov_f(2)<<std::endl;
    limit_vel(get_llv_limit(),llv_f,lov_f);
    deltais_old = deltais;

}
void ForceServoController::update_robot_reference(Robot *robot, Task *t, Eigen::VectorXd kukaft, RobotState* rs){
    Eigen::Vector3d o_target,p_target;
    p_target.setZero();
    o_target.setZero();
    get_desired_lv(robot,t,kukaft,rs);
//    std::cout<<"lv before in tacservo "<<llv<<std::endl;
//    std::cout<<lov<<std::endl;
    llv = llv + llv_f;
    lov = lov + lov_f;
    std::cout<<"lv after in forceservo "<<llv<<std::endl;
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
