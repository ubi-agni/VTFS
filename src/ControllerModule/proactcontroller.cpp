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
    rot_by_motion = false;
}

ProActController::ProActController(ParameterManager &p, const std::string &dmpFileName) : ActController(p)
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

    dmpCtrl.reset(new DMP::UMITSMP(100, 2));
    DMP::SampledTrajectoryV2 trajectory;
    std::vector<DMP::SampledTrajectoryV2> trajs;
    trajectory.readFromCSVFile(dmpFileName);
    trajs.push_back(trajectory);
    dmpCtrl->learnFromTrajectories(trajs);
    init_position.setZero();
    est_rot_angle = 0;
    pro_data_store.open("/tmp/data/pro_data_store.txt");
    lv_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(10,Average,Eigen::Vector3d(0,0,0));
    rot_by_motion = false;
    
//    beginClock = std::chrono::steady_clock::now();

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

void ProActController::get_desired_lv(Robot *robot, Task *t,Eigen::Vector3d cur_dir, Eigen::Vector3d ft_f, RobotState* rs){
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

    
    if(tst.Ssrc == DMP_FEEDBACK){
	    // dmp
	    Eigen::Vector3d localDMPTargetVelocity ;
	    localDMPTargetVelocity.setZero();
	    if (canVal > 1e-8)
	    {
	        double deltaT = 0.004;    
	        canVal = canVal - deltaT;
	        DMP::DVec targetState;
	        currentState = dmpCtrl->calculateDirectlyVelocity(currentState, canVal/motionTimeDuration, deltaT, targetState);
	
	        Eigen::Vector3d currentGlobalPosition = rs->robot_position["eef"];
	
	        //Eigen::Vector3d globalTargetPosition;
	        //globalTargetPosition << targetState.at(0), targetState.at(1), currentGlobalPosition(2);
	
	        //Eigen::Vector3d globalTargetVelocity;
	        //globalTargetVelocity << currentState.at(0).vel/motionTimeDuration, currentState.at(1).vel/motionTimeDuration, 0;
	
	        Eigen::Vector3d localTargetPosition;
	        localTargetPosition << 0, targetState.at(1), targetState.at(2);
	
	//		        Eigen::Vector3d cur_ft = ft_f.normalized();
	//		Eigen::Vector3d init_ft = tst.init_contact_frame.col(0);
	//		Eigen::Matrix3d curr_contact_frame = tst.init_contact_frame;
			
	//		if(ft_f.norm() != 0)
	//		{
	//		Eigen::Vector3d rot_axis = init_ft.cross(cur_ft);
	//		double angle = acos(cur_ft.dot(init_ft));
					
	//		curr_contact_frame =  Eigen::AngleAxisd(angle, rot_axis) * tst.init_contact_frame;
	//		std::cout<<"rot axis " <<rot_axis(0)<<","<<rot_axis(1)<<","<<rot_axis(2)<<","<<std::endl;
	//		std::cout<<"rot angle " <<angleFilter<<std::endl;
	//		std::cout<<"desired contact frame "<<std::endl;
	//		std::cout<<curr_contact_frame<<std::endl;
	//	    }
			
	        Eigen::Vector3d cur_ft = ft_f.normalized();
	        Eigen::Vector3d init_ft = origForce;
	        Eigen::Matrix3d curr_contact_frame = contactFrame;
			
	        float KpForce = 1;
	        if((cur_ft - init_ft).norm() > 1e-3)
			{
	            Eigen::Vector3d currForce = KpForce * (cur_ft - init_ft) + init_ft;
	            Eigen::Vector3d rot_axis = init_ft.cross(currForce);
	            double angle = acos(cur_ft.dot(init_ft));
	
	            curr_contact_frame =  Eigen::AngleAxisd(angle, rot_axis) * contactFrame;
	            std::cout<<"rot axis " <<rot_axis(0)<<","<<rot_axis(1)<<","<<rot_axis(2)<<","<<std::endl;
	            std::cout<<"rot angle " <<angle<<std::endl;
	            std::cout<<"desired contact frame "<<std::endl;
	            std::cout<<curr_contact_frame<<std::endl;
	            origForce = currForce;
	            contactFrame = curr_contact_frame;
		    }
	        Eigen::Vector3d globalTargetPosition = curr_contact_frame * localTargetPosition + init_position;
	//        std::cout<<"init contact frame "<<std::endl;
	//        std::cout<<tst.init_contact_frame<<std::endl;
	        
	//        std::cout << "targetState: " << targetState[0] << " " << targetState[1]<<" "<<targetState[2]<<std::endl;
	//        std::cout<<"globalTargetPosition: " << globalTargetPosition(0) << ","<< globalTargetPosition(1) << ","<< globalTargetPosition(2) << std::endl;
	//        std::cout <<"current position from kuka fk "<<rs->robot_position["eef"](0)<<","<<rs->robot_position["eef"](1)<<","<<rs->robot_position["eef"](2)<<std::endl;
	        
	        pro_data_store<<globalTargetPosition(0) << ","<< globalTargetPosition(1) << ","<< globalTargetPosition(2) << ","
	                    << targetState[0] << "," << targetState[1]<<","<<targetState[2]<<","<<rs->robot_position["eef"](0)
	                    <<","<<rs->robot_position["eef"](1)<<","<<rs->robot_position["eef"](2)<<std::endl;
	        
	        
	        Eigen::Vector3d globalCurrentVelocity = rs->EstRobotEefLVel_Ref(robot);
	
	        double Kp = 12;
	    //    double Kd = 0.05;
	        double Kd = 2;
	        Eigen::Vector3d globalTargetVelocity = Kp * (globalTargetPosition - currentGlobalPosition) + Kd * ( (-1)* globalCurrentVelocity);
	
	        //std::cout << "canVal: " << canVal << " globalTargetVelocity: " << globalTargetVelocity(0) << ","<< globalTargetVelocity(1) << ","<< globalTargetVelocity(2) << std::endl;
	        Eigen::Vector3d localTargetVel = curr_contact_frame.transpose() * globalTargetVelocity;
	        localTargetVel(0) = 0;
	        globalTargetVelocity =  curr_contact_frame * localTargetVel;
	        
	        localDMPTargetVelocity= rs->robot_orien["eef"].transpose() * globalTargetVelocity;
	    }
	    // llv_pro = lv_pro.head(3)
	    llv_pro =  localDMPTargetVelocity;
	    //The rotation behaviour of DMP-based exploration is done by force controller.
	    lov_pro.setZero();
    }
    if(tst.Ssrc == MANUAL){
		Eigen::Vector3d rs_lv,v_ratio,filtered_lv;
		rs_lv.setZero();
		v_ratio.setZero();
		filtered_lv.setZero();
		
		
		Eigen::Matrix3d Rel;
		Rel = rs->robot_orien["eef"].transpose() * tst.init_contact_frame  * Eigen::AngleAxisd(est_rot_angle, Eigen::Vector3d::UnitZ());
		llv_pro = lv_pro.head(3) = Kpp[tst.curtaskname.prot].block(0,0,3,3) * \
	                    psm[tst.curtaskname.prot].block(0,0,3,3) * Rel * identity_v.head(3);   

        if(tst.tmp == SLIDINGY){
            //test for y axis in tool frame
		    identity_v(0) = 0;
		    identity_v(2) = 0;
			llv_pro = Kpp[tst.curtaskname.prot].block(0,0,3,3) * \
	                    psm[tst.curtaskname.prot].block(0,0,3,3) * Rel * identity_v.head(3); 
			}
		if(tst.tmp == SLIDINGZ){
            //test for z axis in tool frame
		    identity_v(0) = 0;
		    identity_v(1) = 0;
			llv_pro = Kpp[tst.curtaskname.prot].block(0,0,3,3) * \
	                    psm[tst.curtaskname.prot].block(0,0,3,3) * Rel * identity_v.head(3); 
			}
			
	    if(tst.tmp == ROTATE_TOWARDS_AXIS){

			global2local(normalized_cur_dir.cross(tst.desired_axis_dir),\
	                     rs->robot_orien["eef"],tmp_rot_axis);
			delta_ag = (1.0-fabs(normalized_cur_dir.dot(tst.desired_axis_dir)));

			delta_ag_int =  delta_ag_int + delta_ag;  

			//obtain the rotation command requested by the desired goal direction of rotation axis              
			lv_pro.tail(3) = delta_ag * tmp_rot_axis + 0.001 * delta_ag_int * tmp_rot_axis;

			lov_pro = Kop[tst.curtaskname.prot] * lv_pro.tail(3);

	    }
	    if(tst.tmp == ROTATE_AROUND_AXIS)
		    lov_pro = Kop[tst.curtaskname.prot] *(-0.02) * rs->robot_orien["eef"].transpose()*normalized_cur_dir;
	    if(tst.tmp == NOPRIM)
	        lov_pro.setZero();


        if(tst.tmp == SLIDINGY_ROTATE_BY_MOTION){
            //test for y axis in tool frame
            identity_v(0) = 0;
            identity_v(2) = 0;
            llv_pro = Kpp[tst.curtaskname.prot].block(0,0,3,3) * \
                        psm[tst.curtaskname.prot].block(0,0,3,3) * Rel * identity_v.head(3);

			//tool slides on a curved surface. Purpose: to estimate the desired contact force direction
			//cited paper: "Integrated vision/force robotic servoing in the task frame formalism"
			    
			//integator to estimate the rotation angle from the rotation rate.
			est_rot_angle = est_rot_angle + v_ratio(2) * 0.004;
			//estimate the current contact frame orientation using estimated rotation angle
			//R_new = R_init * R_axis_angle
			tool_contact_frame = tst.init_contact_frame * Eigen::AngleAxisd(est_rot_angle, Eigen::Vector3d::UnitZ());
			    
			//estimate the tool-origin's linear velocity(global)
			//est_v_g = rs->EstRobotEefLVel_Ref(robot);
			//estimate the tool-origin's linear velocity(local)
			rs_lv = tool_contact_frame.transpose() * rs->EstRobotEefLVel_Ref(robot);
			filtered_lv = lv_filter->push(rs_lv);
			
			if((filtered_lv[0] == 0)&&(filtered_lv[1] == 0)){
			    v_ratio.setZero();
			}
			else{
			    v_ratio(2) = atan(filtered_lv[1]/filtered_lv[0]);
			    v_ratio(1) = atan((-1)*filtered_lv[2]/filtered_lv[0]);
			    v_ratio(0) = atan(filtered_lv[2]/filtered_lv[1]);
			}
			lov_pro = Kop[tst.curtaskname.prot] * v_ratio;
		}

        if(tst.tmp == SLIDINGZ_ROTATE_BY_MOTION){
            identity_v(0) = 0;
            identity_v(1) = 0;
            llv_pro = Kpp[tst.curtaskname.prot].block(0,0,3,3) * \
                        psm[tst.curtaskname.prot].block(0,0,3,3) * Rel * identity_v.head(3);
            //tool slides on a curved surface. Purpose: to estimate the desired contact force direction
            //cited paper: "Integrated vision/force robotic servoing in the task frame formalism"

            //integator to estimate the rotation angle from the rotation rate.
            est_rot_angle = est_rot_angle + v_ratio(2) * 0.004;
            //estimate the current contact frame orientation using estimated rotation angle
            //R_new = R_init * R_axis_angle
            tool_contact_frame = tst.init_contact_frame * Eigen::AngleAxisd(est_rot_angle, Eigen::Vector3d::UnitZ());

            //estimate the tool-origin's linear velocity(global)
            //est_v_g = rs->EstRobotEefLVel_Ref(robot);
            //estimate the tool-origin's linear velocity(local)
            rs_lv = tool_contact_frame.transpose() * rs->EstRobotEefLVel_Ref(robot);
            filtered_lv = lv_filter->push(rs_lv);

            if((filtered_lv[0] == 0)&&(filtered_lv[1] == 0)){
                v_ratio.setZero();
            }
            else{
                v_ratio(2) = atan(filtered_lv[1]/filtered_lv[0]);
                v_ratio(1) = atan((-1)*filtered_lv[2]/filtered_lv[0]);
                v_ratio(0) = atan(filtered_lv[2]/filtered_lv[1]);
            }
            lov_pro = Kop[tst.curtaskname.prot] * v_ratio;
        }
    	
	}
//    limit_vel(get_llv_limit(),llv_pro,lov_pro);
}


void ProActController::update_robot_reference(Robot *robot, Task *t, Eigen::Vector3d cur_dir, Eigen::Vector3d ft_f, RobotState* rs){
    Eigen::Vector3d p_target,o_target;
    p_target.setZero();
    o_target.setZero();
    if(t->mft == GLOBAL){
        p_target = t->get_desired_p_eigen();
        o_target = t->get_desired_o_ax();
    }
    if(t->mft == LOCAL){
        get_desired_lv(robot,t,cur_dir,ft_f,rs);
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
