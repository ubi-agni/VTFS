#include "maniptool.h"
#include <stdlib.h>

ManipTool::ManipTool(RobotState *rs)
{
    mtt = Notool;
    Gama_r = 1.5*Eigen::Matrix3d::Identity();
    L_r = Eigen::Matrix3d::Zero();
    L_r_dot = Eigen::Matrix3d::Zero();
    c_r.setZero();
    c_r_dot.setZero();
    beta_r = 0.9;
    est_trans.setZero();
    est_trans_dot.setZero();
    est_nv.setZero();

    P_bar.setZero();
    L_n.setZero();
    L_n_dot.setZero();
    nv_dot.setZero();
}

void ManipTool::update_tac_sensor_cfm_local(){
    ts.tac_sensor_cfm_local = ts.tac_sensor_cfm_local* ts.rotate_s2sdot;
}

Eigen::Vector3d ManipTool::update_translation_est(Eigen::Vector3d lv,Eigen::Vector3d rv,\
                                       Eigen::Matrix3d robot_eef_rm, MyrmexTac *myrtac){
    Eigen::Matrix3d omiga_skmatrix;
    Eigen::Vector3d temp_lv;
    Eigen::Vector3d r_hat;
    r_hat.setZero();
    temp_lv.setZero();
    omiga_skmatrix.setZero();
    omiga_skmatrix = vectortoskew(rv);
    temp_lv(1) = myrtac->ctc_vel(0);
    temp_lv(0) = myrtac->ctc_vel(1);

    //get the vector from center of tactile sensor to the contact point
    r_hat(0) = myrtac->cog_y -7.5;
    r_hat(1) = myrtac->cog_x -7.5;

    L_r_dot = (-1)*beta_r*L_r-omiga_skmatrix*omiga_skmatrix;

    //compute the ve, comparing with Yannis's paper, I am using -v as the Tao.
    //because tao = r /cross (f), and v = omega /cross (r)

    c_r_dot = (-1)*beta_r*c_r+omiga_skmatrix*(-1)*((ts.tac_sensor_cfm_local*((-1)*temp_lv*0.005/0.004)-\
                                               rv.cross(ts.tac_sensor_cfm_local*r_hat*0.005)) - lv);
    est_trans_dot = (-1)*Gama_r*(L_r*est_trans-c_r);
    L_r = L_r + L_r_dot;
    c_r = c_r + c_r_dot;
    est_trans = est_trans + est_trans_dot;
    return ((ts.tac_sensor_cfm_local*(temp_lv*0.005/0.004)-\
             rv.cross(ts.tac_sensor_cfm_local*r_hat*0.005)) - lv);
}


void ManipTool::update_nv(Eigen::Vector3d lv,Eigen::Vector3d& n_hat,Eigen::Vector3d& nv_dot_fb){
    P_bar = Eigen::Matrix3d::Identity()-n_hat*n_hat.transpose();
    nv_dot = -1*Gama_n*P_bar*L_n*n_hat;
    nv_dot_fb = nv_dot;
    L_n_dot = -beta_n*L_n+(1/(1+pow(lv.norm(),2)))*lv*lv.transpose();
    n_hat = n_hat+nv_dot;
    n_hat = n_hat/n_hat.norm();
    L_n = L_n + L_n_dot;

}


void ManipTool::load_parameters(std::string fn_nv, std::string fn_rorate,std::string fn_trans){
    if((is_file_exist(fn_nv.c_str()) == true)&&(is_file_exist(fn_rorate.c_str()) == true)&&\
            (is_file_exist(fn_trans.c_str()) == true)){
        f_nv.open(fn_nv.c_str());
        ts.rel_o = readMatrix(f_nv);
        f_nv.close();
        f_rotate.open(fn_rorate.c_str());
        ts.rotate_s2sdot = readMatrix(f_rotate);
        f_rotate.close();
        f_trans.open(fn_trans.c_str());
        est_trans = readMatrix(f_trans);
        std::cout<<"init position "<<est_trans<<std::endl;
        f_trans.close();
//        est_trans(0) += 0.5*(double) rand() / (RAND_MAX);
//        est_trans(1) += 0.5*(double) rand() / (RAND_MAX);
//        est_trans(2) += 0.5*(double) rand() / (RAND_MAX);

    }
    else{
        std::cout<<"tool parameters files are not exist"<<std::endl;
        exit(0);
    }

}

void ManipTool::store_parameters(std::string fn_nv, std::string fn_rorate,std::string fn_trans){
    if((is_file_exist(fn_nv.c_str()) == true)&&(is_file_exist(fn_rorate.c_str()) == true)&&\
            (is_file_exist(fn_trans.c_str()) == true)){
        f_nv.open(fn_nv.c_str());
        f_nv<<ts.rel_o<<std::endl;
        f_nv.close();
        f_rotate.open(fn_rorate.c_str());
        f_rotate<<ts.rotate_s2sdot<<std::endl ;
        f_rotate.close();
        f_trans.open(fn_trans.c_str());
        f_trans<<est_trans<<std::endl ;
        f_trans.close();
    }
    else{
        std::cout<<"tool parameters files are not exist"<<std::endl;
        exit(0);
    }
}



