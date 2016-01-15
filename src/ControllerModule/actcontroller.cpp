#include "actcontroller.h"

#include "CtrlParam.h"


//taskctrlpara ActController::task_ctrl_param[CONTACT_POINT_TRACKING];
//taskctrlpara ActController::task_ctrl_param[CONTACT_FORCE_TRACKING];
//taskctrlpara ActController::task_ctrl_param[CONTACT_POINT_FORCE_TRACKING];
//taskctrlpara ActController::task_ctrl_param[Z_ORIEN_TRACKING];
//taskctrlpara ActController::task_ctrl_param[LINEAR_TRACKING];
//taskctrlpara ActController::task_ctrl_param[COVER_OBJECT_SURFACE];
//taskctrlpara ActController::task_ctrl_param[OBJECT_SURFACE_EXPLORING];

Eigen::Vector3d ActController::llv;
Eigen::Vector3d ActController::lov;

ActController::ActController(ParameterManager& p)
{
    glv.setZero();
    gov.setZero();
    eff_p_command.setZero();
    eff_o_command_vec.setZero();
    eff_o_command_mat.setZero();
    m_init_tm.setIdentity();


    m_llv_limit.setZero();
    m_llv_limit(0) = 0.05;
    m_llv_limit(1) = 0.05;
    m_llv_limit(2) = 0.05;
    m_euler_limit(0) = 0.6;
    m_euler_limit(1) = 0.6;
    m_euler_limit(2) = 0.6;
    pose_o_eigen_l.setZero();
    pm = p;
}



void ActController::local_to_global(const Eigen::Vector3d p_in, const Eigen::Matrix3d o_in,\
                                    const Eigen::Vector3d lv, const Eigen::Vector3d ov,\
                                    Eigen::Vector3d& p_out, Eigen::Vector3d& o_out)
{
    p_out = p_in + o_in * lv;
    pose_o_eigen_l += 0.004*ov;
    o_out = euler2axisangle(pose_o_eigen_l,m_init_tm);
}



void ActController::limit_vel(Eigen::Vector3d lim,\
                                   Eigen::Vector3d& llv, Eigen::Vector3d& lov){
    for(int i = 0; i < 3; i++){
        if(llv(i) > lim(i)){
            llv(i) = lim(i);
        }
        if(llv(i) <-lim(i)){
            llv(i) = -lim(i);
        }
    }
}

void ActController::limit_eef_euler(Eigen::Vector3d lim){
    std::cout<<"pose_o_eigen_l "<<pose_o_eigen_l<<std::endl;
    for(int i = 0; i < 3; i++){
        if(pose_o_eigen_l(i) > lim(i)){
            pose_o_eigen_l(i) = lim(i);
        }
        if(pose_o_eigen_l(i) < -lim(i)){
            pose_o_eigen_l(i) = -lim(i);
        }
    }
}


