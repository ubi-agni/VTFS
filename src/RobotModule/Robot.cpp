#include "Robot.h"

Robot::Robot()
{
    m_TM_eigen.setZero();
    m_p_eigen.setZero();
    m_init_tm.setZero();
    q.resize(7);
    eff_force.setZero();
    eff_torque.setZero();
}

void Robot::set_cart_command(double *c){
    for(int i = 0; i < 6; i++)
        cart_command[i] = *(c+i);
}

Eigen::Vector3d Robot::get_cur_cart_p(){
    return m_p_eigen;
}
Eigen::Matrix3d Robot::get_cur_cart_o(){
    return m_TM_eigen;
}
