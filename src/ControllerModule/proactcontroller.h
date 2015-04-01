/*
    This file is part of VTFS--Visuo-Tactile-Force-Servoing.

    VTFS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    VTFS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CBF.  If not, see <http://www.gnu.org/licenses/>.


    Copyright 2009, 2010 Qiang Li
*/
#ifndef PROACTCONTROLLER_H
#define PROACTCONTROLLER_H
#include "actcontroller.h"
#include "RobotModule/Robot.h"
#include "TaskModule/kukaselfctrltask.h"


class ProActController : public ActController
{
public:
    ProActController(ParameterManager &p);
    //hold the object in its current pose
    void update_robot_reference(Robot *);
    //move the object to the desired reference designed by Task
    void update_robot_reference(Robot *, Task *);
    void update_robot_reference(Robot *, Task *,myrmex_msg *){}
    void update_robot_reference(Robot *, Task *, Eigen::VectorXd, RobotState*){}
    void update_controller_para(Eigen::Vector3d,PROTaskNameT);
    void update_controller_para(std::pair<Eigen::Vector3d,double>&,PROTaskNameT);
    void update_controller_para(Eigen::Vector3d vel,Eigen::Vector3d r_vel,PROTaskNameT tnt);
    void update_controller_para_stiffness();
    void updateTacServoCtrlParam(TACTaskNameT){}
    void updateProServoCtrlParam(PROTaskNameT tnt);
    void updateForceServoCtrlParam(FORCETaskNameT fnt){}
    void set_pm(ParameterManager &p);
    void get_desired_lv(Robot *, Task *);
    void get_desired_lv(Robot *, Task *, myrmex_msg *){}
    void get_desired_lv(Robot *, Task *, Eigen::VectorXd kukaft,RobotState*){}
    void get_lv(Eigen::Vector3d& lv, Eigen::Vector3d& ov);
    void set_init_TM(Eigen::Matrix3d tm) {m_init_tm = tm;}
private:
    void get_joint_position();
    void set_eff_command(Eigen::Vector3d p, Eigen::Vector3d o);
    void set_eff_command(Eigen::Vector3d p, Eigen::Matrix3d o);
    void initProServoCtrlParam(PROTaskNameT);
    //!select matrix
    std::map<PROTaskNameT, Eigen::MatrixXd> psm;
    //!pose kp parameter
    std::map<PROTaskNameT, Eigen::MatrixXd> Kpp;
    //!pose kp parameter
    std::map<PROTaskNameT, Eigen::Matrix3d> Kop;
    Eigen::Vector3d llv_pro,lov_pro;
    Eigen::VectorXd lv_pro;
};

#endif // PROACTCONTROLLER_H
