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
#pragma once

#include <Eigen/Dense>
#include "RobotModule/Robot.h"
#include "UtilModule/msgcontenttype.h"
#include <map>
#include "UtilModule/Util.h"
#include <utility> //for std::pair

class Robot;

class RobotState{
public:
    RobotState();
    RobotState(Robot *);
    void updated(Robot *);
    Eigen::Vector3d EstCtcPosition_Ref(Robot *, myrmex_msg&);
    Eigen::Vector3d EstCtcPosition_KUKAPALM(Robot *, myrmex_msg&);
    Eigen::Vector3d EstCtcPosition_KUKAFINGER();
    Eigen::Vector3d EstRobotEefLVel_Ref(Robot *);
    std::pair<Eigen::Vector3d,Eigen::Vector3d> EstRobotEefRVel_InitF(Robot *, bool&);
    Eigen::Vector3d EstCtcNormalVector_Ref(Robot *, myrmex_msg&);
    Eigen::Vector3d EstRobotEefAcc_Ref(Robot *r);

    //old rebacode endeffector pose
	Eigen::Vector3d position;
    Eigen::Vector3d position_tacfoam;
	Eigen::Vector3d orientation;
	Eigen::Matrix3d eigen_orientation;
	Eigen::VectorXd JntPosition;
    Eigen::Vector3d ctposition;
	bool contactflag;
	double gq;

    //state of whole kinematics chain
    std::map<int, std::string> jntnum2name;
    std::map<std::string, Eigen::Vector3d> robot_position;
    std::map<std::string, Eigen::Matrix3d> robot_orien;
    Eigen::Vector3d eef_vel_g;
    Eigen::Vector3d eef_acc_g;
    Eigen::Vector3d eef_position_lastT;
    Eigen::Vector3d eef_vel_g_lastT;
    Eigen::Vector3d eef_acc_g_lastT;
    Eigen::Matrix3d eef_orientation_lastT;
    Eigen::VectorXd JntPosition_act;
    double JntPosition_mea[7];
    Eigen::Vector3d contact_position;
    Eigen::Vector3d contact_nv;
    Eigen::MatrixXd old_hm,cur_hm;
    Eigen::MatrixXd adj_matrix;

    //rotation angle rate estimation
    Eigen::Vector3d omega;
    double rate_old;
    double theta_old;
    void Est_eef_twist(Robot* r,Eigen::Vector3d& v, Eigen::Vector3d& omega);
    void twist_ltog(Eigen::Vector3d gv, Eigen::Vector3d go,Eigen::Vector3d& lv, Eigen::Vector3d& lo);
};

class EnvState{
public:
    EnvState(){
        marker_p.setZero();
        marker_o.setZero();
    }
    //marker position
    Eigen::Vector3d marker_p;
    Eigen::Matrix3d marker_o;
};
