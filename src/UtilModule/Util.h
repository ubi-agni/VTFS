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

#include <iostream>
#include <deque>
#include <time.h> //for program running test(cpu consuming time)
#include <sys/time.h>//for program running test(realtime consuming test)
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <utility>

extern Eigen::Vector3d euler2axisangle(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern Eigen::Vector3d g_euler2axisangle(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern Eigen::Vector3d tm2axisangle(Eigen::Matrix3d tm);
extern std::pair<Eigen::Vector3d,double>  tm2axisangle_4(Eigen::Matrix3d,bool&);
extern Eigen::Matrix3d g_euler2tm(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern Eigen::Matrix3d euler2tm(Eigen::Vector3d la,Eigen::Matrix3d tm);
extern double _smooth_filter(std::deque<double> t);
extern Eigen::Matrix3d GetSkrewFromVector(Eigen::Vector3d vec);
extern long long timeval_diff(struct timeval *difference, struct timeval *end_time, struct timeval *start_time);
extern Eigen::Matrix3d AlignVec(Eigen::Vector3d cur, Eigen::Vector3d des);
extern Eigen::Vector3d kdl2eigen_position(const KDL::Frame& f);
extern Eigen::Matrix3d kdl2eigen_orien(const KDL::Frame& f);
extern void global2local(Eigen::Vector3d, Eigen::Matrix3d, Eigen::Vector3d &);
extern std::pair<Eigen::Vector3d,double> omega_transform(std::pair<Eigen::Vector3d,Eigen::Vector3d>,Eigen::Matrix3d);
extern void gen_hm(Eigen::Matrix3d m,Eigen::Vector3d v,Eigen::MatrixXd& hm);
extern Eigen::Vector3d skewtovector(Eigen::Matrix3d m);
extern Eigen::Matrix3d vectortoskew(Eigen::Vector3d v);
