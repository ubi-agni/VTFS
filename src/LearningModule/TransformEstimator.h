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
#ifndef TFESTIMATOR_H
#define TFESTIMATOR_H

/*This is the implementation of rigid transform estimator
using the idea SVD method
reference paper:
(1) Fast and Globally Convergent Pose Estimation From Video Images
(2) Least Squares Fitting of Two 3-D Point Sets
*/
#include <iostream>
#include <vector>
#include <Eigen/SVD>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Geometry>

class TFEstimator{
public:
    TFEstimator();
    std::vector<Eigen::Vector3d> src,dst;
	Eigen::Matrix3d R;
	Eigen::Vector3d t;
	void load(Eigen::Vector3d pi, Eigen::Vector3d qi);
	bool EstRt(Eigen::Matrix3d &tm, Eigen::Vector3d &position);
    void clear();
private:
    Eigen::Vector3d center_src,center_dst;
    Eigen::Vector3d pdev,qdev;
    Eigen::Matrix3d M;

};

#endif // TFESTIMATOR_H
