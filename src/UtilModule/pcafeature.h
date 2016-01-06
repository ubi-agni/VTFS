#ifndef PCAFEATURE_H
#define PCAFEATURE_H
#include <Eigen/Core>
#include <iostream>
#include <Eigen/SVD>
#include <deque>

class PCAFeature
{
public:
    PCAFeature(int len);
    Eigen::Vector3d getSlope_batch();
private:
    Eigen::MatrixXd dataset;
    void GetData(std::deque<Eigen::Vector3d>);
    int filter_len;
    Eigen::MatrixXd data_m;
    Eigen::Matrix3d covar_m;
    Eigen::Matrix3d bsxfun_min(Eigen::MatrixXd used_data);
    Eigen::Vector3d meanvalue(Eigen::MatrixXd used_data);
};

#endif // PCAFEATURE_H
