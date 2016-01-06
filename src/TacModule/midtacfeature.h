#ifndef MIDTACFEATURE_H
#define MIDTACFEATURE_H
#include <Eigen/Core>
#include <iostream>
#include <Eigen/SVD>

class MidTacFeature
{
public:
    MidTacFeature(Eigen::MatrixXd data,int len);
    MidTacFeature(int len);
    //assume all dataset are available
    Eigen::MatrixXd getSlope_batch();
    //assume that slope is obtained while the data is increamentally sampled.
    Eigen::Vector3d getSlope(Eigen::Vector3d u);
private:
    Eigen::MatrixXd dataset;
    int filter_len;
    Eigen::MatrixXd data_m;
    Eigen::Matrix3d covar_m;
    Eigen::Matrix3d bsxfun_min(Eigen::MatrixXd used_data);
    Eigen::Vector3d median(Eigen::MatrixXd used_data);
    bool isZerorow();

};

#endif // MIDTACFEATURE_H
