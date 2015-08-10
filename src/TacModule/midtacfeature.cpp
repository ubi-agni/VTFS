#include "midtacfeature.h"

MidTacFeature::MidTacFeature(Eigen::MatrixXd data,int len)
{
    dataset = data;
    filter_len = len;
    data_m.setZero(filter_len,3);
    covar_m.setZero();
}

MidTacFeature::MidTacFeature(int len){
    filter_len = len;
    data_m.setZero(filter_len,3);
    covar_m.setZero();
}

Eigen::MatrixXd MidTacFeature::getSlope_batch(){
    double scale;
    Eigen::MatrixXd u_m;
    u_m.setZero(dataset.rows() - filter_len,3);
    scale = 0.001;
    for(int i = 0; i < dataset.rows() - filter_len; i++){
            data_m = dataset.block(i,0,filter_len,3);
            covar_m = bsxfun_min(data_m);
            Eigen::JacobiSVD<Eigen::MatrixXd> svd(covar_m,Eigen::ComputeThinU | Eigen::ComputeThinV);
            u_m.row(i) = (-1) * scale * (svd.matrixV().col(0)).transpose();
    }
    return u_m;
}

Eigen::Matrix3d MidTacFeature::bsxfun_min(Eigen::MatrixXd used_data){
    Eigen::Vector3d v;
    Eigen::MatrixXd pre_prodata;
    Eigen::Matrix3d covar;
    v.setZero();
    pre_prodata.setZero(used_data.rows(),used_data.cols());
    v = median(used_data);
    for(int i = 0; i < used_data.rows(); i++){
        pre_prodata.row(i) = used_data.row(i) - v.transpose();
    }

    covar = 1.0/filter_len * pre_prodata.transpose() * pre_prodata;
    return covar;
}

Eigen::Vector3d MidTacFeature::median(Eigen::MatrixXd used_data){
    Eigen::Vector3d v;
    Eigen::Vector3d sum_v;
    sum_v.setZero();
    v.setZero();
    for(int i = 0; i < used_data.rows(); i++){
        sum_v += used_data.row(i);
    }
    v = sum_v / used_data.rows();
    return v;
}

Eigen::Vector3d MidTacFeature::getSlope(Eigen::Vector3d u){
    Eigen::Vector3d v;
    double scale;
    scale = 0.001;
    v.setZero();
    //fill in the new data, get rid of the old data
    for(int i = 0; i < filter_len-1; i++){
        data_m.row(i) = data_m.row(i+1);
    }
    data_m.row(filter_len-1) = u.transpose();
    if(isZerorow() == false){
        covar_m = bsxfun_min(data_m);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(covar_m,Eigen::ComputeThinU | Eigen::ComputeThinV);
        v = (-1) * scale * (svd.matrixV().col(0)).transpose();
    }
    return v;
}
bool MidTacFeature::isZerorow(){
    bool zeroflag;
    zeroflag = false;
    for(int i = 0; i < filter_len-1; i++){
        if((fabs(data_m(i,0))<1e-5)&&(fabs(data_m(i,1))<1e-5)&&(fabs(data_m(i,2))<1e-5)){
            zeroflag = true;
        }
    }
    return zeroflag;
}
