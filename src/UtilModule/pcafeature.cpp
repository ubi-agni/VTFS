#include "pcafeature.h"


PCAFeature::PCAFeature(int len){
    filter_len = len;
    data_m.setZero(filter_len,3);
    covar_m.setZero();
}

void PCAFeature::GetData(std::deque<Eigen::Vector3d> p){
    for(unsigned int i = 0; i < p.size(); i++)
        data_m.row(i) = p.at(i).transpose();
//    std::cout<<"all data traecjtory" <<std::endl;
//    std::cout<<data_m;
}

Eigen::Vector3d PCAFeature::getSlope_batch(){
    double scale;
    Eigen::Vector3d u_m;
    Eigen::Vector3d nv_dir;
    nv_dir.setZero();
    u_m.setZero();
    //compute the major vector using pca.
    scale = 0.001;
    covar_m = bsxfun_min(data_m);
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(covar_m,Eigen::ComputeThinU | Eigen::ComputeThinV);
    u_m = (-1) * scale * (svd.matrixV().col(0)).transpose();
    //use the hieristic info: normal direction of tool is the same from start point
    //to end point in the data set
    nv_dir = (data_m.row(data_m.rows()-1) - data_m.row(0));
    nv_dir.normalize();
    if(u_m.dot(nv_dir) > 0)
        return u_m;
    else{
        return -1.0 * u_m;
    }
}

Eigen::Matrix3d PCAFeature::bsxfun_min(Eigen::MatrixXd used_data){
    Eigen::Vector3d v;
    Eigen::MatrixXd pre_prodata;
    Eigen::Matrix3d covar;
    v.setZero();
    pre_prodata.setZero(used_data.rows(),used_data.cols());
    v = meanvalue(used_data);
    for(int i = 0; i < used_data.rows(); i++){
        pre_prodata.row(i) = used_data.row(i) - v.transpose();
    }

    covar = 1.0/filter_len * pre_prodata.transpose() * pre_prodata;
    return covar;
}

Eigen::Vector3d PCAFeature::meanvalue(Eigen::MatrixXd used_data){
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
