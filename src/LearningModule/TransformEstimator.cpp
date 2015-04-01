#include "TransformEstimator.h"

TFEstimator::TFEstimator() {
    R.setZero();
    t.setZero();
    center_src.setZero();
    center_dst.setZero();
    pdev.setZero();
    qdev.setZero();
    M.setZero();
}
void TFEstimator::load(Eigen::Vector3d pi, Eigen::Vector3d qi){
    src.push_back(pi);
    dst.push_back(qi);
}
void TFEstimator::clear(){
    src.clear();
    dst.clear();
    center_src.setZero();
    center_dst.setZero();
    pdev.setZero();
    qdev.setZero();
    M.setZero();
}

bool TFEstimator::EstRt(Eigen::Matrix3d& tm, Eigen::Vector3d& position){
    int ps_size = 0;
    ps_size = src.size();

    for (int i=0; i<ps_size; ++i)
    {
        center_src += src[i];
        center_dst += dst[i];
    }
    center_src /= (double)ps_size;
    center_dst /= (double)ps_size;


    Eigen::MatrixXd S(ps_size, 3), D(ps_size, 3);
    for (int i=0; i<ps_size; ++i)
    {
        for (int j=0; j<3; ++j)
            S(i, j) = src[i][j] - center_src[j];
        for (int j=0; j<3; ++j)
            D(i, j) = dst[i][j] - center_dst[j];
    }
    Eigen::MatrixXd Dt = D.transpose();
    M = Dt*S;
    Eigen::Matrix3d W, U, V;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd;
//    Eigen::MatrixXd H_(3, 3);
//    for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) H_(i, j) = M(i, j);
    svd.compute(M, Eigen::ComputeThinU | Eigen::ComputeThinV );
    if (!svd.computeU() || !svd.computeV()) {
        std::cerr << "decomposition error" << std::endl;
        return false;
    }
    Eigen::Matrix3d Vt = svd.matrixV().transpose();
    R = svd.matrixU()*Vt;
    tm = R;
    t = center_dst - R*center_src;
    position = t;
    return true;
}
