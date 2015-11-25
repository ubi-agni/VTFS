#include "gamaft.h"
#include <unistd.h>
#include <math.h>
#include <iostream>

gamaFT::gamaFT()
{
    raw_ft_f.setZero();
    raw_ft_t.setZero();
    calib_ft_f.setZero();
    calib_ft_t.setZero();
    mean_ft_f.setZero();
    mean_ft_t.setZero();
    std_ft_f.setZero();
    std_ft_t.setZero();
}


void gamaFT::calibFT(int sample_num){
    Eigen::MatrixXd sample_f;
    Eigen::MatrixXd mean_f;
    Eigen::MatrixXd sample_t;
    Eigen::MatrixXd mean_t;
    Eigen::MatrixXd std_f;
    Eigen::MatrixXd std_t;
    Eigen::MatrixXd std_f_2;
    Eigen::MatrixXd std_t_2;
    sample_f.setZero(3,sample_num);
    mean_f.setOnes(3,sample_num);
    mean_t.setOnes(3,sample_num);
    sample_t.setZero(3,sample_num);
    std_f.setOnes(3,sample_num);
    std_t.setZero(3,sample_num);
    std_f_2.setOnes(3,sample_num);
    std_t_2.setZero(3,sample_num);
    for(int i = 0; i < sample_num; i++){
        sample_f.col(i) = raw_ft_f;
        sample_t.col(i) = raw_ft_t;
        usleep(10000);
        std::cout<<"data count is "<<i<<std::endl;
    }
    //caculate the mean
    for(int i= 0 ; i < 3; i++){
        mean_ft_f(i) = sample_f.row(i).mean();
        mean_ft_t(i) = sample_t.row(i).mean();
        mean_f.row(i) = mean_ft_f(i) * Eigen::Vector3d::Ones(1,sample_num);
        mean_t.row(i) = mean_ft_t(i) * Eigen::Vector3d::Ones(1,sample_num);
    }

    //caculate the std
    std_f = sample_f - mean_f;
    std_t = sample_t - mean_t;
    std_f_2 = std_f.cwiseProduct(std_f);
    std_t_2 = std_t.cwiseProduct(std_t);

    for(int i = 0; i < 3; i++){
        std_ft_f(i) = sqrt(std_f_2.row(i).sum())/(sample_num-1);
        std_ft_t(i) = sqrt(std_t_2.row(i).sum())/(sample_num-1);
    }
    std::cout<<"force mean is "<<mean_ft_f<<std::endl;
    std::cout<<"torque mean is "<<mean_ft_t<<std::endl;

    std::cout<<"force std is "<<std_ft_f<<std::endl;
    std::cout<<"torque std is "<<std_ft_t<<std::endl;
}
