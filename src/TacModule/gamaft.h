#ifndef GAMAFT_H
#define GAMAFT_H
#include <Eigen/Dense>

class gamaFT
{
public:
    gamaFT();
    Eigen::Vector3d raw_ft_f;
    Eigen::Vector3d raw_ft_t;
    Eigen::Vector3d calib_ft_f;
    Eigen::Vector3d calib_ft_t;
    Eigen::Vector3d mean_ft_f;
    Eigen::Vector3d mean_ft_t;
    Eigen::Vector3d std_ft_f;
    Eigen::Vector3d std_ft_t;
    Eigen::Vector3d filtered_gama_f,filtered_gama_t;
    void calibFT(int);
};

#endif // GAMAFT_H
