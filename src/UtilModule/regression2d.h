#ifndef REGRESSION2D_H
#define REGRESSION2D_H
#include <Eigen/Core>
#include <iostream>
#include <vector>

//this class is designed for 2d linear regression to compute
//use the math from https://en.wikipedia.org/wiki/Simple_linear_regression#Fitting_the_regression_line
//a simplified version is impelmented following http://stackoverflow.com/questions/11449617/how-to-fit-the-2d-scatter-data-with-a-line-with-c
struct Reg_param{
    double k;
    double b;
    int sign_k;
    double deltay,deltax;
    Reg_param(){
        k = 0;
        b = 0;
        sign_k = 0;
        deltax = 0;
        deltay = 0;
    }
};

class Regression2d
{
public:
    Regression2d();
    //get the linear slope [0] and intercept [1]
    Reg_param get_kb_batch(std::vector<Eigen::Vector2d>);
private:
    int data_num;
    double sum_x,sum_y,sum_xy,sum_x2;
    double cov_xy,varx;
};

#endif // REGRESSION2D_H
