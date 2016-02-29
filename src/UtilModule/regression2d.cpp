#include "regression2d.h"
#define sign(a) ( ( (a) < 0 )  ?  -1   : ( (a) > 0 ) )

#include <math.h>       /* atan2 */


Regression2d::Regression2d()
{
    data_num = 0;
    sum_x = 0;
    sum_xy = 0;
    sum_x2 = 0;
    sum_y = 0;
    cov_xy = 0;
    varx = 0;
}

Reg_param Regression2d::get_kb_batch(std::vector<Eigen::Vector2d> ps){
    Reg_param rgp;
    for (std::vector<Eigen::Vector2d>::iterator it = ps.begin(); it != ps.end(); ++it){
        sum_x += (*it)(0); sum_y += (*it)(1); sum_xy += (*it)(0) * (*it)(1); sum_x2 += (*it)(0) * (*it)(0);
    }
    // means
    double mean_x = sum_x / ps.size();
    double mean_y = sum_y / ps.size();

    double varx = sum_x2 - sum_x * mean_x;
    double cov = sum_xy - sum_x * mean_y;

    rgp.k = cov / varx;
    rgp.b = mean_y - rgp.k * mean_x;

    double a;
    rgp.deltay = (*ps.end())(1) - (*ps.begin())(1);
    rgp.deltax = (*ps.end())(0) - (*ps.begin())(0);
    a = atan2(rgp.deltay,rgp.deltax);
    std::cout<<"atan2 "<<a*180.0/M_PI<<std::endl;
    rgp.sign_k = sign(a);

    //clear everything for the next computation
    ps.clear();
    sum_x = 0;
    sum_xy = 0;
    sum_x2 = 0;
    sum_y = 0;
    cov_xy = 0;
    varx = 0;
    return rgp;
}









