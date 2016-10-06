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
    for (std::vector<Eigen::Vector2d>::iterator it = ps.begin(); it != ps.end(); ++it){
        sum_x += (*it)(0); sum_y += (*it)(1); sum_xy += (*it)(0) * (*it)(1); sum_x2 += (*it)(0) * (*it)(0);
    }
    std::cout<<"length of the dataset "<<ps.size()<<std::endl;
    // means
    double mean_x = sum_x / ps.size();
    double mean_y = sum_y / ps.size();

    double varx = sum_x2 - sum_x * mean_x;
    double cov = sum_xy - sum_x * mean_y;

    if( std::fabs(cov) < 1e-7 ) {
          // Fail: it seems a vertical line
        rgp.k = M_PI/2;
        rgp.b = 0;
        std::cout<<"the line is a vertical line"<<std::endl;
        }

    rgp.k = cov / varx;
    rgp.b = mean_y - rgp.k * mean_x;

    double a;
    rgp.deltay = (*(ps.end()-1))(1) - (*ps.begin())(1);
    rgp.deltax = (*(ps.end()-1))(0) - (*ps.begin())(0);
    std::cout<<"cov, varx are "<<cov<<","<<varx<<std::endl;
    std::cout<<"end is "<<(*(ps.end()-1))(1)<<","<<(*(ps.end()-1))(0)<<std::endl;
    std::cout<<"start is "<<(*ps.begin())(1)<<","<<(*ps.begin())(0)<<std::endl;
    a = atan2(rgp.deltay,rgp.deltax);
    std::cout<<"atan2 "<<a<<std::endl;
    rgp.sign_k = sign(a);

    //clear everything for the next computation
    ps.clear();
    sum_x = 0;
    sum_xy = 0;
    sum_x2 = 0;
    sum_y = 0;
    cov_xy = 0;
    return rgp;
}



double Regression2d::get_rotation_angle(EXPDIR ed){
    double DeltaGama;
    switch ( ed ) {
    case XP:
        if(rgp.sign_k == 1){
            if((rgp.deltay>0)&&(rgp.deltax>0)){
                DeltaGama = atan(rgp.k) + M_PI;
            }
            else{
                DeltaGama = atan(rgp.k);
            }
        }
        else{
            if((rgp.deltay<0)&&(rgp.deltax>0)){
                DeltaGama = atan(rgp.k) + M_PI;
            }
            else{
                DeltaGama = atan(rgp.k);
            }
        }
        return DeltaGama;
        break;
    case XN:
        if((rgp.deltay>0)&&(rgp.deltax>0)){
            DeltaGama = atan(rgp.k);
        }
        else if((rgp.deltay>0)&&(rgp.deltax<0)){
            DeltaGama = atan(rgp.k)+  M_PI;
        }
        else if((rgp.deltay<0)&&(rgp.deltax>0)){
            DeltaGama = atan(rgp.k);
        }
        else{
            DeltaGama = atan(rgp.k) +  M_PI;
        }
        return DeltaGama;
        break;
    case YP:
        if((rgp.deltay>0)&&(rgp.deltax>0)){
            DeltaGama = atan(rgp.k)+M_PI/2;
        }
        else if((rgp.deltay>0)&&(rgp.deltax<0)){
            DeltaGama = atan(rgp.k)-M_PI/2;
        }
        else if((rgp.deltay<0)&&(rgp.deltax>0)){
            DeltaGama = atan(rgp.k) + M_PI/2;
        }
        else{
            DeltaGama = atan(rgp.k) -  M_PI/2;
        }
        return DeltaGama;
        break;
    case YN:
        if((rgp.deltay>0)&&(rgp.deltax>0)){
            DeltaGama = atan(rgp.k)-M_PI/2;
        }
        else if((rgp.deltay>0)&&(rgp.deltax<0)){
            DeltaGama = atan(rgp.k)+M_PI/2;
        }
        else if((rgp.deltay<0)&&(rgp.deltax>0)){
            DeltaGama = atan(rgp.k) - M_PI/2;
        }
        else{
            DeltaGama = atan(rgp.k) +  M_PI/2;
        }
        return DeltaGama;
        break;
    default:
        std::cout<<"wrong direction you ar using";
        return 0;
        break;
    }
}
