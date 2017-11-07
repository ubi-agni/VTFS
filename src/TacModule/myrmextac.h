#ifndef MYRMEXTAC_H
#define MYRMEXTAC_H
#include "UtilModule/msgcontenttype.h"
#include "UtilModule/TemporalSmoothingFilter.h"

class MyrmexTac
{
public:
    MyrmexTac();
    double cog_x,cog_y;
    double old_cog_x;
    double old_cog_y;
    int contactnum;
    bool contactflag;
    double cf;
    double lineorien;
    void update_initial_data(myrmex_msg msg);
    void cal_ctc_lv();
    Eigen::Vector3d ctc_vel;
    TemporalSmoothingFilter<Eigen::Vector3d>* ctc_vel_filter;
    TemporalSmoothingFilter<double>* line_orien_filter;
};

#endif // MYRMEXTAC_H
