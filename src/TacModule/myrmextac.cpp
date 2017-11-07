#include "myrmextac.h"

MyrmexTac::MyrmexTac()
{
    cog_x = -1;
    cog_y = -1;
    old_cog_x = -1;
    old_cog_y = -1;
    contactnum = 0;
    contactflag = false;
    cf = 0;
    lineorien = 0;
    ctc_vel.setZero();
    ctc_vel_filter = new TemporalSmoothingFilter<Eigen::Vector3d>(200,Average,Eigen::Vector3d(0,0,0));
    line_orien_filter = new TemporalSmoothingFilter<double>(20,Average,0);
}

//use the myrmex sampling frequency to update myrmex data.
void MyrmexTac::update_initial_data(myrmex_msg msg){
    cog_x = msg.cogx;
    cog_y = msg.cogy;
    contactnum = msg.contactnum;
    contactflag = msg.contactflag;
    cf = msg.cf;
    lineorien = line_orien_filter->push(msg.lineorien);
}


void MyrmexTac::cal_ctc_lv(){
    Eigen::Vector3d tmp_ctc;
    tmp_ctc.setZero();
    ctc_vel(0) = cog_x -old_cog_x;
    ctc_vel(1) = cog_y - old_cog_y;
    tmp_ctc = ctc_vel_filter->push(ctc_vel);
    ctc_vel = tmp_ctc;
    old_cog_x = cog_x;
    old_cog_y = cog_y;
}
