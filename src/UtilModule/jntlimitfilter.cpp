#include "jntlimitfilter.h"
#include <limits>
#include <math.h>

#define OVERALL_DAMP 0.9
#define JERK_DAMP OVERALL_DAMP*0.0015
#define ACCEL_DAMP OVERALL_DAMP*0.9
#define VELOCITY_DAMP OVERALL_DAMP

JntLimitFilter::JntLimitFilter(double t)
{
    jerk_limits[0] = JERK_DAMP*20.944;
    jerk_limits[1] = JERK_DAMP*20.944;
    jerk_limits[2] = JERK_DAMP*27.925;
    jerk_limits[3] = JERK_DAMP*27.925;
    jerk_limits[4] = JERK_DAMP*43.633;
    jerk_limits[5] = JERK_DAMP* 76.794;
    jerk_limits[6] = JERK_DAMP*76.794;

    accel_limits[0] = ACCEL_DAMP*20.944;
    accel_limits[1] = ACCEL_DAMP*20.944;
    accel_limits[2] = ACCEL_DAMP*27.925;
    accel_limits[3] =  ACCEL_DAMP*27.925;
    accel_limits[4] = ACCEL_DAMP*43.633;
    accel_limits[5] = ACCEL_DAMP* 76.794;
    accel_limits[6] = ACCEL_DAMP*76.794;

    velocity_limits[0] = VELOCITY_DAMP*2.0944;
    velocity_limits[1] = VELOCITY_DAMP*2.0944;
    velocity_limits[2] = VELOCITY_DAMP* 2.7925;
    velocity_limits[3] = VELOCITY_DAMP*  2.7925;
    velocity_limits[4] = VELOCITY_DAMP* 4.3633;
    velocity_limits[5] = VELOCITY_DAMP* 3.8397;
    velocity_limits[6] = VELOCITY_DAMP*3.8397;

    cycle_time = t;
    speedlimit = 0.2;
    for (int i=0; i < 7; i++){
        lastJerk[i] = 0;
        lastAccel[i] = 0;
        lastCorr[i] = 0;
    }
}

void JntLimitFilter::get_filtered_value(double *v_in, double* v_out){
    double factor = 1.0;
    double temp;
    double min_factor = 1.0;
    for (int i = 0; i < 7; i++){
        if (fabs(factor*v_in[i]-lastCorr[i]) > std::numeric_limits<double>::epsilon()){
            if ( (temp = (speedlimit*accel_limits[i]*cycle_time / fabs(factor*v_in[i]-lastCorr[i])))  < factor){
                factor = temp;
            }
        }
        if (fabs (factor*v_in[i]) > std::numeric_limits<double>::epsilon()){
            if ((temp = (speedlimit*velocity_limits[i]*cycle_time / fabs (factor*v_in[i])))  < factor){
                factor = temp;
            }
        }
    }
    for (int i=0; i < 7; i++){
        v_out[i] = v_in[i]*factor;
    }
    {
        double f;
        double f_min=1.0;
        int j;
        for (j=0; j < 7; j++){
            f = (speedlimit*jerk_limits[j]*cycle_time) / fabsf ( v_out[j] - lastCorr[j] - lastAccel[j]);
            if (f < f_min)
                f_min = f;
        }
        min_factor = f_min;
    }
    for (int i=0; i < 7; i++){
        v_out[i] = min_factor*v_out[i] + ((1.0-min_factor)*lastCorr[i]);
    }
    for (int i = 0; i < 7; i++){
        lastJerk[i] = lastAccel[i] - (lastCorr[i] - v_out[i]);
        lastAccel[i] = lastCorr[i] - v_out[i];
        lastCorr[i] = v_out[i];
    }
}
