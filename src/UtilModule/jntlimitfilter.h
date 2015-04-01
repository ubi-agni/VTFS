/*
    This file is part of VTFS--Visuo-Tactile-Force-Servoing.

    VTFS is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 2 of the License, or
    (at your option) any later version.

    VTFS is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with CBF.  If not, see <http://www.gnu.org/licenses/>.


    Copyright 2009, 2010 Qiang Li
*/

#ifndef JNTLIMITFILTER_H
#define JNTLIMITFILTER_H



class JntLimitFilter
{
public:
    JntLimitFilter(double t);
    void get_filtered_value(double *v_in, double *v_out);
private:
    double jerk_limits[7];
    double accel_limits[7];
    double velocity_limits[7];
    double vel_out[7];
    double lastCorr[7];
    double lastAccel[7];
    double lastJerk[7];
    double cycle_time;
    double speedlimit;

};

#endif // JNTLIMITFILTER_H
