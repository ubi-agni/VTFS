/*
 ============================================================================
 Name        : ComOkc.h
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : Calibrate front terminal gui the kuka setup in new building
 ============================================================================
 */

#ifndef COMOKC_H
#define COMOKC_H

#include <fri_okc_comm.h>
#include <fri_okc_types.h>
#include <fri_okc_helper.h>
#include "ComInterface.h"
#include <string.h>
#include <iostream>
#include <stdexcept>
#include <sys/time.h>//for program running test(realtime consuming test)

#include "cd_dynamics/CDDynamics.hpp"

#define LEFT_ROBOT_ID   1
#define RIGHT_ROBOT_ID  2


class ComOkc : public ComInterface
{
public:
    ComOkc(RobotNameT connectToRobot, const char* hostname, const char* port);
    void connect();
    bool isConnected();
    void waitForFinished();
    int getrobot_id();
    bool controller_update;
    float jnt_position_act[7];
    float jnt_position_mea[7];
    bool data_available;
    fri_float_t jnt_command[7];
    fri_float_t new_cartpos[12];
    coords_t* ft;
    void set_stiffness(double *s, double *d);
    void set_cp_stiffness(double *cps,double *cpd);
    void set_cp_ExtTcpFT(double *tcpft);
    void start_brake();
    void release_brake();
    void switch_to_cp_impedance();
    void switch_to_jnt_impedance();
    void request_monitor_mode();
    void sleep_cycle_time();
private:
    static int instance_count;
    char hostname[16];
    char port[6];
    bool legacy_axis_mode;
    static okc_handle_t* okc;
    void initServer();
    void bindToName(const char* name);
    void registerCallback(int index);
    int robot_id;
    void yield();
    RobotNameT rn,rn1,rn2;
    static int left_okcAxisAbsCallback (void* priv, const fri_float_t* pos_act, fri_float_t* new_pos);
    static int right_okcAxisAbsCallback (void* priv, const fri_float_t* pos_act, fri_float_t* new_pos);
    static int okcCartposAxisAbsCallback (void* priv, const fri_float_t* cartpos_act, fri_float_t* axispos_act,fri_float_t* new_cartpos, fri_float_t* new_axispos);
    void get_cycle_time();
    lbr_axis_t astiffness;
    lbr_axis_t adamping;
    coords_t cpstiff;
    coords_t cpdamping;
    coords_t extft;

    CDDynamics *left_filter;
    CDDynamics *right_filter;

    Eigen::VectorXd mLeftCurrentJoint;
    Eigen::VectorXd mRightCurrentJoint;
    Eigen::VectorXd mLeftDesiredJoint;
    Eigen::VectorXd mRightDesiredJoint;

    bool mIsLeftArmInitialized;
    bool mIsRightArmInitialized;
};

#endif // COMOKC_H
