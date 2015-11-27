#include "ComOkc.h"
#include "UtilModule/Util.h"
#include "cd_dynamics/CDDynamics.hpp"

int ComOkc::instance_count = 0;
okc_handle_t* ComOkc::okc = NULL;
struct timeval v_last;

int ComOkc::left_okcAxisAbsCallback (void* priv, const fri_float_t* pos_act, fri_float_t* new_pos){
    ComOkc *com_okc_ptr = (ComOkc*) priv;
    fri_float_t jnt_pos[7];

    okc_get_jntpos_act(ComOkc::okc,com_okc_ptr->getrobot_id(), jnt_pos);
    okc_get_ft_tcp_est(ComOkc::okc,com_okc_ptr->getrobot_id(), com_okc_ptr->ft);

    for(int i = 0; i <7; i++){
        com_okc_ptr->jnt_position_act[i] = pos_act[i];
        com_okc_ptr->jnt_position_mea[i] = jnt_pos[i];
    }
//    std::cout<<"controller updated2"<<std::endl;
    // feed back data from FRI is available
    com_okc_ptr->data_available = true;


    //for the starting stage, without this kuka can not switch to the fri mode.
    if (OKC_OK != okc_is_robot_in_command_mode(com_okc_ptr->okc,com_okc_ptr->robot_id)){
        // monitor mode

        for(int i = 0; i <7; i++){
            new_pos[i] = jnt_pos[i];
        }
        com_okc_ptr->controller_update = false;
    }
    else{
        // command mode
        for(int i=0; i<7; i++ ) com_okc_ptr->mLeftCurrentJoint(i) = jnt_pos[i];

        // initialize filter
        if(com_okc_ptr->mIsLeftArmInitialized == false)
        {
            com_okc_ptr->left_filter->SetStateTarget(com_okc_ptr->mLeftCurrentJoint, com_okc_ptr->mLeftCurrentJoint);
            com_okc_ptr->mIsLeftArmInitialized = true;
        }


        // receive the joint command from the main controller
        if(com_okc_ptr->controller_update == true){
            for(int i = 0; i <7; i++){
                com_okc_ptr->mLeftDesiredJoint(i) = com_okc_ptr->jnt_command[i];
            }
            com_okc_ptr->left_filter->SetTarget(com_okc_ptr->mLeftDesiredJoint);
//            std::cout << "new target is set" << std::endl;
        }

        // do filtering
        com_okc_ptr->left_filter->Update();
        com_okc_ptr->left_filter->GetState(com_okc_ptr->mLeftDesiredJoint);

        // send the joint command to the real robot through FRI
        for(int i = 0; i <7; i++){
            com_okc_ptr->jnt_command[i] = com_okc_ptr->mLeftDesiredJoint(i);
            new_pos[i] = com_okc_ptr->mLeftDesiredJoint(i);
        }
        com_okc_ptr->controller_update = false;
    }
    return (OKC_OK);

}
int ComOkc::right_okcAxisAbsCallback (void* priv, const fri_float_t* pos_act, fri_float_t* new_pos){
    ComOkc *com_okc_ptr = (ComOkc*) priv;
    fri_float_t jnt_pos[7];

    okc_get_jntpos_act(ComOkc::okc,com_okc_ptr->getrobot_id(),jnt_pos);
    okc_get_ft_tcp_est(ComOkc::okc,com_okc_ptr->getrobot_id(), com_okc_ptr->ft);

    for(int i = 0; i <7; i++){
        com_okc_ptr->jnt_position_act[i] = pos_act[i];
        com_okc_ptr->jnt_position_mea[i] = jnt_pos[i];
    }
    // feed back data from FRI is available
    com_okc_ptr->data_available = true;

    if (OKC_OK != okc_is_robot_in_command_mode(com_okc_ptr->okc,com_okc_ptr->robot_id)){
        //monitor mode
        for(int i = 0; i <7; i++){
            new_pos[i] = jnt_pos[i];
        }
        com_okc_ptr->controller_update = false;
    }
    else{
        // command mode
        for(int i=0; i<7; i++ ) com_okc_ptr->mRightCurrentJoint(i) = jnt_pos[i];

        // initialize filter
        if(com_okc_ptr->mIsRightArmInitialized == false)
        {
            com_okc_ptr->right_filter->SetStateTarget(com_okc_ptr->mRightCurrentJoint, com_okc_ptr->mRightCurrentJoint);
            com_okc_ptr->mIsRightArmInitialized = true;
        }


        // receive the joint command from the main controller
        if(com_okc_ptr->controller_update == true){
            for(int i = 0; i <7; i++){
                com_okc_ptr->mRightDesiredJoint(i) = com_okc_ptr->jnt_command[i];
            }
            com_okc_ptr->right_filter->SetTarget(com_okc_ptr->mRightDesiredJoint);
//            std::cout << "new target is set" << std::endl;
        }

        // do filtering
        com_okc_ptr->right_filter->Update();
        com_okc_ptr->right_filter->GetState(com_okc_ptr->mRightDesiredJoint);

        // send the joint command to the real robot through FRI
        for(int i = 0; i <7; i++){
            com_okc_ptr->jnt_command[i] = com_okc_ptr->mRightDesiredJoint(i);
            new_pos[i] = com_okc_ptr->mRightDesiredJoint(i);
        }
        com_okc_ptr->controller_update = false;

    }
    return (OKC_OK);
}
int ComOkc::okcCartposAxisAbsCallback (void* priv, const fri_float_t* cartpos_act, fri_float_t* axispos_act,fri_float_t* new_cartpos, fri_float_t* new_axispos){
    struct timeval v_cur, v_old;
    long long intervaltime;
    fri_float_t jnt_pos[7];
    ComOkc *com_okc_ptr = (ComOkc*) priv;
    okc_get_jntpos_act(ComOkc::okc,com_okc_ptr->getrobot_id(),jnt_pos);
    okc_get_ft_tcp_est(ComOkc::okc,com_okc_ptr->getrobot_id(), com_okc_ptr->ft);
    for(int i = 0; i <7; i++){
        com_okc_ptr->jnt_position_act[i] = axispos_act[i];
        com_okc_ptr->jnt_position_mea[i] = jnt_pos[i];
    }

    if (OKC_OK != okc_is_robot_in_command_mode(com_okc_ptr->okc,com_okc_ptr->robot_id)){
        okc_cp_lbr_mnj(jnt_pos,new_axispos);
        okc_cp_cart_frm_dim(cartpos_act,new_cartpos);
        return (OKC_OK);
    }
    intervaltime = 0;
    if(gettimeofday(&v_old,NULL)){
        std::cout<<"gettimeofday function error at the current time"<<std::endl;
    }
    v_last = v_old;
    com_okc_ptr->data_available = true;
    while((intervaltime < 1500)&&(com_okc_ptr->controller_update == false)){
        if(gettimeofday(&v_cur,NULL)){
            std::cout<<"gettimeofday function error at the current time"<<std::endl;
        }
        intervaltime = timeval_diff(NULL,&v_cur,&v_old);
    }
    if(com_okc_ptr->controller_update == true){
        //Todo:use the updated control output
        for(int i = 0; i <7; i++){
            new_axispos[i] = jnt_pos[i];
        }
        for(int i = 0; i <12; i++){
            new_cartpos[i] = cartpos_act[i];
        }
        com_okc_ptr->controller_update = false;
        com_okc_ptr->data_available = false;
    }
    else{
        okc_cp_lbr_mnj(axispos_act,new_axispos);
        okc_cp_cart_frm_dim(cartpos_act,new_cartpos);
        std::cout<<"cartesian stiffness did not get respond in time"<<std::endl;
        com_okc_ptr->controller_update = false;
        com_okc_ptr->data_available = false;
    }
    return (OKC_OK);
}

void ComOkc::waitForFinished(){
    usleep(10000*cycle_time);
}

void ComOkc::get_cycle_time(){
    okc_get_cycle_time(okc, 0, &cycle_time);
}

int ComOkc::getrobot_id(){
    return robot_id;
}

void ComOkc::start_brake(){
    for (int i = 0; i < 5; i++)
    okc_sleep_cycletime(okc,robot_id);
    okc_request_monitor_mode(okc,robot_id);
}

void ComOkc::release_brake(){
    for (int i = 0; i < 5; i++)
    okc_sleep_cycletime(okc,robot_id);
    okc_request_command_mode(okc,robot_id);
}

void ComOkc::connect(){
    int quality = FRI_QUALITY_UNACCEPTABLE;
    std::cout << "waiting for robot to connect" << std::endl;
    while (OKC_OK != okc_is_robot_avail (okc,robot_id)){
        sleep (1);
    }
    if (OKC_OK != okc_get_cycle_time (okc,robot_id,&cycle_time)){
        std::cout << "could not get cycle time" << std::endl;
    }
    else
        std::cout << "Cycle Time is " << cycle_time << std::endl;

    std::cout << "waiting for decent connection quality . . . ";

    while ((FRI_QUALITY_OK != quality) && (FRI_QUALITY_PERFECT != quality) ){
        sleep(1);
        okc_get_connection_quality (okc,robot_id,&quality);
    }
    std::cout << "done!!!" << std::endl << "Requesting command mode . . . ";

    if (legacy_axis_mode){
        okc_switch_to_axis_impedance(okc,robot_id);
        okc_alter_cmdFlags(okc,robot_id,OKC_CMD_FLAGS_AXIS_IMPEDANCE_MODE);
        //       okc_alter_cmdFlags (okc,robot_id,OKC_CMD_FLAGS_CP_AXIS_IMPEDANCE_MODE);
        //       okc_switch_to_cp_impedance(okc,robot_id);
    }
    else{
        okc_alter_cmdFlags (okc,robot_id,OKC_CMD_FLAGS_CP_AXIS_IMPEDANCE_MODE);
        okc_switch_to_cp_impedance(okc,robot_id);
    }
    //okc_switch_to_axis_impedance(okc,robot_id);
//    okc_switch_to_position (okc,robot_id);


    //okc_alter_cmdFlags(okc,robot_id,OKC_CMD_FLAGS_AXIS_IMPEDANCE_MODE);
//    okc_alter_cmdFlags(okc,robot_id,OKC_CMD_FLAGS_POSITION_CONTROL_MODE);

    while (OKC_OK != okc_is_robot_in_command_mode (okc,robot_id)){
        if (OKC_OK != okc_request_command_mode (okc,robot_id)){
            std::cout << "Some error occured. Bailing out" << std::endl;
            exit (EXIT_FAILURE);
        }
        sleep(1);
//        okc_sleep_cycletime(okc,robot_id);
    }
    std::cout << "done" << std::endl;
}

bool ComOkc::isConnected(){
    std::cout<<"okc is available "<<okc_is_robot_avail(okc,robot_id)<<std::endl;
    if (OKC_OK == okc_is_robot_avail(okc,robot_id))
        return true;
    return false;
}

void ComOkc::yield(){
    struct timespec time_val;
    struct timespec time_left;

    time_val.tv_sec = 0;
    time_val.tv_nsec = 100000000; /* 0.1 sec sleep */
    while (0 != nanosleep (&time_val,&time_left)){
        time_val = time_left;
    }
}

void ComOkc::initServer (){
    if (legacy_axis_mode)
        ComOkc::okc = okc_start_server (hostname,port, OKC_MODE_CALLBACK_AXIS_ABS);
    else
        ComOkc::okc = okc_start_server (hostname,port, OKC_MODE_CALLBACK_POS_AXIS_ABS);

    if (NULL == ComOkc::okc){
        std::cerr << "cbf_planner: could not set up OpenKC server thread, exiting" << std::endl;
        exit (EXIT_FAILURE);
    }
}

void ComOkc::bindToName (const char* bindName){
    int i=0;
    char name[256]="";
    std::cout << "Binding to name '" << bindName <<"' . . . " << std::flush;

    if ((OKC_OK != okc_is_robot_avail(okc,i))|| (OKC_OK != okc_get_robot_name(okc,i ,name,256)))
        strncpy (name,"",256);
    while (0 != strncmp(name, bindName,256)){
        i++;
        if (i >= OKC_MAX_ROBOTS)
            i=0;
        if ((OKC_OK != okc_is_robot_avail(okc,i)) || (OKC_OK != okc_get_robot_name(okc,i ,name,256)))
            strncpy (name,"",256);
        yield();
    }
    robot_id = i;
    okc_get_robot_name (okc,robot_id,name,256);
    std::cout << "Robots Name is " << name<< std::endl;
    std::cout<<"robot id "<<robot_id<<std::endl;
}

void ComOkc::registerCallback(int index){
    if(index == LEFT_ROBOT_ID){
        std::cout<<"callback register left"<<std::endl;
        if (OKC_OK != okc_register_axis_set_absolute_callback(ComOkc::okc,robot_id, (okc_callback_axis_t) &ComOkc::left_okcAxisAbsCallback,this)){
            std::cerr << "cbf_planner: left arm could not register callback, exiting" << std::endl;
            exit (EXIT_FAILURE);
        }
    }
    if(index == RIGHT_ROBOT_ID){
        std::cout<<"callback register right"<<std::endl;
        if (OKC_OK != okc_register_axis_set_absolute_callback(ComOkc::okc,robot_id, (okc_callback_axis_t) &ComOkc::right_okcAxisAbsCallback,this)){
            std::cerr << "cbf_planner: right arm could not register callback, exiting" << std::endl;
            exit (EXIT_FAILURE);
        }
    }
    if (OKC_OK != okc_register_cartpos_axis_set_absolute_callback(ComOkc::okc, robot_id, (okc_callback_cartpos_axis_t) &ComOkc::okcCartposAxisAbsCallback,this)){
        std::cerr << "cbf_planner: could not register callback, exiting" << std::endl;
        exit (EXIT_FAILURE);
    }
}

void ComOkc::set_stiffness(double *s, double *d){
    adamping.a1 = d[0];
    adamping.a2 = d[1];
    adamping.e1 = d[2];
    adamping.a3 = d[3];
    adamping.a4 = d[4];
    adamping.a5 = d[5];
    adamping.a6 = d[6];

    astiffness.a1 = s[0];
    astiffness.a2 = s[1];
    astiffness.e1 = s[2];
    astiffness.a3 = s[3];
    astiffness.a4 = s[4];
    astiffness.a5 = s[5];
    astiffness.a6 = s[6];
    okc_set_axis_stiffness_damping(okc,robot_id,astiffness,adamping);
}

void ComOkc::set_cp_stiffness(double *cps,double *cpd){
    cpstiff.x = cps[0];
    cpstiff.y = cps[1];
    cpstiff.z = cps[2];
    cpstiff.a = cps[3];
    cpstiff.b = cps[4];
    cpstiff.c = cps[5];

    cpdamping.x = cpd[0];
    cpdamping.y = cpd[1];
    cpdamping.z = cpd[2];
    cpdamping.a = cpd[3];
    cpdamping.b = cpd[4];
    cpdamping.c = cpd[5];
    okc_set_cp_stiffness_damping(okc,robot_id,cpstiff,cpdamping);
}

void ComOkc::set_cp_ExtTcpFT(double *tcpft){
    extft.x = tcpft[0];
    extft.y = tcpft[1];
    extft.z = tcpft[2];
    extft.a = tcpft[3];
    extft.b = tcpft[4];
    extft.c = tcpft[5];
    okc_set_cp_addTcpFT(okc,robot_id,extft);
}

void ComOkc::switch_to_cp_impedance(){
    okc_alter_cbmode(okc,robot_id,OKC_MODE_CALLBACK_POS_AXIS_ABS);
    okc_alter_cmdFlags (okc,robot_id,OKC_CMD_FLAGS_CP_AXIS_IMPEDANCE_MODE);
    okc_switch_to_cp_impedance(okc,robot_id);
}

void ComOkc::switch_to_jnt_impedance(){
    okc_alter_cbmode(okc,robot_id,OKC_MODE_CALLBACK_AXIS_ABS);
    okc_alter_cmdFlags (okc,robot_id,OKC_CMD_FLAGS_AXIS_IMPEDANCE_MODE);
    okc_switch_to_axis_impedance(okc,robot_id);
}


void ComOkc::request_monitor_mode(){
    okc_request_monitor_mode(okc,robot_id);
}


ComOkc::ComOkc(RobotNameT connectToRobot=kuka_right, \
               const char* ahostname = OKC_HOST, const char* aport = OKC_PORT)
{
    legacy_axis_mode = true;
    rn = connectToRobot;
    controller_update = false;
    ft = new coords_t;
    if (0 == ComOkc::instance_count)
    {
        strncpy (hostname,ahostname,16);
        strncpy (port,aport,6);
        initServer();
    }
    ComOkc::instance_count++;
    if (rn == kuka_left){
        bindToName((const char*)LEFTARM_IP);
    }
    else{
        if (rn == kuka_right){
            bindToName((const char*)RIGHTARM_IP);
        }
        else
            throw std::runtime_error("Undefined Robot");
    }
    if(rn == kuka_left){
        registerCallback(LEFT_ROBOT_ID);
    }
    if(rn == kuka_right){
        registerCallback(RIGHT_ROBOT_ID);
    }


    // initialize filters
    left_filter  = new CDDynamics( 7, 0.004, 250.0);
    right_filter  = new CDDynamics(7, 0.004, 250.0);

    mIsLeftArmInitialized = false;
    mIsRightArmInitialized = false;

    mLeftCurrentJoint.setZero(7);
    mRightCurrentJoint.setZero(7);
    mLeftDesiredJoint.setZero(7);
    mRightDesiredJoint.setZero(7);
}

void ComOkc::sleep_cycle_time(){
    okc_sleep_cycletime(okc,robot_id);
}

