
/*
 ============================================================================
 Name        : TipComTest.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : This is communication test programe to get the raw data from teensy fingertip
 ============================================================================
 */

/*
 *
*/

//for ROS
#ifdef HAVE_ROS
#include <ros/ros.h>
#include <sr_robot_msgs/UBI0All.h>
//#include <agni_utils/tactile_calibration.hpp>
#endif
#include <iomanip>
#include "Timer.h"
#include "comrsb.h"
#include "fingertiptac.h"
#include <mutex>

#ifdef HAVE_ROS
// ROS objects
ros::NodeHandle *nh;
#endif


using namespace rsb;
ComRSB *com_rsb;
bool StopFlag;

//teensy fingertip taxel num
#define TAC_NUM 12
//initialize fingertip instance
FingertipTac *ftt;

//using mutex locking controller ptr while it is switching.
std::mutex mutex_tac;

void get_mid_info(){
    bool contact_f = false;
    contact_f = ftt->isContact(ftt->data);
    if(contact_f == true){
        int taxId;
        taxId = ftt->est_ct_taxelId(ftt->data);
        ftt->contactarea = ftt->WhichArea(taxId);
        std::cout<<"cp is "<<ftt->pos[0]<<","<<ftt->pos[1]<<","<<ftt->pos[2]<<std::endl;
        std::cout<<"num and [ id ] "<<ftt->act_taxel_num<<",[ ";
        for(int i = 0; i < ftt->act_taxel_num; i++)
            std::cout<<ftt->act_Ids[i]<<",";
        std::cout<<" ]"<<std::endl;
        std::cout<<"est pressure is "<< ftt->pressure<<"[ ";
        for(int i = 0; i < ftt->act_taxel_num; i++)
            std::cout<<ftt->data.fingertip_tac_pressure[ftt->act_Ids[i]]<<",";
        std::cout<<" ]"<<std::endl;

    }
    else{

    }
}

#ifdef HAVE_ROS
// receive Tactile data from UBI Fingertips
void
recvTipTactile(const sr_robot_msgs::UBI0AllConstPtr& msg)
{
    double press_val=0;
    ftt->clear_data();
    Eigen::Vector3d pos_val,nv_val;

    // for first sensor each taxel
    std::cout <<"0"<<"\t"<<"1"<<"\t"<<"2"<<"\t"<<"3"<<"\t"<<"4"<<"\t"<<"5"<<"\t"<<"6"<<"\t"<<"7"<<"\t"<<"8"<<"\t"<<"9"<<"\t"<<"10"<<"\t"<<"11"<<"\t"<<std::endl;
    std::cout << std::fixed;
    for(size_t j = 0; j < msg->tactiles[0].distal.size(); ++j) {
//        val= 1.0-(msg->tactiles[0].distal[j]/1023.0); //agni::tactileCalibration(msg->tactiles[0].distal[j], 1024, agni::UBI0);
        press_val= 1.0-(msg->tactiles[0].distal[j]/1023.0);
        if(press_val < MID_THRESHOLD) press_val = 0.0;
        ftt->data.fingertip_tac_pressure.push_back(press_val);
        std::cout<<std::setprecision(5)<<press_val<<"\t";
    }
    std::cout<<std::endl;
    get_mid_info();
}

#endif

void closeprog_cb(boost::shared_ptr<std::string> data){
    StopFlag = true;
    std::cout<<"The program will be closed"<<std::endl;
}

void run(){
#ifdef HAVE_ROS
    ros::Rate r(100);
    while (ros::ok())
    {
      r.sleep();
      ros::spinOnce();
    }
#endif
}


int main(int argc, char* argv[])
{

    boost::function<void(boost::shared_ptr<std::string>)> button_closeprog(closeprog_cb);
    StopFlag = false;
    ftt = new FingertipTac(12);
    #ifdef HAVE_ROS
        ros::init(argc, argv, "KukaRos",ros::init_options::NoSigintHandler);
        nh = new ros::NodeHandle();
        ros::Subscriber	tacTipSub;
        if (argc > 1){
            std::cout<<"set hand by user"<<std::endl;
            std::string handname(argv[1]);
            tacTipSub=nh->subscribe("/"+handname+"/teensy/tactile", 1, // buffer size
                                                    &recvTipTactile);
          }
        else{
            //default is left hand
            std::cout<<"use the left hand as default"<<std::endl;
            tacTipSub=nh->subscribe("/lh/teensy/tactile", 1, // buffer size
                                                    &recvTipTactile);
        }
    #endif
    //start a timer thread
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(20));
    thrd_getMsg.start(true);
    //start rsb
    com_rsb = new ComRSB();
    //register cb function
    com_rsb->register_external("/foo/closeprog",button_closeprog);
    //main thread is hanging
    while(!StopFlag){
    }

    //stop ros
    #ifdef HAVE_ROS
    ros::shutdown();
    delete nh;
    #endif

    //stop the timer
    thrd_getMsg.stop();
    return 0;
}



