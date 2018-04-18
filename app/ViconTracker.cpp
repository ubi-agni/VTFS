/*
 ============================================================================
 Name        : ViconTracker.cpp
 Author      : Qiang Li
 Version     :
 Copyright   : Copyright Qiang Li, Universität Bielefeld
 Description : update vicon marker pose. This is done by translating
               the coordinateframe of old one from vicon bridge along z axis 4cm.
               We need do this translation because the 
               origin of vicon calibration tool is shifted 4 cm up along z axis in the 
               world frame defined for robotic system.
 ============================================================================
 */

/*
 *
*/

#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <iostream>

#include "Timer.h"

ros::NodeHandle *nh;
tf::TransformBroadcaster *br;
tf::TransformListener *listener;


void run(){

    ros::Rate r(500);
    while (ros::ok())
    {
      r.sleep();
      ros::spinOnce();
    }
}


void ros_publisher(){
    tf::StampedTransform transform_rev;
    try{
        listener->waitForTransform("/world","/vicon/WiperTool_4small_markers/wiper_frame", ros::Time(0), ros::Duration(10.0) );
        listener->lookupTransform("/world", "/vicon/WiperTool_4small_markers/wiper_frame",
                                  ros::Time(0), transform_rev);
    }
    catch (tf::TransformException &ex) {
         ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
      }
//       std::cout<<"origin position "<<transform_rev.getOrigin().getX()<<","<<transform_rev.getOrigin().getY()<<","<<transform_rev.getOrigin().getZ()<<std::endl;
//       
//       Eigen::Quaterniond eigen_pose;
//       tf::quaternionTFToEigen (transform_rev.getRotation(), eigen_pose);
//       Eigen::Matrix3d R = eigen_pose.toRotationMatrix();
//       std::cout<<"current orien"<<std::endl;
//       std::cout<<R<<std::endl;
// //       Eigen::Matrix3d trans;
// //       trans(0,1) = 1;
// //       trans(1,2) = -1;
// //       trans(2,0) = -1;
//       Eigen::Matrix3d newR;
//       tf::Matrix3x3 tfR;
//       newR = R.transpose();
//       tf::matrixEigenToTF (newR, tfR);
      
    tf::Transform transform_new(transform_rev.getRotation(), tf::Vector3(transform_rev.getOrigin().getX(), transform_rev.getOrigin().getY(), transform_rev.getOrigin().getZ()+0.05));
//     transform_new.setBasis(tfR);
    br->sendTransform(tf::StampedTransform(transform_new, ros::Time::now(), "world", "new_viconmarker_frame"));
    
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "Update_viconmarker",ros::init_options::NoSigintHandler);
    nh = new ros::NodeHandle();
    ros::spinOnce();
    br = new tf::TransformBroadcaster();
    listener = new tf::TransformListener();
    std::cout<<"ros init finished"<<std::endl;
    //start a timer thread
    Timer thrd_getMsg(run);
    thrd_getMsg.setSingleShot(false);
    thrd_getMsg.setInterval(Timer::Interval(1));
    thrd_getMsg.start(true);
    
    //start ros publisher thread
    Timer thrd_rospublisher(ros_publisher);
    thrd_rospublisher.setSingleShot(false);
    thrd_rospublisher.setInterval(Timer::Interval(20));
    thrd_rospublisher.start(true);
    

    while(1){
    }

    //stop ros
    ros::shutdown();
    thrd_getMsg.stop();
    thrd_rospublisher.stop();
    delete nh;

    return 0;
}



