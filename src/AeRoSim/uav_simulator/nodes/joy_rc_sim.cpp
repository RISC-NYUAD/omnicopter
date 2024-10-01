#include <iostream>
#include <stdio.h>
#include <cstring>
#include <tf/tf.h>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>
#include <Eigen/Dense>

ros::Publisher cmd_pub;

mav_msgs::RollPitchYawrateThrust cmd_msg;
void joy_cb(const mav_msgs::RollPitchYawrateThrust& );

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rc_sim_node");		
  ros::NodeHandle n_h;
  double frequency = 200;
  ros::Rate rate(frequency);
  
  std::string namespc = argv[1];
  std::string setpoint_topic = namespc + "/attitude_setpoint" ;


  cmd_pub = n_h.advertise<mav_msgs::RollPitchYawrateThrust>(setpoint_topic, 1);
  ros::Subscriber cmd_sub = n_h.subscribe("/uav_setpoint", 1, &joy_cb);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}


void joy_cb(const mav_msgs::RollPitchYawrateThrust& msg){
	cmd_msg = msg ;
//	cmd_msg.thrust.z *= (500 * 0.06) ;
	cmd_pub.publish(cmd_msg);
}


