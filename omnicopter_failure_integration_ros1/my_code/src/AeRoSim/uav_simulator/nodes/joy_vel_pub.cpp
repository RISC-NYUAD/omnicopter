#include <iostream>
#include <stdio.h>
#include <cstring>
#include <tf/tf.h>
#include <ros/ros.h>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>
#include <Eigen/Dense>

ros::Publisher cmd_pub;

geometry_msgs::Twist cmd_msg;
void joy_cb(const mav_msgs::RollPitchYawrateThrust& );

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pos_ctrl_sim_node");		
  ros::NodeHandle n_h;
  double frequency = 200;
  ros::Rate rate(frequency);
  
  std::string namespc = argv[1];
  std::string setpoint_topic = namespc + "/body_vel_setpoint" ;

  cmd_pub = n_h.advertise<geometry_msgs::Twist>(setpoint_topic, 1);
  ros::Subscriber cmd_sub = n_h.subscribe("/uav_setpoint", 1, &joy_cb);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}


void joy_cb(const mav_msgs::RollPitchYawrateThrust& msg){

	cmd_msg.linear.x = msg.pitch*2;
	cmd_msg.linear.y = msg.roll*2;
	cmd_msg.linear.z = msg.thrust.z*2;
	cmd_msg.angular.z = -msg.yaw_rate*2.5;
//	cmd_msg.angular.x = msg.thrust.x;

	cmd_pub.publish(cmd_msg);
}


