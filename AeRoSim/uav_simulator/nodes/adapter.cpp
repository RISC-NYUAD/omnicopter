#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstring>
#include <tf/tf.h>
#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>
#include <Eigen/Dense>
#include <omni_firmware/Pose.h>
#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>

#define RPS2RPM 9.549296585513721
ros::Publisher cmd_pub;
ros::Publisher pose_pub;





void pose_cb(const nav_msgs::Odometry& );
void motor_cb(const omni_firmware::MotorSpeed& msg);

//AttitudeState UAV_RPY;
//SetpointState UAV_CMD;
//ControlState UAV_CTRL;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "adapter_node");		
  ros::NodeHandle n_h;
  double frequency = 1000;
  ros::Rate rate(frequency);
  
  	
  std::string namespc = argv[1];
    
  std::string pose_topic = namespc + "/odometry";
  std::string cmd_topic = namespc + "/command_motor_speed";
  std::string setpoint_topic = namespc + "/attitude_setpoint" ;
  std::string pose_new = "pose_full";
  std::string Motortopic = "MotorSpeed";


  ros::Subscriber pose_sub = n_h.subscribe(pose_topic, 1, &pose_cb);  
  ros::Subscriber motor_sub = n_h.subscribe(Motortopic, 1, &motor_cb);  


  cmd_pub = n_h.advertise<mav_msgs::Actuators>(cmd_topic, 1);
  pose_pub = n_h.advertise<omni_firmware::FullPose>(pose_new, 1);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}

void pose_cb(const nav_msgs::Odometry& msg){
	omni_firmware::FullPose pose_new;

	pose_new.pose = msg.pose.pose;
	pose_new.vel = msg.twist.twist;
	pose_pub.publish(pose_new);

}

void motor_cb(const omni_firmware::MotorSpeed& msg){
	mav_msgs::Actuators cmd_msg;
	double motor_thr;
	for(int i = 0 ; i < 8 ; i++){
		motor_thr = msg.velocity[i];
		motor_thr = -0.0101*motor_thr*motor_thr + 61.8029*motor_thr - 52696;
		motor_thr = motor_thr < 0 ? 0 : motor_thr;
		motor_thr = motor_thr > 30000 ? 30000 : motor_thr;
		cmd_msg.angular_velocities.push_back(motor_thr/30000);
	}
	cmd_pub.publish(cmd_msg);

}


