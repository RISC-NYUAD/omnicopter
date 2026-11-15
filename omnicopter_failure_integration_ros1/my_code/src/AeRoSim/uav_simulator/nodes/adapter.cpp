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
#include <std_msgs/Float64MultiArray.h>
#include <cmath>
#include <algorithm>
#include <omni_firmware/Pose.h>
#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>

#define RPS2RPM 9.549296585513721
ros::Publisher cmd_pub;
ros::Publisher pose_pub;
bool direct_thrust=false;
double thrust_constant=4.4086e-6, max_rpm=30000;
Eigen::Vector3d com_offset=Eigen::Vector3d::Zero();
ros::Time last_thrust_time;
void thrust_cb(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if(msg->data.size()!=8) {ROS_ERROR_THROTTLE(1,"Expected eight motor thrusts"); return;}
    mav_msgs::Actuators out;
    for(double thrust:msg->data) {
        if(!std::isfinite(thrust) || thrust<0) {ROS_ERROR_THROTTLE(1,"Invalid thrust command"); return;}
        // Exact inverse of the Gazebo plugin's steady-state thrust law.
        const double rpm=std::sqrt(thrust/thrust_constant)/0.10472;
        out.angular_velocities.push_back(std::min(1.0,rpm/max_rpm));
    }
    last_thrust_time=ros::Time::now(); cmd_pub.publish(out);
}






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
  n_h.param("/omnicopter_sim/direct_thrust",direct_thrust,false);
  n_h.param("/omnicopter_sim/thrust_constant",thrust_constant,4.4086e-6);
  n_h.param("/omnicopter_sim/max_rpm",max_rpm,30000.0);
  std::vector<double> offset;
  if(n_h.getParam("/omnicopter_sim/com_offset",offset) && offset.size()==3)
    com_offset=Eigen::Vector3d(offset[0],offset[1],offset[2]);
  if(!(thrust_constant>0 && max_rpm>0) || !std::isfinite(thrust_constant+max_rpm) || !com_offset.allFinite())
    {ROS_FATAL("Invalid thrust model parameters"); return 1;}
  ros::Subscriber motor_sub;
  if(direct_thrust) motor_sub=n_h.subscribe("/omnicopter/motor_thrust",1,thrust_cb);
  else motor_sub=n_h.subscribe(Motortopic,1,&motor_cb);  


  cmd_pub = n_h.advertise<mav_msgs::Actuators>(cmd_topic, 1);
  pose_pub = n_h.advertise<omni_firmware::FullPose>(pose_new, 1);

  while(ros::ok()){
    ros::spinOnce();
    if(direct_thrust && (last_thrust_time.isZero() ||
       (ros::Time::now()-last_thrust_time).toSec()>0.1)) {
      mav_msgs::Actuators stop; stop.angular_velocities.resize(8,0); cmd_pub.publish(stop);
    }
    rate.sleep();
  }
  
}

void pose_cb(const nav_msgs::Odometry& msg){
	omni_firmware::FullPose pose_new;

	pose_new.header = msg.header;
	pose_new.pose = msg.pose.pose;
	pose_new.vel = msg.twist.twist;
    if(direct_thrust) {
      // Plugin reports base-link CoG. Controller uses aggregate vehicle CoM.
      Eigen::Quaterniond q(msg.pose.pose.orientation.w,msg.pose.pose.orientation.x,
                           msg.pose.pose.orientation.y,msg.pose.pose.orientation.z);
      if(!q.coeffs().allFinite() || q.norm()<1e-8) return;
      Eigen::Matrix3d R=q.normalized().toRotationMatrix();
      Eigen::Vector3d dp=R*com_offset;
      Eigen::Vector3d w(msg.twist.twist.angular.x,msg.twist.twist.angular.y,msg.twist.twist.angular.z);
      Eigen::Vector3d dv=R*w.cross(com_offset);
      pose_new.pose.position.x+=dp.x(); pose_new.pose.position.y+=dp.y(); pose_new.pose.position.z+=dp.z();
      pose_new.vel.linear.x+=dv.x(); pose_new.vel.linear.y+=dv.y(); pose_new.vel.linear.z+=dv.z();
    }
	pose_pub.publish(pose_new);

}

void motor_cb(const omni_firmware::MotorSpeed& msg){
	if(msg.velocity.size()!=8) return;
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


