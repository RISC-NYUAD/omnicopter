#include <iostream>
#include <fstream>
#include <stdio.h>
#include <cstring>
#include <tf/tf.h>
#include <ros/ros.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <geometry_msgs/Twist.h>
#include <Eigen/Dense>

#define RPS2RPM 9.549296585513721

ros::Publisher rpyt_pub;

struct VehicleState{
	double prev_t;
	tf::Vector3 rpy;
	tf::Vector3 vel;
	tf::Vector3 acc;
	VehicleState() : prev_t(-1), rpy(tf::Vector3(0.0,0.0,0.0)), vel(tf::Vector3(0.0,0.0,0.0)), acc(tf::Vector3(0.0,0.0,0.0)) { }
};

struct SetpointState{
	double vx;
	double vy;
	double vz;
	double yaw_rate;

	SetpointState() : vx(0), vy(0), vz(0), yaw_rate(0) { }
};

struct ControlState{
	double mass;
	double Kpx;
	double Kpy;
	double Kpz;
	double Kdx;
	double Kdy;
	double Kdz;
	double Kiz;
	double Jz;
	
	ControlState() : mass(2.0),	Kpx(2.0), Kpy(2.0), Kpz(2.0), Kdx(0.5), Kdy(0.5), Kdz(0.5), Kiz(0.2), Jz(0) { }
};


void pose_cb(const nav_msgs::Odometry& );
void setpoint_cb(const geometry_msgs::Twist& );
void control_update(const ros::TimerEvent&);
void init_params(const ros::NodeHandle );

VehicleState UAV_STATE;
SetpointState UAV_CMD;
ControlState UAV_CTRL;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vel_ctrl_node");		
  ros::NodeHandle n_h;
  double frequency = 1000;
  ros::Rate rate(frequency);
  
  init_params(n_h);
	
  std::string namespc = argv[1];
  
  std::string pose_topic = namespc + "/odometry";
  std::string cmd_topic = namespc + "/attitude_setpoint";
  std::string setpoint_topic = namespc + "/body_vel_setpoint" ;


  ros::Subscriber pose_sub = n_h.subscribe(pose_topic, 1, &pose_cb);  
  ros::Subscriber att_sub = n_h.subscribe(setpoint_topic, 1, &setpoint_cb);
  rpyt_pub = n_h.advertise<mav_msgs::RollPitchYawrateThrust>(cmd_topic, 1);
  
  ros::Timer ctrl_timer = n_h.createTimer(ros::Duration(2.0/frequency), &control_update);  

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}

void pose_cb(const nav_msgs::Odometry& msg){
	double dt;
	double time = msg.header.stamp.toSec();
	if(UAV_STATE.prev_t==-1){
		UAV_STATE.prev_t = time - 0.001;
	}
	dt = time - UAV_STATE.prev_t ;
	UAV_STATE.prev_t = time ; //in case we need dt
	
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf::Matrix3x3 R(q);
	double roll, pitch, yaw;
	R.getRPY(roll,pitch,yaw);
	
	UAV_STATE.rpy.setX(roll);
	UAV_STATE.rpy.setY(pitch);
	UAV_STATE.rpy.setZ(yaw);

	UAV_STATE.acc.setX( (msg.twist.twist.linear.x - UAV_STATE.vel.getX())/dt );
	UAV_STATE.acc.setY( (msg.twist.twist.linear.y - UAV_STATE.vel.getY())/dt );
	UAV_STATE.acc.setZ( (msg.twist.twist.linear.z - UAV_STATE.vel.getZ())/dt );

	UAV_STATE.vel.setX( msg.twist.twist.linear.x );
	UAV_STATE.vel.setY( msg.twist.twist.linear.y );
	UAV_STATE.vel.setZ( msg.twist.twist.linear.z );	 //WORLD FRAME VELOCITIES

	//msg.twist.twist.angular.<xyz> unused
}


void setpoint_cb(const geometry_msgs::Twist& msg){
	UAV_CMD.vx = msg.linear.x;
	UAV_CMD.vy = msg.linear.y;
	UAV_CMD.vz = msg.linear.z;
	UAV_CMD.yaw_rate = msg.angular.z;
}

void control_update(const ros::TimerEvent& e){

	tf::Vector3 world_vel_cmd ;
	tf::Matrix3x3 RotMat;
	double psi = UAV_STATE.rpy.getZ();
	RotMat.setRPY(0.0, 0.0, psi);
	world_vel_cmd = RotMat * tf::Vector3(UAV_CMD.vx, UAV_CMD.vy, UAV_CMD.vz) ; //desired velocities now in world frame

	tf::Vector3 error = UAV_STATE.vel - world_vel_cmd;	
	
	UAV_CTRL.Jz += 0.01 * error.getZ() ;
	UAV_CTRL.Jz = std::max(std::min(UAV_CTRL.Jz,5.0),-5.0); 
	
	double acc_des_x = - UAV_CTRL.Kpx * error.getX() - UAV_CTRL.Kdx * UAV_STATE.acc.getX() ;
	double acc_des_y = - UAV_CTRL.Kpy * error.getY() - UAV_CTRL.Kdy * UAV_STATE.acc.getY() ;
	double acc_des_z = 9.81 - UAV_CTRL.Kpz * error.getZ() - UAV_CTRL.Kdz * UAV_STATE.acc.getZ() - UAV_CTRL.Kiz * UAV_CTRL.Jz;

	double Ax = cos(psi)*acc_des_x + sin(psi)*acc_des_y;
	double Ay = sin(psi)*acc_des_x - cos(psi)*acc_des_y;	

	double thrust = UAV_CTRL.mass * sqrt(pow(acc_des_z,2) + pow(Ax,2) + pow(Ay,2));

	/* FOR REGULAR MULTICOPTER 
	double phi_des = asin(UAV_CTRL.mass*Ay/thrust);
	double theta_des = asin(UAV_CTRL.mass*Ax/(thrust * cos(UAV_STATE.rpy.getX())));	
	phi_des = (phi_des > 0.3) ? 0.3 : phi_des ;
	phi_des = (phi_des < -0.3) ? -0.3 : phi_des ;	
	theta_des = (theta_des > 0.3) ? 0.3 : theta_des ;
	theta_des = (theta_des < -0.3) ? -0.3 : theta_des ;		
	*/

	double fx,fy;
	tf::Matrix3x3 Rb2w;
	Rb2w.setRPY(UAV_STATE.rpy.getX(), UAV_STATE.rpy.getY(), psi);
	tf::Matrix3x3 Rw2b = Rb2w.transpose();
	tf::Vector3 world_actions(UAV_CTRL.mass * acc_des_x, UAV_CTRL.mass * acc_des_y, UAV_CTRL.mass * acc_des_z);
	tf::Vector3 body_actions = Rw2b * world_actions;
	fx = body_actions.getX();
	fy = body_actions.getY();
	thrust = body_actions.getZ();

	mav_msgs::RollPitchYawrateThrust cmd_msg;

	cmd_msg.roll = 0.0;
	cmd_msg.pitch = 0.0; //FOR HOVERING FLIGHT
	cmd_msg.yaw_rate = UAV_CMD.yaw_rate;
	cmd_msg.thrust.x = fx;
	cmd_msg.thrust.y = fy;
	cmd_msg.thrust.z = thrust;

	rpyt_pub.publish(cmd_msg);
	

}


void init_params(const ros::NodeHandle nh){

	nh.getParam("/mass", UAV_CTRL.mass);

	nh.getParam("/Kp/kx", UAV_CTRL.Kpx);
	nh.getParam("/Kp/ky", UAV_CTRL.Kpy);
	nh.getParam("/Kp/kz", UAV_CTRL.Kpz);

	nh.getParam("/Kd/kx", UAV_CTRL.Kdx);
	nh.getParam("/Kd/ky", UAV_CTRL.Kdy);
	nh.getParam("/Kd/kz", UAV_CTRL.Kdz);

	nh.getParam("/Ki/kz", UAV_CTRL.Kiz);
}




