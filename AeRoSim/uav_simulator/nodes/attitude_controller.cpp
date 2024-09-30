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

#define RPS2RPM 9.549296585513721
ros::Publisher cmd_pub;

struct AttitudeState{
	tf::Vector3 rpy;
	tf::Vector3 omega;
    double prev_t;

	AttitudeState() : rpy(tf::Vector3(0.0,0.0,0.0)), omega(tf::Vector3(0.0,0.0,0.0)), prev_t(-1) { }
};

struct SetpointState{
	double thrust;
	double roll;
	double pitch;
	double yaw_rate;
	double yaw_des;
	double fx;
	double fy;

	SetpointState() : thrust(0), roll(0), pitch(0), yaw_rate(0), yaw_des(0), fx(0), fy(0) { }
};

struct ControlState{
	double fcon;
	double throttle;
	double Ixx;
	double Iyy;
	double Izz;
	int num_of_props;
	double tz;
	Eigen::MatrixXd allocation_mat;
	double Kp;
	double Kt;
	double RLL_P;
	double RLL_D;
	double RLL_I;
	double PIT_P;
	double PIT_D;
	double PIT_I;
	double YAW_P;
	double YAW_D;
    double YAW_I;
	double J_RLL;
	double J_PIT;
	double J_YAW;
	double RLL_V_ERR;
	double PIT_V_ERR;
	double YAW_V_ERR;
	double phi_err_dot;
	double theta_err_dot;
  
	ControlState() : fcon(0.0027), throttle(500), Ixx(0.01), Iyy(0.01), Izz(0.02), num_of_props(4), tz(0.0), allocation_mat(Eigen::MatrixXd::Zero(4,4)),
	Kp(10.0), Kt(10.0), RLL_P(3.0), RLL_D(0.5), RLL_I(0.1), PIT_P(3.0), PIT_D(0.5), PIT_I(0.1), YAW_P(2.0), YAW_D(0.4),
			 J_RLL(0), J_PIT(0), RLL_V_ERR(0), PIT_V_ERR(0), YAW_V_ERR(0), phi_err_dot(0), theta_err_dot(0), YAW_I(0), J_YAW(0) { }
};


void pose_cb(const nav_msgs::Odometry& );
void setpoint_cb(const mav_msgs::RollPitchYawrateThrust& );
void control_update(const ros::TimerEvent&);
void init_params(const ros::NodeHandle );
double lp_filt(double, double, double);

AttitudeState UAV_RPY;
SetpointState UAV_CMD;
ControlState UAV_CTRL;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "attitude_ctrl_node");		
  ros::NodeHandle n_h;
  double frequency = 1000;
  ros::Rate rate(frequency);
  
  init_params(n_h);
	
  std::string namespc = argv[1];
    
  std::string pose_topic = namespc + "/odometry";
  std::string cmd_topic = namespc + "/command_motor_speed";
  std::string setpoint_topic = namespc + "/attitude_setpoint" ;

  ros::Subscriber pose_sub = n_h.subscribe(pose_topic, 1, &pose_cb);  
  ros::Subscriber att_sub = n_h.subscribe(setpoint_topic, 1, &setpoint_cb);
  cmd_pub = n_h.advertise<mav_msgs::Actuators>(cmd_topic, 1);
  
  ros::Timer ctrl_timer = n_h.createTimer(ros::Duration(1.0/frequency), &control_update);  

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
  
}

void pose_cb(const nav_msgs::Odometry& msg){
	//Figure out the time step
	double dt;
	double time = msg.header.stamp.toSec();
	if(UAV_RPY.prev_t == -1){
		UAV_RPY.prev_t = time - 0.001;
	}
	dt = time - UAV_RPY.prev_t ;
	UAV_RPY.prev_t = time ;

	//Compute attitude state
	tf::Quaternion q(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	tf::Matrix3x3 R(q);
	double roll, pitch, yaw;
	R.getRPY(roll,pitch,yaw);
	
	UAV_RPY.rpy.setX(roll);
	UAV_RPY.rpy.setY(pitch);
	UAV_RPY.rpy.setZ(yaw);

	UAV_RPY.omega.setX( msg.twist.twist.angular.x );
	UAV_RPY.omega.setY( msg.twist.twist.angular.y );
	UAV_RPY.omega.setZ( msg.twist.twist.angular.z );
}


void setpoint_cb(const mav_msgs::RollPitchYawrateThrust& msg){
	UAV_CMD.fx = msg.thrust.x;
	UAV_CMD.fy = msg.thrust.y;
	UAV_CMD.thrust = msg.thrust.z;
	UAV_CMD.roll = msg.roll;
	UAV_CMD.pitch = msg.pitch;
//	UAV_CMD.yaw_des += msg.yaw_rate*0.5;
//	UAV_CMD.yaw_rate = -1.4*atan2(sin(UAV_RPY.rpy.getZ() - UAV_CMD.yaw_des),cos(UAV_RPY.rpy.getZ() - UAV_CMD.yaw_des));
	UAV_CMD.yaw_rate = msg.yaw_rate;
}

void control_update(const ros::TimerEvent& e){
	
	double f_thrust = UAV_CMD.thrust ; //Should be already in Newton
	//If using directly attitude control, i.e. UAV_CMD.thrust is normalized [0-1], make it Newtons here	
	double phides = UAV_CMD.roll;
	double thetades = UAV_CMD.pitch;

	double phi = UAV_RPY.rpy.getX();
	double theta = UAV_RPY.rpy.getY();
	double phi_d = UAV_RPY.omega.getX();
	double theta_d = UAV_RPY.omega.getY();
	double psi_d = UAV_RPY.omega.getZ();
	
	double art_dt = 0.001; //artificial dt, good enough
	
	double error = phi - phides ;
	double v_des = -UAV_CTRL.Kp * error ;
	double v_err = phi_d - v_des ;
	UAV_CTRL.J_RLL += art_dt * v_err ;
	double v_err_dot = ( v_err - UAV_CTRL.RLL_V_ERR ) / art_dt ;
	UAV_CTRL.phi_err_dot = lp_filt(v_err_dot, UAV_CTRL.phi_err_dot, 0.2);
	UAV_CTRL.RLL_V_ERR = v_err ;
	double tx = -UAV_CTRL.RLL_P * v_err - UAV_CTRL.RLL_I * UAV_CTRL.J_RLL - UAV_CTRL.RLL_D * UAV_CTRL.RLL_V_ERR ;
	tx *= UAV_CTRL.Ixx;
	
	error = theta - thetades ;
	v_des = -UAV_CTRL.Kt * error ;
	v_err = theta_d - v_des ;
	UAV_CTRL.J_PIT += art_dt * v_err ;
	v_err_dot = ( v_err - UAV_CTRL.PIT_V_ERR ) / art_dt ;
	UAV_CTRL.theta_err_dot = lp_filt(v_err_dot, UAV_CTRL.theta_err_dot, 0.2);
	UAV_CTRL.PIT_V_ERR = v_err ;
	double ty = -UAV_CTRL.PIT_P * v_err - UAV_CTRL.PIT_I * UAV_CTRL.J_PIT - UAV_CTRL.PIT_D * UAV_CTRL.PIT_V_ERR ;
	ty *= UAV_CTRL.Iyy;

	error = psi_d - UAV_CMD.yaw_rate ;
	UAV_CTRL.J_YAW += art_dt * error ;
	v_err = (error - UAV_CTRL.YAW_V_ERR)/art_dt ;
	UAV_CTRL.YAW_V_ERR = error ;
	double tz = -UAV_CTRL.YAW_P * error - UAV_CTRL.YAW_D * v_err - UAV_CTRL.YAW_I * UAV_CTRL.J_YAW ;
	tz *= 4.0*UAV_CTRL.Izz; 
	  //Control was too aggressive sometimes
	if(abs(UAV_CTRL.tz - tz)>10.0){
		tz *= 0.01;
	}
	
	UAV_CTRL.tz = lp_filt(tz,UAV_CTRL.tz,0.01); 

	Eigen::VectorXd mot_thr(UAV_CTRL.num_of_props);
	Eigen::VectorXd ctrls(6);
	ctrls << UAV_CMD.fx, UAV_CMD.fy, f_thrust, tx, ty, UAV_CTRL.tz;
	mot_thr = UAV_CTRL.allocation_mat * ctrls;
	double m = mot_thr.minCoeff();
	double addition = abs(m)*1.04;
	Eigen::VectorXd ones(8);
	ones << addition, addition, addition, addition, addition, addition, addition, addition ;
	mot_thr = mot_thr + ones ;
	double maximum = mot_thr.maxCoeff();
	if(maximum>=13.2){ //assuming every motor can give up to ~1.4Kg of thrust
		for(int i = 0 ; i < UAV_CTRL.num_of_props ; i++){
			mot_thr(i) *= (13.2/maximum) ;
		}
	}
	//TURN THRUSTS TO 0-1 OF MAX_RPM
	mav_msgs::Actuators cmd_msg;
	cmd_msg.angular_velocities.clear();
	for(int i = 0 ; i < UAV_CTRL.num_of_props ; i++){
		cmd_msg.angular_velocities.push_back( sqrt(mot_thr(i)/UAV_CTRL.fcon) * RPS2RPM / UAV_CTRL.throttle );
	}
	cmd_pub.publish(cmd_msg);
}

void init_params(const ros::NodeHandle nh){

	nh.getParam("/Ixx", UAV_CTRL.Ixx);
	nh.getParam("/Iyy", UAV_CTRL.Iyy);
	nh.getParam("/Izz", UAV_CTRL.Izz);

	nh.getParam("/Att/kx", UAV_CTRL.RLL_P);
	nh.getParam("/Att/ky", UAV_CTRL.PIT_P);
	nh.getParam("/Att/kz", UAV_CTRL.YAW_P);

	nh.getParam("/Omega/kx", UAV_CTRL.RLL_D);
	nh.getParam("/Omega/ky", UAV_CTRL.PIT_D);
	nh.getParam("/Omega/kz", UAV_CTRL.YAW_D);							
								
	nh.getParam("/Integral/kx", UAV_CTRL.RLL_I);
	nh.getParam("/Integral/ky", UAV_CTRL.PIT_I);
	nh.getParam("/Integral/kyaw", UAV_CTRL.YAW_I);
								
	nh.getParam("/Att/kp", UAV_CTRL.Kp);
	nh.getParam("/Att/kt", UAV_CTRL.Kt);
	
	double mcon;
	nh.getParam("/thr_const", UAV_CTRL.fcon);
	nh.getParam("/drag_const", mcon);
	nh.getParam("/max_rpm", UAV_CTRL.throttle);
	
	int num_of_props;
	nh.getParam("/num_of_props", num_of_props);
	UAV_CTRL.num_of_props = num_of_props;
	std::string param_preface = "/propeller_";
	
	Eigen::MatrixXd ctrl_to_thr(num_of_props,6);

	ctrl_to_thr.row(0) << 0.05672595,  0.30532735,  0.22103447,  1.34366265, -0.20123229, 0.10340595;
	ctrl_to_thr.row(1) << -0.29740887,  0.24432859,  0.17201345, -1.23561926, -0.20164879, -0.3404224;
	ctrl_to_thr.row(2) << -0.22981984, -0.29660767,  0.00229046,  0.07218196,  0.05310532, -0.44562788;
	ctrl_to_thr.row(3) << 0.01294391,  0.23981251, -0.34150929,  0.29266017,  1.00092644, 0.3116706;
	ctrl_to_thr.row(4) << 0.50699376,  0.00542995,  0.02709579,  1.1140566 ,  0.49020019, -0.55076316;
	ctrl_to_thr.row(5) << -0.02700161, -0.06731981, -0.35838813, -0.38171035, -1.13501878, 0.15742413;
	ctrl_to_thr.row(6) << 0.16147557, -0.18154778,  0.17071623, -1.58165043, -0.40588238, 0.40624333;
	ctrl_to_thr.row(7) << -0.18915626, -0.25156428,  0.10476156,  0.35410129,  0.39684149, 0.34967509;

	UAV_CTRL.allocation_mat = ctrl_to_thr;
}


double lp_filt(double x1, double x2, double a){
	return (x1*a + x2*(1-a));
}

















