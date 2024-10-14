#include "controller.hpp"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

#include <omni_firmware/Pose.h>
#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>
#include <omni_firmware/errorMsg.h>
#include <omni_firmware/Uvector.h>

#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <math.h>
#include <cmath>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>


#include "utils.cpp"
int serial = 0;

void sendSerialData(uint8_t* data, size_t size) {
    if (serial == 0){
      serial = open("/dev/ttyACM0", O_WRONLY | O_NONBLOCK);//O_NOCTTY);
      struct termios options;
      tcgetattr(serial, &options);
      cfsetispeed(&options, B115200);  // Set input baud rate
      cfsetospeed(&options, B115200);  // Set output baud rate
      tcsetattr(serial, TCSANOW, &options);
    }

    if (serial < 0) {
        printf("Failed to open serial port\n");
        exit(1);
    }
	write(serial, data, size);

}

Controller::Controller() : nh_() {

	// the following are containers to get topic names
	std::string prop_cmd_topic;
	std::string prop_act_topic;
	std::string log_data_topic = "controller_log";
	std::string pose_topic;
	std::string pose_d_topic;
	if(!nh_.getParam("controller/prop_cmd", prop_cmd_topic)) {
		ROS_ERROR("Could not find prop_cmd parameter!, setting DEFAULT");
		prop_cmd_topic = "prop_cmd";
	}
	if(!nh_.getParam("controller/pose", pose_topic)) {
		ROS_ERROR("Could not find pose parameter!, setting DEFAULT");
		pose_topic = "pose";
	}
	if(!nh_.getParam("controller/pose_d", pose_d_topic)) {
		ROS_ERROR("Could not find desired pose parameter!, setting DEFAULT");
		pose_d_topic = "pose_d";
	}

	prop_cmd_pub = nh_.advertise < omni_firmware::MotorSpeed > (prop_cmd_topic, 1000);
	controller_log_pub = nh_.advertise < omni_firmware::errorMsg > (log_data_topic, 1000);
	this->Uvector_pub = nh_.advertise < omni_firmware::Uvector>("/u", 1000);
	this->Uvector_new_pub = nh_.advertise < omni_firmware::Uvector>("/u_new", 1000);
	
    toggle_sub = nh_.subscribe("/toggle_topic", 1000, &Controller::toggleCallback, this);
	pose_sub = nh_.subscribe(pose_topic, 1000, &Controller::poseCallback, this);
	pose_d_sub = nh_.subscribe(pose_d_topic, 1000, &Controller::poseDCallback, this);

	linearWrench_service = nh_.advertiseService("LinearWrench", &Controller::linear_wrench_callback, this);
	angularWrench_service = nh_.advertiseService("AngularWrench", &Controller::angular_wrench_callback, this);

	// the following are containers to get params
	bool ground_start;
	double dz_pre_trans;
	double weight;
	double cmd_min;
	double cmd_max;
	double sat_ex_, sat_ev_, sat_iex_, sat_ier_;
	double yaw_moment_limit, rp_moment_limit;
	std::vector<double> att_kp_list;
	std::vector<double> att_kp_ground_list;
	std::vector<double> att_kd_list;
	std::vector<double> att_ki_list;
	std::vector<double> pos_kp_list;
	std::vector<double> pos_kd_list;
	std::vector<double> pos_ki_list;
	std::vector<double> allocation_list;
	std::vector<double> full_allocation_list;
    	std::vector<double> positives_array_list;
	std::vector<double> null_space_list;
	
	// each will set the parameter to default value if
	// the corresponding param is not set

	if(!nh_.getParam("controller/controller_gains/enable_ground", ground_start)) {
		ROS_ERROR("Could not find GROUND parameter!, STARTING ON GROUND BY DEFAULT");
		this->ground_start = true;
		this->ground_config_flag = true;
	} else
	{
		this->ground_start = ground_start;
		this->ground_config_flag = ground_start;
	}
	if(!nh_.getParam("controller/controller_gains/dz_pre_trans", dz_pre_trans)) {
		ROS_ERROR("Could not find dz_pre_trans parameter!, STARTING BY DEFAULT");
		this->dz_pre_trans = 0.2;
	} else
		this->dz_pre_trans = dz_pre_trans;

	if(!nh_.getParam("controller/controller_gains/weight", weight)) {
		ROS_ERROR("Could not find weight parameter!, setting DEFAULT");
		this->weight = 1;
	} else
		this->weight = weight;

	if(!nh_.getParam("controller/controller_gains/cmd_min", cmd_min)) {
		ROS_ERROR("Could not find cmd_min parameter!, setting DEFAULT");
		this->cmd_min = 0;
	} else
		this->cmd_min = cmd_min;

	if(!nh_.getParam("controller/controller_gains/cmd_max", cmd_max)) {
		ROS_ERROR("Could not find cmd_max parameter!, setting DEFAULT");
		this->cmd_max = 125;
	} else
		this->cmd_max = cmd_max;

	if(!nh_.getParam("controller/controller_gains/sat/ev", sat_ev_)) {
		ROS_ERROR("Could not find sat_ev parameter!, setting DEFAULT");
		this->sat_ev = 0.5;
	} else
		this->sat_ev = sat_ev_;

	if(!nh_.getParam("controller/controller_gains/sat/iex", sat_iex_)) {
		ROS_ERROR("Could not find sat_iex parameter!, setting DEFAULT");
		this->sat_iex = 2;
	} else
		this->sat_iex = sat_iex_;

	if(!nh_.getParam("controller/controller_gains/sat/ier", sat_ier_)) {
		ROS_ERROR("Could not find sat_ier parameter!, setting DEFAULT");
		this->sat_ier = 2;
	} else
		this->sat_ier = sat_ier_;

	if(!nh_.getParam("controller/controller_gains/sat/ex", sat_ex_)) {
		ROS_ERROR("Could not find sat_ex parameter!, setting DEFAULT");
		this->sat_ex = 0.3;
	} else
		this->sat_ex = sat_ex_;

	if(!nh_.getParam("controller/controller_gains/att_pid/kp", att_kp_list)) {
		ROS_ERROR("Could not find topic att_kp parameter!, setting DEFAULT");
		this->att_kp << 1.25, 1.25, 3;
	} else {
		this->att_kp = Eigen::Vector3d::Map(att_kp_list.data(), 3);
	}

	if(!nh_.getParam("controller/controller_gains/att_pid/kpGround", att_kp_ground_list)) {
		ROS_ERROR("Could not find kpGround parameter!, setting DEFAULT");
		this->att_kp_ground << 1.25, 1.25, 3;
	} else {
		this->att_kp_ground = Eigen::Vector3d::Map(att_kp_ground_list.data(), 3);
	}

	if(!nh_.getParam("controller/controller_gains/att_pid/kd", att_kd_list)) {
		ROS_ERROR("Could not find topic att_kd parameter!, setting DEFAULT");
		this->att_kd << 2.0, 2.0, 5.0;
	} else {
		this->att_kd = Eigen::Vector3d::Map(att_kd_list.data(), 3);
	}

	if(!nh_.getParam("controller/controller_gains/att_pid/ki", att_ki_list)) {
		ROS_ERROR("Could not find topic att_ki parameter!, setting DEFAULT");
		this->att_ki << 0.25, 0.25, 0.25;
	} else {
		this->att_ki = Eigen::Vector3d::Map(att_ki_list.data(), 3);
	}

	if(!nh_.getParam("controller/controller_gains/pos_pid/kp", pos_kp_list)) {
		ROS_ERROR("Could not find topic pos_kp parameter!, setting DEFAULT");
		this->pos_kp << 1.25, 1.25, 4;
	} else {
		this->pos_kp = Eigen::Vector3d::Map(pos_kp_list.data(), 3);
	}

	if(!nh_.getParam("controller/controller_gains/pos_pid/kd", pos_kd_list)) {
		ROS_ERROR("Could not find topic pos_kd parameter!, setting DEFAULT");
		this->pos_kd << 2.0, 2.0, 6.0;
	} else {
		this->pos_kd = Eigen::Vector3d::Map(pos_kd_list.data(), 3);
	}

	if(!nh_.getParam("controller/controller_gains/pos_pid/ki", pos_ki_list)) {
		ROS_ERROR("Could not find topic pos_ki parameter!, setting DEFAULT");
		this->pos_ki << 0.25, 0.25, 0.22;
	} else {
		this->pos_ki = Eigen::Vector3d::Map(pos_ki_list.data(), 3);
	}
	
	if(!nh_.getParam("controller/controller_gains/yaw_moment_limit", yaw_moment_limit)) {
		ROS_ERROR("Could not find topic yaw_moment_limit parameter!, setting DEFAULT");
		this->YAW_MOMENT_LIMIT = 2.0;
	} else {
		this->YAW_MOMENT_LIMIT = yaw_moment_limit;
	}
	
	if(!nh_.getParam("controller/controller_gains/rp_moment_limit", rp_moment_limit)) {
		ROS_ERROR("Could not find topic rp_moment_limit parameter!, setting DEFAULT");
		this->RP_MOMENT_LIMIT = 3.0;
	} else {
		this->RP_MOMENT_LIMIT = rp_moment_limit;
	}
	
	if(!nh_.getParam("controller/controller_gains/Allocation_matrix", allocation_list)) {
		ROS_ERROR("Could not find topic allocation_matirx parameter!, setting DEFAULT");
		allocation_list = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.00025, 0.00025, 0.00025, 0.00025, 0.00025, 0.00025, 0., 0.0000325, 0.0000325, 0., -0.0000325, -0.0000325, -0.0000375, -0.0000188, 0.0000187, 0.0000375, 0.0000188, -0.0000188, 0.0000074, -0.0000074, 0.0000074, -0.0000074, 0.0000074, -0.0000074 };
	}
	double G_buffer[allocation_list.size()];
	std::copy(allocation_list.begin(), allocation_list.end(), G_buffer);
	this->n = allocation_list.size() / 6;
	const int n = this->n;
	
	if(!nh_.getParam("controller/controller_gains/null_space", null_space_list)) {
		ROS_ERROR("Could not find topic allocation_matirx parameter!, setting DEFAULT");
		null_space_list = {0.301429102388748,0.735955403463547,-0.744949427323837,-0.421971587344518,0.172298872444366,-0.794293521479409,0.819043900300765,0.046510193764713,0.904741735148129,-0.593704394980915,-0.293022583675305,-0.421163826202063,0.953420437122514,0.265376405450306,-0.138753111972226,-0.751244359993357,-0.30096991363634,0.325399347714704,0.599323048268742,-0.802845571061326,-0.247593555310946,0.546509986337043,0.556699796387724,-0.658383409157718,-0.00888400144355125,0.215117903216544,0.206878060048819,-0.241382414880486,-0.0474608747180168,-0.217249131581812,-0.273493529619868,0.221027676501879,0.168915338845869,0.127629220480947,0.0214932509610041,-0.259584331122989,-0.0801275667833885,-0.107326620103627,-0.090396649549601,0.243303861879375,0.438758999209262,-0.333424682483333,0.291021295990369,0.258926401815371,-0.380895378375629,-0.293347679694737,0.402790861728662,-0.189524582517963};
	}
	double nB_buffer[null_space_list.size()];
	std::copy(null_space_list.begin(),null_space_list.end(), nB_buffer);
	Eigen::Map < Eigen::MatrixXd > nB(nB_buffer, n, 2);
	this->nB = nB;

	// the following is the storing of the allocation
	// matrix in the RowMajor form with dynamic matrix
	// and inverting of the allocation matrix
	Eigen::Map < Eigen::MatrixXd > G(G_buffer, n, 6);
	
	Eigen::MatrixXd Srg(3,3);
	Srg <<     0.0, -0.0134, 0.0048,
		    0.0134,     0.0, 0.0010,
		   -0.0048, -0.0010,    0.0;
	double drag_lift_ratio = 0.0130;//0.023;
	Eigen::MatrixXd G_;
	G_ = G.transpose();	
	
	#undef solve
	

	this->iG = G_.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(Eigen::Matrix<
			double, 6, 6>::Identity());
	#define solve lu_solve
    double u_min, u_max;
    u_min = cmd_min * cmd_min;
    u_max = cmd_max * cmd_max;
    
    this->Iex << 0.0, 0.0, 0.0;
	this->Ier << 0.0, 0.0, 0.0;
	this->ex << 0.0, 0.0, 0.0;
	this->ev << 0.0, 0.0, 0.0;
	this->ew << 0.0, 0.0, 0.0;
	this->ea << 0.0, 0.0, 0.0;
	this->log_data_flag = false;

	this->thrust_dmd = 0.0f;

	this->accd_x_dmd = 0.0f;
	this->accd_y_dmd = 0.0f;

	this->state_timer = 0;
	this->active_integrator = false;
    this->full_actuation = false;

	this->prop_cmd_d  = Eigen::VectorXd::Zero(n);
	this->weight_ = 1;
}

Controller::~Controller() {
}


// Linear Wrench is a service for wrench testing
// Wrench testing can only be used when lift_off is not yet called
// Linear wrench requires a start force and end force and a duration
// Linearly interpolated wrench is applied between the two wrenches depending on the time
bool Controller::linear_wrench_callback(controller::LinearWrench::Request &req, controller::LinearWrench::Response &res) {
	
	if(this->test_wrench_received) {
		res.status = false;
		return false;
	} else {
		this->wrench_max_time = req.duration + req.ramp;
		this->wrench_ramp_time = req.ramp;
		this->wrench_start_time = ros::Time::now();
		
		this->linearWrench_start << req.fx1, req.fy1, req.fz1;
		this->linearWrench_end   << req.fx2, req.fy2, req.fz2;
		this->test_wrench_received = true;
		this->linearWrench = true;
		res.status = true;
		return true;
	}
}


void Controller::apply_linear_wrench_test(){
	ros::Time thisTime = ros::Time::now();
	double seconds = (thisTime - this->wrench_start_time).toSec();
	Eigen::VectorXd prop_cmd_d(this->n);

	if (seconds > wrench_max_time)
	{
		for(int i = 0; i < this->n; i++) {
			prop_cmd_d[i] = 0;
		}
		this->wrench << 0, 0, 0, 0., 0., 0.;
		this->prop_cmd_d = prop_cmd_d.array();
		this->test_wrench_received = false;
		this->linearWrench = false;
	} else if (seconds < wrench_ramp_time){
		Eigen::Vector3d force = linearWrench_start + (linearWrench_end - linearWrench_start)*(seconds/wrench_ramp_time);
		this->wrench << force[0], force[1], force[2], 0., 0., 0.;
		prop_cmd_d = (this->iG * this->wrench);
    	double min_prop_cmd = 100000;


		for(int i = 0; i < this->n; i++) {
      		if (prop_cmd_d[i] < min_prop_cmd)
        	{
				min_prop_cmd = prop_cmd_d[i];
        	}
      	}

	Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
	Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
	Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);

	prop_cmd_d = prop_cmd_d + this->nB * x;
	this->prop_cmd_d = prop_cmd_d.array().max(0);
	}
	else
	{
		Eigen::Vector3d force = linearWrench_end;
		this->wrench << force[0], force[1], force[2], 0., 0., 0.;
		prop_cmd_d = (this->iG * this->wrench);
    	double min_prop_cmd = 100000;


		for(int i = 0; i < this->n; i++) {
      		if (prop_cmd_d[i] < min_prop_cmd)
        	{
				min_prop_cmd = prop_cmd_d[i];
        	}
       }	
	Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
	Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
	Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);

	prop_cmd_d = prop_cmd_d + this->nB * x;

	this->prop_cmd_d = prop_cmd_d.array().max(0);	}
	last_wrench_sent = false;
}


// Angular Wrench is a service for wrench testing
// Wrench testing can only be used when lift_off is not yet called
// Angular wrench requires a start z-force and two angles to rotate this force about y-axis (radians), and duration
// wrench will be applied depending on the time.
bool Controller::angular_wrench_callback(controller::AngularWrench::Request &req, controller::AngularWrench::Response &res) {
	
	if(this->test_wrench_received) {
		res.status = false;
		return false;
	} else {
		this->wrench_max_time = req.duration;
		this->wrench_start_time = ros::Time::now();
		this-> angularWrench_f = req.fz;
		this->angularWrenchPhi_1 = req.phi1;
		this->angularWrenchPhi_2 = req.phi2;
		this->test_wrench_received = true;
		this->angularWrench = true;
		res.status = true;
		return true;
	}
}

void Controller::apply_angular_wrench_test(){
	ros::Time thisTime = ros::Time::now();
	double seconds = (thisTime - this->wrench_start_time).toSec();
	Eigen::VectorXd prop_cmd_d(this->n);

	if (seconds > wrench_max_time*(1+(1./3.)))
	{
		for(int i = 0; i < this->n; i++) {
			prop_cmd_d[i] = 0;
		}
		this->prop_cmd_d = prop_cmd_d.array();
		this->test_wrench_received = false;
		this->angularWrench = false;
	} else {
	if (seconds > wrench_max_time*(1./3.))
	{
		double phi = angularWrenchPhi_1 + (angularWrenchPhi_2 - angularWrenchPhi_1)*((seconds - (1./3.)*wrench_max_time)/wrench_max_time);
		
		this->wrench << 0, angularWrench_f*sin(phi), angularWrench_f*cos(phi), 0., 0., 0.;}
		else{
			double phi = angularWrenchPhi_1;
			double force = angularWrench_f* (seconds/wrench_max_time)*3.0;
			this->wrench << force*sin(phi), 0, force*cos(phi), 0., 0., 0.;

		}
			
		prop_cmd_d = (this->iG * this->wrench);
    	double min_prop_cmd = 100000;


		for(int i = 0; i < this->n; i++) {
      		if (prop_cmd_d[i] < min_prop_cmd)
        	{
				min_prop_cmd = prop_cmd_d[i];
        	}
      	}

		Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
		Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
		Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);

		prop_cmd_d = prop_cmd_d + this->nB * x;

		this->prop_cmd_d = prop_cmd_d.array().max(0);

	}
	last_wrench_sent = false;
}


void Controller::log_data() {

	unsigned int i;

	// publish data required to be logged
	omni_firmware::errorMsg controller_log_pub;
	controller_log_pub.ex.x = this->ex.x();
	controller_log_pub.ex.y = this->ex.y();
	controller_log_pub.ex.z = this->ex.z();
	controller_log_pub.ev.x = this->ev.x();
	controller_log_pub.ev.y = this->ev.y();
	controller_log_pub.ev.z = this->ev.z();
	controller_log_pub.ea.x = this->ea.x();
	controller_log_pub.ea.y = this->ea.y();
	controller_log_pub.ea.z = this->ea.z();
	controller_log_pub.eR.x = this->eR.x();
	controller_log_pub.eR.y = this->eR.y();
	controller_log_pub.eR.z = this->eR.z();
	controller_log_pub.ew.x = this->ew.x();
	controller_log_pub.ew.y = this->ew.y();
	controller_log_pub.ew.z = this->ew.z();
	controller_log_pub.Iex.x = this->Iex.x();
	controller_log_pub.Iex.y = this->Iex.y();
	controller_log_pub.Iex.z = this->Iex.z();
	controller_log_pub.Ier.x = this->Ier.x();
	controller_log_pub.Ier.y = this->Ier.y();
	controller_log_pub.Ier.z = this->Ier.z();
	controller_log_pub.Accd.x = this->acc_d.x();
	controller_log_pub.Accd.y = this->acc_d.y();
	controller_log_pub.Accd.z = this->acc_d.z();
	std::vector<float> wrench(this->wrench.data(), this->wrench.data() + this->wrench.size());
	std::vector<float> prop_vel(prop_cmd_d.data(), prop_cmd_d.data() + prop_cmd_d.size());
	
	controller_log_pub.Wrench = wrench;
	controller_log_pub.prop = prop_vel;
	controller_log_pub.weight = this->weight_;

	this->controller_log_pub.publish(controller_log_pub);

}



void Controller::update_prop_cmd() {
	// this method updates the propeller commands topic
	// advertised by the Controller node
	omni_firmware::MotorSpeed prop_cmd;
	std::vector < std::string > prop_names = { "propeller1", "propeller2", "propeller3", "propeller4", "propeller5", "propeller6","propeller7","propeller8" };
	std::vector<float> prop_vel(prop_cmd_d.data(), prop_cmd_d.data() + prop_cmd_d.size());
	for(int i = 0; i < this->n; i++) {
		if(prop_vel[i] > this->cmd_max)
			prop_vel[i] = this->cmd_max;
		else if(prop_vel[i] < this->cmd_min)
			prop_vel[i] = this->cmd_min;
	}

	Eigen::ArrayXd a_vals(this->n);  // Vector of 'a' values, one per row
	Eigen::ArrayXd b_vals(this->n);  // Vector of 'b' values, one per row
	Eigen::ArrayXd c_vals(this->n);  // Vector of 'c' values, one per row

	// Initialize a, b, c values
	a_vals << 128.8304,
  			  105.0259,
  			  110.1237,
  			  110.4929,
  			  104.4360,
  			  118.3979,
  			  126.6510,
  			  109.5292; 
	b_vals << 4.5982,
    		  3.8684,
    		  3.5600,
    		  6.1778,
    		  5.7502,
    		  5.3335,
    		  6.0286,
    		  4.2650;
	c_vals << 993.1251,
  			  998.0056,
  			  995.9796,
  			  996.8340,
  			  996.5013,
  			  996.2702,
  			  994.8973,
  			  995.8504;
	int xcount = 0;
	Eigen::ArrayXd prop_PWM;
	prop_PWM = a_vals * prop_cmd_d.array().sqrt() +
			   b_vals * prop_cmd_d.array() + 
			   c_vals;
	while (xcount <= 7)
	{
		if (prop_PWM[xcount] <= 1030)
			prop_PWM[xcount] = 1000;
		if (prop_PWM[xcount] >= 1900)
			prop_PWM[xcount] = 1900;
		prop_vel[xcount] = prop_PWM[xcount];
		xcount++;			
	}
	prop_cmd.name = prop_names;
	prop_cmd.velocity = prop_vel;
	this->prop_cmd_pub.publish(prop_cmd);

}

void Controller::send_zero_PWM() {
    omni_firmware::MotorSpeed prop_cmd;
    std::vector<std::string> prop_names = { 
        "propeller1", "propeller2", "propeller3", "propeller4", 
        "propeller5", "propeller6", "propeller7", "propeller8" 
    };
    
    std::vector<float> prop_vel(prop_names.size(), 1000.0f);
    prop_cmd.name = prop_names;
    prop_cmd.velocity = prop_vel;
    this->prop_cmd_pub.publish(prop_cmd);
}

void Controller::send_idle_PWM() {
    omni_firmware::MotorSpeed prop_cmd;
    std::vector<std::string> prop_names = { 
        "propeller1", "propeller2", "propeller3", "propeller4", 
        "propeller5", "propeller6", "propeller7", "propeller8" 
    };
    
    std::vector<float> prop_vel(prop_names.size(), 1060.0f);
    prop_cmd.name = prop_names;
    prop_cmd.velocity = prop_vel;
    this->prop_cmd_pub.publish(prop_cmd);
}

void Controller::toggleCallback(const std_msgs::Bool::ConstPtr &msg)
{
	this->armed = msg->data;
    ROS_INFO("Armed Controller: %s", this->armed ? "true" : "false");
	if (!this->armed) this->first_desired_received = false;
}


void Controller::poseCallback(const omni_firmware::FullPose &_pose) {
	this->pose_ = _pose;
	Eigen::Quaternion<double> q;
	q.coeffs() << _pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z, _pose.pose.orientation.w;
	this->R = q.toRotationMatrix();
	this->first_pose_received = true;
}

void Controller::poseDCallback(const omni_firmware::FullPose &_pose_d) {
	this->pose_d = _pose_d;
	Eigen::Quaternion<double> qd;
	qd.coeffs() << _pose_d.pose.orientation.x, _pose_d.pose.orientation.y, _pose_d.pose.orientation.z, _pose_d.pose.orientation.w;
	this->Rd = qd.toRotationMatrix();
	if (!this->first_desired_received){
		this->first_desired_received = true;
		this->active_integrator = true;
		this->height_at_lift_off = this->pose_.pose.position.z;

	}
}



void Controller::position_controller() {
	Eigen::Vector3d ex, ev, ea, Iex;
	Eigen::Vector3d e1;
	Eigen::Vector3d e2;
	Eigen::Vector3d e3;
	e1 << 1, 0, 0;
	e2 << 0, 1, 0;
	e3 << 0, 0, 1;

	Iex << 0.0, 0.0, 0.0;
	double weight_;
	ros::Time thisTime = ros::Time::now();
	double seconds = (thisTime - this->ramp_time).toSec();
	double launchTime, secondTime, thirdTime, fourthTime;
	if (this->ground_config_flag)
	{
		launchTime =  1.0;
		secondTime =  1.2;
		thirdTime  = 1.4;
		fourthTime  = 1.6;
	} else
	{
		launchTime =  1.0;
		secondTime =  5.0;
		thirdTime  = 9.0;
		fourthTime  = 10.0;
	} 

		

		ex << this->pose_d.pose.position.x - this->pose_.pose.position.x, 
              this->pose_d.pose.position.y - this->pose_.pose.position.y, 
              this->pose_d.pose.position.z - this->pose_.pose.position.z;
		ev << this->pose_d.vel.linear.x - this->pose_.vel.linear.x, 
              this->pose_d.vel.linear.y - this->pose_.vel.linear.y, 
              this->pose_d.vel.linear.z - this->pose_.vel.linear.z;
		ea << this->pose_d.acc.linear.x,
		this->pose_d.acc.linear.y, 
		this->pose_d.acc.linear.z; 

		int i;
		for(i = 0; i < 3; i++) {
			if(fabs(ex(i)) > this->sat_ex)
				ex(i) = copysign(this->sat_ex, ex(i));
		}

		for(i = 0; i < 3; i++) {
			if(fabs(ev(i)) > this->sat_ev) {
				ev(i) = copysign(this->sat_ev, ev(i));
			}
		}
		
		// Execute integrator only if active
		if(this->active_integrator & seconds > fourthTime){
			Iex << this->Iex.x(), this->Iex.y(), this->Iex.z();
			Iex += ex * 1.0/((float)this->period);
			
			for(i = 0; i < 3; i++) {
				if(fabs(Iex(i)) > (this->sat_iex)) {
					Iex(i) = copysign((this->sat_iex), Iex(i));
				}
			}
			this->Iex = Iex;
		}

		if(this->log_data_flag) {
			this->ex = ex;
			this->ev = ev;
			this->ea = ea;
		}
	
		if (seconds >thirdTime) weight_ = this->weight;
		else{
			if (seconds > secondTime)
			weight_ = this->weight*0.8  + this->weight*0.2*(seconds - secondTime)/(thirdTime - secondTime);
			else {
				if (seconds > launchTime)
					weight_ = this->weight*0.8;
				else
					weight_ = this->weight*0.25 + this->weight*0.55*(seconds/launchTime);
			}
		}
		this->weight_ = weight_;		

		
		if (seconds > fourthTime)	
			this->acc_d = this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array() + weight_ * (ea.array() + 9.81 * e3.array());
		else if (seconds > thirdTime)
		{       Eigen::Vector3d multiplier;
			multiplier(0) =  1.0;
			multiplier(1) = 1.0;
			multiplier(2) = 1.0;
			this->acc_d = multiplier.array()* (this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array()) +
                                         weight_ * (ea.array() + 9.81 * e3.array());
                }
		else if (seconds > secondTime )
                {
			Eigen::Vector3d multiplier;                 
                        multiplier(0) =  1.0;
                        multiplier(1) = 1.0;
                        multiplier(2) = (seconds - secondTime)/(thirdTime - secondTime);
			this->acc_d = multiplier.array()* (this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array()) +
                                         weight_ * (ea.array() + 9.81 * e3.array());
                }

		else if (seconds > launchTime )
		{
			Eigen::Vector3d multiplier;                 
			 multiplier(0) =  (seconds - launchTime)/(secondTime - launchTime);
                        multiplier(1) = multiplier(0);
                        multiplier(2) = 0.0;
			this->acc_d = multiplier.array()* (this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array()) +
                                         weight_ * (ea.array() + 9.81 * e3.array());
                }
		else
                        this->acc_d =  weight_ * (ea.array() + 9.81 * e3.array());
}

void Controller::attitude_controller() {

	Eigen::Matrix3d E;
	Eigen::Matrix3d Rd;
	Eigen::Vector3d eR;
	Eigen::Vector3d Ier;

	Eigen::Vector3d w;
	Eigen::Vector3d wd;
	Eigen::Vector3d ew;

	Ier << 0.0, 0.0, 0.0;

    Rd = this->Rd;
	
	E = 0.5 * (Rd.transpose() * this->R - this->R.transpose() * Rd);
	eR << (E(2, 1) - E(1, 2)) / 2., (E(0, 2) - E(2, 0)) / 2., (E(1, 0) - E(0, 1)) / 2.;

	w << pose_.vel.angular.x, pose_.vel.angular.y, pose_.vel.angular.z;
	wd << pose_d.vel.angular.x, pose_d.vel.angular.y, pose_d.vel.angular.z;
	wd = this->R.transpose() * wd;
	ew = w - wd;

	// Execute integrator only if active

	
    for(int i=0 ; i<3 ; i=i+1 ){
      if( isnan(this->Ier(i))){
            this->Ier << 0.0, 0.0, 0.0;
        }
    }
	// Execute integrator only if active
	if(this->active_integrator){
		
    Ier << this->Ier.x() , this->Ier.y(), this->Ier.z();
    Ier += eR * 1.0/(this->period * 1.0f);

    for(int i = 0; i < 3; i++) {
        if(fabs(Ier(i)) > (this->sat_ier)) {
            Ier(i) = copysign((this->sat_ier), Ier(i));
        }
    }
		}
    this->Ier = Ier;

	this->eR = eR;
	this->ew = ew;

}

void Controller::coplanar_collinear_actuation() {
	
    double Z = this->acc_d.dot(this->R.col(2));
	this->wrench.block<3, 1>(0, 0) << 0., 0., Z;
	this->wrench.block<3, 1>(3, 0) << -this->att_kp * this->eR.array() 
      - this->att_kd * this->ew.array() 
      - this->att_ki * this->Ier.array();
	
	if(this->wrench[3] > this->RP_MOMENT_LIMIT){
		this->wrench[3] = this->RP_MOMENT_LIMIT;
	} else{
		if(this->wrench[3] < -this->RP_MOMENT_LIMIT){
			this->wrench[3] = -this->RP_MOMENT_LIMIT;
		}		
	}
	
	if(this->wrench[4] > this->RP_MOMENT_LIMIT){
		this->wrench[4] = this->RP_MOMENT_LIMIT;
	} else{
		if(this->wrench[4] < -this->RP_MOMENT_LIMIT){
			this->wrench[4] = -this->RP_MOMENT_LIMIT;
		}		
	}

	if(this->wrench[5] > this->YAW_MOMENT_LIMIT){
		this->wrench[5] = this->YAW_MOMENT_LIMIT;
	} else{
		if(this->wrench[5] < -this->YAW_MOMENT_LIMIT){
			this->wrench[5] = -this->YAW_MOMENT_LIMIT;
		}		
	}

    Eigen::VectorXd prop_cmd_d(this->n * 3);

	prop_cmd_d = (this->iG * this->wrench);
    double min_prop_cmd = 100000;
    double prop_nF_val;
    int counter = 0;
    float lamda;
    Eigen::VectorXd nullSpaceVector(8);
    nullSpaceVector << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;

	for(int i = 0; i < this->n; i++) {
      if (prop_cmd_d[i] < min_prop_cmd)
        {
			min_prop_cmd = prop_cmd_d[i];
			counter = i;
        }
      }

	Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
	Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
	Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);
	prop_cmd_d = prop_cmd_d + this->nB * x;
	this->prop_cmd_d = prop_cmd_d.array().max(0);
}


void Controller::full_allocation_actuation() {
	
	Eigen::Array3d att_kp;
	ros::Time thisTime = ros::Time::now();
	double alpha = 0;
	double trans_time = 1.0;
	double seconds = (thisTime - this->ramp_time).toSec();

	if (this->ground_start == true)
	{
		att_kp = this->att_kp_ground;
		this->Ier << 0., 0., 0.;
		if (this->pose_.pose.position.z - this->height_at_lift_off >= this->dz_pre_trans)
		{
			this->ground_start = false;
			this->time_at_lift_off = seconds;
		}
	}

	else if (seconds < this->time_at_lift_off + trans_time)
	{
		alpha = (seconds - this->time_at_lift_off)/trans_time;
		att_kp = alpha*this->att_kp + (1.0-alpha)*this->att_kp_ground;
	}

	else
	{
		att_kp = this->att_kp;
	}

	this->wrench.block<3, 1>(0, 0) << this->R.transpose() * this->acc_d;
	this->wrench.block<3, 1>(3, 0) << -att_kp * this->eR.array() 
      - this->att_kd * this->ew.array() 
      - this->att_ki * this->Ier.array();


	if(this->wrench[3] > this->RP_MOMENT_LIMIT){
		this->wrench[3] = this->RP_MOMENT_LIMIT;
	} else{
		if(this->wrench[3] < -this->RP_MOMENT_LIMIT){
			this->wrench[3] = -this->RP_MOMENT_LIMIT;
		}		
	}
	
	if(this->wrench[4] > this->RP_MOMENT_LIMIT){
		this->wrench[4] = this->RP_MOMENT_LIMIT;
	} else{
		if(this->wrench[4] < -this->RP_MOMENT_LIMIT){
			this->wrench[4] = -this->RP_MOMENT_LIMIT;
		}		
	}

	if(this->wrench[5] > this->YAW_MOMENT_LIMIT){
		this->wrench[5] = this->YAW_MOMENT_LIMIT;
	} else{
		if(this->wrench[5] < -this->YAW_MOMENT_LIMIT){
			this->wrench[5] = -this->YAW_MOMENT_LIMIT;
		}		
	}

    Eigen::VectorXd prop_cmd_d(this->n);

	prop_cmd_d = (this->iG * this->wrench);
    double min_prop_cmd = 100000;


	for(int i = 0; i < this->n; i++) {
      if (prop_cmd_d[i] < min_prop_cmd)
        {
			min_prop_cmd = prop_cmd_d[i];
        }
      }

	Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
	Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
	Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);
	prop_cmd_d = prop_cmd_d + this->nB * x;
	this->prop_cmd_d = prop_cmd_d.array().max(0);
 
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    Controller controller_;
    controller_.period = 250;
    controller_.log_data_flag = true;
    ros::Rate loop_rate(controller_.period);

    while (ros::ok()) {
        ros::Time thisTime = ros::Time::now();
        if (controller_.armed) {
            if (controller_.first_pose_received && controller_.first_desired_received) {
                controller_.position_controller();
                controller_.attitude_controller();

                controller_.full_allocation_actuation();
                controller_.update_prop_cmd();

                if (controller_.log_data_flag) {
                    controller_.log_data();
                }
            } else {
                controller_.ramp_time = thisTime;

                if (controller_.test_wrench_received) {
                    if (controller_.linearWrench) {
                        controller_.apply_linear_wrench_test();
                    } else if (controller_.angularWrench) {
                        controller_.apply_angular_wrench_test();
                    }
                    controller_.update_prop_cmd();
                    if (controller_.log_data_flag) {
                        controller_.log_data();
                    }
                } else {
                    controller_.send_idle_PWM();
                    if (controller_.log_data_flag) {
                        controller_.log_data();
                    }
                }
            }
        } else {
            controller_.send_zero_PWM();
            controller_.first_desired_received = false;
            if (controller_.log_data_flag) {
                controller_.log_data();
            }
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



