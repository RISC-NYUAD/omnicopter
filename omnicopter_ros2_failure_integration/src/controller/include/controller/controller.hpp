#ifndef CONTROLLER
#define CONTROLLER

#include "rclcpp/rclcpp.hpp"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
//#include <sensor_msgs/msg/joint_state.hpp>

#include "maneuver/msg/full_pose.hpp"
#include <controller/msg/pose.hpp>
#include <controller/msg/motor_speed.hpp>
#include <controller/msg/uvector.hpp>
#include <controller/msg/error_msg.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <math.h>
#include <cmath>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>

#include <time.h>


#include "controller/srv/linear_wrench.hpp"
#include "controller/srv/angular_wrench.hpp"

class Controller : public rclcpp::Node {
public:
	Controller();

	~Controller();

	void update_prop_cmd();
	void send_idle_PWM();
	void send_zero_PWM();
	void log_data();

	void toggleCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
	void poseCallback(const std::shared_ptr<maneuver::msg::FullPose> _pose);
	void poseDCallback(const std::shared_ptr<maneuver::msg::FullPose> _pose_d);

	void position_controller();
	void attitude_controller();
	void coplanar_collinear_actuation();
	void full_allocation_actuation();
	
	bool linear_wrench_callback(const std::shared_ptr<controller::srv::LinearWrench::Request> req,
                       std::shared_ptr<controller::srv::LinearWrench::Response> res);
	void apply_linear_wrench_test();
	bool angular_wrench_callback(const std::shared_ptr<controller::srv::AngularWrench::Request> req,
                       std::shared_ptr<controller::srv::AngularWrench::Response> res);
	void apply_angular_wrench_test();

	void compute_external_wrench();

	rclcpp::Time ramp_time;
	
	bool first_pose_received = false;
	bool first_desired_received = false;
	int period;
	bool log_data_flag = false;
	
	rclcpp::Time wrench_start_time;
	double wrench_max_time;
	double wrench_ramp_time;
	bool test_wrench_received = false;
	Eigen::Vector3d linearWrench_start, linearWrench_end;
	double angularWrench_f, angularWrenchPhi_1, angularWrenchPhi_2;
	bool linearWrench = false;
	bool angularWrench = false;
	bool last_wrench_sent = true;
	bool armed = false;
		
	double thrust_dmd;
	double accd_x_dmd;
	double accd_y_dmd;

	double weight_;

	std::string oper_mode;
	
	unsigned int pos_ctrl_mode = false;
    bool full_actuation = false; // flag used to enable full actuation (on true)

    double* positives_array;
    
protected:
	// publisher thread for propeller commands
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr prop_cmd_pub;
	// publisher thread controller logging
    rclcpp::Publisher<controller::msg::ErrorMsg>::SharedPtr controller_log_pub;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr toggle_sub;
	// subsriber threads to platform pose and desired pose
    rclcpp::Subscription<maneuver::msg::FullPose>::SharedPtr pose_sub;
    rclcpp::Subscription<maneuver::msg::FullPose>::SharedPtr pose_d_sub;

    rclcpp::Service<controller::srv::LinearWrench>::SharedPtr linear_wrench_service_;
    rclcpp::Service<controller::srv::AngularWrench>::SharedPtr angular_wrench_service_;

	// containers for published and subscribed threads
	maneuver::msg::FullPose pose_;
	maneuver::msg::FullPose pose_d;
	std_msgs::msg::Float64MultiArray prop_cmd;

    // attitude controller pid gains
	Eigen::Array3d att_kp;
	Eigen::Array3d att_kp_ground;
	Eigen::Array3d att_kd;
	Eigen::Array3d att_ki;
	// position controller pid gains
	Eigen::Array3d pos_kp;
	Eigen::Array3d pos_kd;
	Eigen::Array3d pos_ki;
	// propeller limits
	double cmd_max;
	double cmd_min;
	double height_at_lift_off = 0;
	double time_at_lift_off = -10000;
	double dz_pre_trans; // required height change to finish take off
	int ground_start;
	double ground_config_flag;
	double weight; // platform weight
	Eigen::Matrix3d J; //inertia Matrix
	short n; //
	Eigen::Vector3d Iex; // integral position error
	Eigen::Vector3d Ier; // integral angular error
	double sat_ex, sat_ev, sat_iex, sat_ier;
	Eigen::Matrix3d R, Rd; // stored rotMat and desired rotMat
	Eigen::Vector3d Euler;
	// desired rotMat is provided from pose_d
	Eigen::Vector3d acc_d; // calculated desired virtual acceletaion
	Eigen::Matrix<double, 6, 1> wrench; // calculated wrench
	Eigen::Matrix<double, 8, 6> iG; // inverse allocation matrix
	Eigen::Matrix<double, 8, 2> nB; // null space basis of allocation matrix
	Eigen::VectorXd prop_cmd_d; // calculated propeller commands
	Eigen::Vector3d ex, ev, eR, ew, ea;

	// wrench observer variables:
	Eigen::Vector3d exF,exM;
	Eigen::Vector3d LF, LM; // gains for the force and moment computation
	rclcpp::Time last_exW_est_time;

	bool active_integrator;						// flag used to enable and disable the integrators
	int state_timer;

	double YAW_MOMENT_LIMIT, RP_MOMENT_LIMIT;
};

#endif