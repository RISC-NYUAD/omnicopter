#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <omni_firmware/Pose.h>
#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>
#include <omni_firmware/Uvector.h>

#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>



#include "controller/LinearWrench.h"
#include "controller/AngularWrench.h"


// this node reads:
// platform current pose
// platform desired pose
// outputs:
// propeller commands

// #define DYAW_ERROR_LIMIT 			(1.5707963267948966f)				// Maximum error in yaw rate. 90 deg/s
// #define YAW_ERROR_LIMIT 			(0.5235987755982988)				// Maximum error in yaw rate. 30 deg
//#define YAW_MOMENT_LIMIT			10.0								// Maximum yaw moment command. Unit: N

#define ONGROUND_THR_THREASHOLD    5.0f							// acceleration threashold to determine state
#define AIRBORNE_THR_THREASHOLD    8.0f							// acceleration threashold to determine state

#define STATE_TIMER_THREASHOLD     50							// timer ticks to debounce acceleration threashold

#define POS_MOD_MAN				0
#define POS_MOD_XY				1
#define POS_MOD_XYZ				2

enum ENUM_FLIGHT_STATE {STATE_AIRBORNE, STATE_ONGROUND};
enum ENUM_STATE_EVENT {evNOEVENT, evAIRBORNE, evONGROUND};

class Controller {
protected:
	ros::NodeHandle nh_;
	// containers for published and subscribed threads
	omni_firmware::FullPose pose_;
	omni_firmware::FullPose pose_d;
	omni_firmware::MotorSpeed prop_cmd;
	// publisher thread for propeller commands
	ros::Publisher prop_cmd_pub;
	// publisher thread controller logging
	ros::Publisher controller_log_pub;
	// subsriber threads to platform pose and desired pose
	ros::Subscriber pose_sub;
	ros::Subscriber pose_d_sub;
	ros::Subscriber opr_cmd_sub;
	// rosservice callbacks
	ros::ServiceServer linearWrench_service;
	ros::ServiceServer angularWrench_service;
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
	ros::Time last_exW_est_time;

	ENUM_FLIGHT_STATE flight_state;				// onground/airbrone state indicator
	bool active_integrator;						// flag used to enable and disable the integrators
	int state_timer;

	ros::Publisher Uvector_pub;
	ros::Publisher Uvector_new_pub;
 	ros::Subscriber toggle_sub;

	double YAW_MOMENT_LIMIT, RP_MOMENT_LIMIT;

public:
	Controller();

	~Controller();

	void update_prop_cmd();
	void send_idle_PWM();
	void send_zero_PWM();
	void log_data();

	void toggleCallback(const std_msgs::Bool::ConstPtr &msg);
	void poseCallback(const omni_firmware::FullPose &_pose);
	void poseDCallback(const omni_firmware::FullPose &_pose_d);

	void position_controller();
	void attitude_controller();
	void coplanar_collinear_actuation();
	void full_allocation_actuation();
	
	bool linear_wrench_callback(controller::LinearWrench::Request &req, controller::LinearWrench::Response &res);
	void apply_linear_wrench_test();
	bool angular_wrench_callback(controller::AngularWrench::Request &req, controller::AngularWrench::Response &res);
	void apply_angular_wrench_test();

	void compute_external_wrench();

	ros::Time ramp_time;
	
	bool first_pose_received = false;
	bool first_desired_received = false;
	int period;
	bool log_data_flag = false;
	
	ros::Time wrench_start_time;
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

	unsigned int pos_ctrl_mode = false;
    bool full_actuation = false; // flag used to enable full actuation (on true)

	omni_firmware::Uvector u_vect;
	omni_firmware::Uvector u_new_vect;

    double* positives_array;
};

#endif

