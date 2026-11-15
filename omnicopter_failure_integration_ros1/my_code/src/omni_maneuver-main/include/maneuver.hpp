#ifndef MANEUVER_HPP
#define MANEUVER_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <omni_firmware/Pose.h>
#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>
#include "maneuver/LiftOff.h"
#include "maneuver/ArmDisarm.h"
#include "maneuver/GotoPoint.h"
#include "maneuver/Goto6DPoint.h"
#include "maneuver/RotateTo.h"
#include "maneuver/Land.h"
#include "maneuver/Ellipse5D.h"
#include "maneuver/FullFlip.h"


#include "libkdtp/libkdtp.h"

#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>

// this node hosts a series of services
// each service publishes desired pose of the platform
// desired pose is published in Full Pose (refer to msg format)
// desired pose is published as a time series published sequentially
// these services include:
// take-off, land, maneuver to pose
//
// Each maneuver sevice will start path from current pose and time

class Maneuver {
protected:
	// containers for published and subscribed threads
	omni_firmware::FullPose pose_d;
	omni_firmware::FullPose pose_;
	// publisher thread desired pose
	ros::Publisher pose_d_pub;
	// subsriber threads to platform pose
	ros::Subscriber pose_sub;
	ros::Publisher toggle_pub;
	Eigen::Matrix3d R;

	void poseCallback(const omni_firmware::FullPose &_pose);

	void log_data();

public:
	ros::NodeHandle nh_;
	Maneuver();

	~Maneuver();

	kdtp::Robot robot;

	struct pose_sequence {
		int _maximum;
		int _length;
		std::vector<omni_firmware::FullPose> _buffer;
	} path;

	bool lift_off_flag = false;
	// time variable is the current time of the machine in seconds
	double time;
	// end_time variable is the time at which the maneuver should
	// end
	double end_time;
	int period;
	bool Armed = false;
	bool first_pose_received;
	bool log_data_flag;
	double lift_off_time = 2;
	double lift_off_height = 1;
	bool moving = false;
	int moving_step = 0;

	bool lift_off_callback(maneuver::LiftOff::Request &req, maneuver::LiftOff::Response &res);
	bool ArmDisarm_callback(maneuver::ArmDisarm::Request &req, maneuver::ArmDisarm::Response &res);
	bool goto_callback(maneuver::GotoPoint::Request &req, maneuver::GotoPoint::Response &res);
	bool goto_6D_callback(maneuver::Goto6DPoint::Request &req, maneuver::Goto6DPoint::Response &res);
	bool goto_5DEllipse_callback(maneuver::Ellipse5D::Request &req, maneuver::Ellipse5D::Response &res);
	bool rotateTo_callback(maneuver::RotateTo::Request &req, maneuver::RotateTo::Response &res);
	bool land_callback(maneuver::Land::Request &req, maneuver::Land::Response &res);
	bool full_flip_callback(maneuver::FullFlip::Request &req, maneuver::FullFlip::Response &res);

	void publishDesired();

};

#endif
