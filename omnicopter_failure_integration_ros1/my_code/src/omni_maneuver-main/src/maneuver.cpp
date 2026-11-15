#include "maneuver.hpp"
#include "helpers.cc"
#include "maneuver_codes.cc"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>

#include <libkdtp/libkdtp.h>

#include <omni_firmware/Pose.h>
#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>
#include <omni_firmware/errorMsg.h>
#include "maneuver/ArmDisarm.h"
#include "maneuver/LiftOff.h"
#include "maneuver/GotoPoint.h"
#include "maneuver/Goto6DPoint.h"
#include "maneuver/RotateTo.h"
#include "maneuver/Land.h"
#include "maneuver/Ellipse5D.h"
#include "maneuver/FullFlip.h"

#include <sstream>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Core>

// this node reads:
// platform current pose
// platform desired pose
// outputs:
// propeller commands

//TODO: add waypoint navigation

Maneuver::Maneuver() : nh_(), robot("rotorcraft") {

	this->path._length = 0;
	this->path._maximum = 15000;
	// the following are containers to get topic names
	std::string log_data_topic = "maneuver_log";
	std::string pose_topic;
	std::string pose_d_topic;
	if(!nh_.getParam("maneuver/pose", pose_topic)) {
		ROS_ERROR("Could not find pose parameter!, setting DEFAULT");
		pose_topic = "pose";
	}
	if(!nh_.getParam("maneuver/pose_d", pose_d_topic)) {
		ROS_ERROR("Could not find desired pose parameter!, setting DEFAULT");
		pose_d_topic = "pose_d";
	}
	pose_d_pub = nh_.advertise < omni_firmware::FullPose > (pose_d_topic, 1000);

	pose_sub = nh_.subscribe(pose_topic, 1000, &Maneuver::poseCallback, this);

	toggle_pub = nh_.advertise<std_msgs::Bool>("/toggle_topic", 1000);

	mv_plan_start(this);

	this->end_time = ros::Time::now().toSec() - 1;
	this->time = ros::Time::now().toSec();
	this->log_data_flag = false;

}

Maneuver::~Maneuver() {
}

void Maneuver::log_data() {
	// publish data required to be logged
}

//bool Maneuver::lift_off_callback(std_srvs::Empty::Request& request, 
//      std_srvs::Empty::Response& response){
bool Maneuver::lift_off_callback(maneuver::LiftOff::Request &req, maneuver::LiftOff::Response &res) {
	double height = req.height;
	double duration = req.duration;
	if(this->lift_off_flag || !Armed) {
		res.status = false;
		return false;
	} else {
		end_time = ros::Time::now().toSec() + this->lift_off_time;

		bool status = mv_plan_take_off(this, this->pose_, height, duration, &this->path);
		res.status = status;
		if(status) {
			this->lift_off_flag = true;
			this->moving = true;
			this->moving_step = 0;
			return true;
		} else {
			return false;
		}
	}
}

bool Maneuver::land_callback(maneuver::Land::Request &req, maneuver::Land::Response &res)
{
	double height_1 = req.height_1;
	double duration_1 = req.duration_1;
	double height_2 = req.height_2;
	double duration_2 = req.duration_2;
	if(!this->lift_off_flag || !Armed) {
		res.status = false;
		return false;
	} else {
		end_time = ros::Time::now().toSec() + this->lift_off_time;

		bool status = mv_plan_land(this, this->pose_, height_1, duration_1, height_2, duration_2, &this->path, this->moving);
		res.status = status;
		if(status) {
			if(!this->moving){
			this->moving = true;
			this->moving_step = 0;}
			return true;
		} else {
			return false;
		}
	}	
}


bool Maneuver::full_flip_callback(maneuver::FullFlip::Request &req, maneuver::FullFlip::Response &res)
{
	bool roll_bool = req.roll_bool;
	bool pitch_bool = req.pitch_bool;
	double duration = req.duration;
	if(!this->lift_off_flag || !Armed) {
		res.status = false;
		return false;
	} else {
		end_time = ros::Time::now().toSec() + this->lift_off_time;

		bool status = mv_plan_flip(this, this->pose_, roll_bool, pitch_bool, duration, &this->path);
		res.status = status;
		if(status) {
			if(!this->moving){
			this->moving = true;
			this->moving_step = 0;}
			return true;
		} else {
			return false;
		}
	}	
}


bool Maneuver::ArmDisarm_callback(maneuver::ArmDisarm::Request &req, maneuver::ArmDisarm::Response &res) {
        Armed = !Armed;
        ROS_ERROR("Armed Maneuver %s", Armed ? "true" : "false");
		if (!Armed)
			this->lift_off_flag = false;
		std_msgs::Bool msg;
    	msg.data = Armed;
    	this->toggle_pub.publish(msg);
        res.success = true;
        return true;
}

bool Maneuver::goto_callback(maneuver::GotoPoint::Request &req, maneuver::GotoPoint::Response &res) {
	double x = req.x;
	double y = req.y;
	double z = req.z;
	double w = req.yaw;
	double duration = req.duration;
	if(!this->lift_off_flag) {
		res.status = false;
		return false;
	} else {
		end_time = ros::Time::now().toSec() + this->lift_off_time;

		bool status = mv_goto(this, this->pose_, x, y, z, w, duration, &this->path, this->moving);
		res.status = status;
		if(status) {
			if(!this->moving){
			this->moving = true;
			this->moving_step = 0;}
			return true;
		} else {
			return false;
		}
	}
}

bool Maneuver::goto_6D_callback(maneuver::Goto6DPoint::Request &req, maneuver::Goto6DPoint::Response &res) {
	double x = req.x;
	double y = req.y;
	double z = req.z;
	double roll = req.roll;
	double pitch = req.pitch;
	double yaw = req.yaw;
	double duration = req.duration;
	if(!this->lift_off_flag) {
		res.status = false;
		return false;
	} else {
		end_time = ros::Time::now().toSec() + this->lift_off_time;

		bool status = mv_goto_6D(this, this->pose_, x, y, z, roll, pitch, yaw, duration, &this->path, this->moving);
		res.status = status;
		if(status) {
			if(!this->moving){
			this->moving = true;
			this->moving_step = 0;}
			return true;
		} else {
			return false;
		}
	}
}


bool Maneuver::goto_5DEllipse_callback(maneuver::Ellipse5D::Request &req, maneuver::Ellipse5D::Response &res)
{

	double x_min = req.x_min;
	double x_max = req.x_max;
	double y_min = req.y_min;
	double y_max = req.y_max;
	double z_min = req.z_min;
	double z_max = req.z_max;
	double roll_start = req.roll_start;
	double roll_mid = req.roll_mid;
	double roll_end = req.roll_end;
	double pitch_start = req.pitch_start;
	double pitch_mid = req.pitch_mid;
	double pitch_end = req.pitch_end;
	double yaw = req.yaw;
	double duration = req.duration;
	if(!this->lift_off_flag || !Armed) {
		res.status = false;
		return false;
	} else {
		end_time = ros::Time::now().toSec() + this->lift_off_time;

		bool status = mv_plan_5DEllipse(this, this->pose_, 
						x_min, x_max, 
						y_min, y_max, 
						z_min, z_max, 
						roll_start, roll_mid, roll_end, 
						pitch_start, pitch_mid, pitch_end,
						yaw, duration,  &this->path, this->moving);
		res.status = status;
		if(status) {
			if(!this->moving){
			this->moving = true;
			this->moving_step = 0;}
			return true;
		} else {
			return false;
		}
	}	
}

bool Maneuver::rotateTo_callback(maneuver::RotateTo::Request &req, maneuver::RotateTo::Response &res) {
	double roll = req.roll;
	double pitch = req.pitch;
	double yaw = req.yaw;
	double duration = req.duration;
	if(!this->lift_off_flag) {
		res.status = false;
		return false;
	} else {
		end_time = ros::Time::now().toSec() + this->lift_off_time;

		bool status = mv_rotateTo(this, this->pose_, roll, pitch, yaw, duration, &this->path, this->moving);
		res.status = status;
		if(status) {
			if(!this->moving){
			this->moving = true;
			this->moving_step = 0;}
			return true;
		} else {
			return false;
		}
	}
}
void Maneuver::publishDesired() {
	if(this->moving && this->moving_step >= this->path._length) {
		this->moving = false;
	} else {
		if(this->moving) {
			this->pose_d = this->path._buffer[this->moving_step];
			this->moving_step++;
		}
	}
	this->pose_d_pub.publish(this->pose_d);
}

void Maneuver::poseCallback(const omni_firmware::FullPose &_pose) {
	this->pose_ = _pose;
	Eigen::Quaternion<double> q;
	q.coeffs() << _pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z, _pose.pose.orientation.w;
	this->R = q.toRotationMatrix();
	Eigen::Vector3d gravity;
	gravity << 0., 0., 9.81;
	gravity = this->R * gravity;
	this->pose_.acc.linear.x = _pose.acc.linear.x + gravity(0);
	this->pose_.acc.linear.y = _pose.acc.linear.y + gravity(1);
	this->pose_.acc.linear.z = _pose.acc.linear.z + gravity(2);
	this->first_pose_received = true;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "maneuver");

	Maneuver maneuver_;
	maneuver_.period = 100;
	maneuver_.log_data_flag = false;

	ros::ServiceServer ArmDisarm_service = maneuver_.nh_.advertiseService("ArmDisarm", &Maneuver::ArmDisarm_callback, &maneuver_);

	ros::ServiceServer lift_off_service = maneuver_.nh_.advertiseService("lift_off", &Maneuver::lift_off_callback, &maneuver_);

	ros::ServiceServer land_service = maneuver_.nh_.advertiseService("land", &Maneuver::land_callback, &maneuver_);

	ros::ServiceServer goto_service = maneuver_.nh_.advertiseService("go_to", &Maneuver::goto_callback, &maneuver_);

	ros::ServiceServer goto_6D_service = maneuver_.nh_.advertiseService("go_to_6D", &Maneuver::goto_6D_callback, &maneuver_);

	ros::ServiceServer rotate_service = maneuver_.nh_.advertiseService("rotate_to", &Maneuver::rotateTo_callback, &maneuver_);

	ros::ServiceServer ellipse5D_service = maneuver_.nh_.advertiseService("ellipse5D", &Maneuver::goto_5DEllipse_callback, &maneuver_);

	ros::ServiceServer full_flip_service = maneuver_.nh_.advertiseService("fullFlip", &Maneuver::full_flip_callback, &maneuver_);
	ros::Rate loop_rate(maneuver_.period);

	int count = 0;

	while(ros::ok()) {
		ros::spinOnce();
		maneuver_.time = ros::Time::now().toSec();

		if(maneuver_.lift_off_flag) {
			maneuver_.publishDesired();

		}

		loop_rate.sleep();
		++count;
	}

	return 0;

}

