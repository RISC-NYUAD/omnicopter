#ifndef STATE_HPP
#define STATE_HPP

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/AccelStamped.h"
#include <std_srvs/Empty.h>
#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/tf.h>

#include <thread>
#include <mutex>
#include <serial/serial.h>
#include <vector>
#include <cmath> 
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

class State {
protected:
	// containers for published and subscribed threads
	omni_firmware::FullPose full_pose;
    geometry_msgs::Pose pose_;
    geometry_msgs::Twist twist_;
    geometry_msgs::Accel accel_;

	// publisher thread desired pose
	ros::Publisher pose_pub;
	ros::Publisher imu_pub;
	ros::Publisher gps_pub;
    ros::Subscriber pose_sub;
    ros::Subscriber twist_sub;
	ros::Subscriber cmd_sub;
    //ros::Subscriber accel_sub;
	
	const int dataLength = 18+18+6;
	sensor_msgs::Imu imu_data;
	sensor_msgs::NavSatFix gps_data;
	float yaw;
	uint8_t buffer_read[18+6+18];            
	uint8_t MSP_motor_buffer[22];
	uint8_t MSP_motor_Idle_buffer[22];
	bool initialized = false;
	ros::Time ramp_time;
	void poseCallback(const geometry_msgs::PoseStamped &_pose);
	void twistCallback(const geometry_msgs::TwistStamped &_twist);
    
    const int wind = 50;
	std::vector<double> filtered_signal_vicon_buffer_x;
	std::vector<double> filtered_signal_vicon_buffer_y;
	std::vector<double> filtered_signal_vicon_buffer_z;
	std::vector<double> vicon_twist_time;
	double old_vicon_pose_x;
	double old_vicon_pose_y;
	double old_vicon_pose_z;
	
	int counter;
	double pos_time;
	uint32_t seqID;
	ros::Time state_last_time;

	// GPS and Barometer data
	uint32_t gps_counter = 0;
	bool new_gps_data = false;
	double lattitude, longitude;
	double baro_alt;


public:
	ros::NodeHandle nh_;
	State();

	~State();
	serial::Serial ser;
	std::mutex dataMutex;
	std::mutex writeMutex;
	std::mutex cmdMutex;
	std::vector<float> motor_speeds;
	
	void publishFullPose();
	void readData();
	void IMU_callback();
	void sendMSPRequest();
	void MotorCmdCallback(const omni_firmware::MotorSpeed &prop_cmd);
	void IdleCmd();
	void sendMotorMSP(const bool emergency);
	void processMessage(const std::vector<uint8_t>& message);
	int new_data = 0;
	bool cmd_pending = false;
	
};

#endif
