#include "state.hpp"
#include "ros/ros.h"
#include "signal.h"
#include <ros/xmlrpc_manager.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_srvs/Empty.h>
#include <tf/tf.h>
#include <algorithm>



#include <omni_firmware/FullPose.h>
#include <omni_firmware/MotorSpeed.h>

#include <sstream>
#include <vector>
#include <string>
#include <termios.h>
#include <fcntl.h>
#include <time.h>
#include <thread>
#include <mutex>

#define MSP_RAW_IMU 102
#define GRAVITY_MSS 9.80665f
#define M_PIf  3.14159
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>

#include "ukf.h"

int16_t idle_pwm = 1000;
// Signal-safe flag for whether shutdown is requested
sig_atomic_t volatile g_request_shutdown = 0;
bool emergency = false;

State::State() : nh_() {

	pose_pub = nh_.advertise < omni_firmware::FullPose > ("/pose_full", 20);
	imu_pub = nh_.advertise<sensor_msgs::Imu>("/Raw_IMU", 20);
	pose_sub = nh_.subscribe("/vrpn_client_node/Omnicopter/pose", 2, &State::poseCallback, this);
	twist_sub = nh_.subscribe("/vrpn_client_node/Omnicopter/twist", 2, &State::twistCallback, this);
	//accel_sub = nh_.subscribe("/vrpn_client_node/Omnicopter/accel", 1000, &State::accelCallback, this);
	cmd_sub = nh_.subscribe("/MotorSpeed",2,&State::MotorCmdCallback,this);
	gps_pub = nh_.advertise<sensor_msgs::NavSatFix>("/Raw_GPS_baro",10);
	
		try
    {
        ser.setPort("/dev/ttyACM0");
        ser.setBaudrate(115200);
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        ser.setTimeout(to);
        ser.open();
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port ");
            
	}

    if(ser.isOpen()){
        ROS_INFO_STREAM("Serial Port initialized");
    }else{
        ROS_ERROR_STREAM("Unable to initialize port ");
    }
	this->filtered_signal_vicon_buffer_x.reserve(wind);
	this->filtered_signal_vicon_buffer_y.reserve(wind);
	this->filtered_signal_vicon_buffer_z.reserve(wind);
	this->counter = 0;
	this->pos_time = 0;

	this->seqID = 0;
}

State::~State() {
}

void State::publishFullPose() {
  this->full_pose.pose = this->pose_;
  this->full_pose.vel = this->twist_;
  this->full_pose.acc = this->accel_;
  this->full_pose.header.stamp = this->state_last_time;
  this->full_pose.header.frame_id = "MSP_state";
  this->full_pose.header.seq = seqID;
  seqID++;

  this->pose_pub.publish(this->full_pose);
}

void State::poseCallback(const geometry_msgs::PoseStamped &_pose) {
	ros::Time thisTime = ros::Time::now();
    if (this->pos_time == 0)
	{
		this->pose_.position = _pose.pose.position;
		this->pos_time = thisTime.toSec();
		this->old_vicon_pose_x = _pose.pose.position.x;
		this->old_vicon_pose_y = _pose.pose.position.y;
		this->old_vicon_pose_z = _pose.pose.position.z;
	}
	else
	{
		double dt = thisTime.toSec() - this->pos_time;
		this->pos_time += dt;
		double vicon_pos_x = _pose.pose.position.x;
		double vicon_pos_y = _pose.pose.position.y;
		double vicon_pos_z = _pose.pose.position.z;
		double pred_x = dt*this->twist_.linear.x + this->pose_.position.x;
		double pred_y = dt*this->twist_.linear.y + this->pose_.position.y;
		double pred_z = dt*this->twist_.linear.z + this->pose_.position.z;
		
		if ((abs(vicon_pos_x - this->old_vicon_pose_x) > 3*dt )
			|| (abs(vicon_pos_y - this->old_vicon_pose_y) > 3*dt )
			|| (abs(vicon_pos_z - this->old_vicon_pose_z) > 3*dt))
			{this->pose_.position = this->pose_.position;
			this->yaw = this->yaw;}
		else{
			this->pose_.position = _pose.pose.position;
			tf::Quaternion q(_pose.pose.orientation.x, _pose.pose.orientation.y, _pose.pose.orientation.z, _pose.pose.orientation.w);
    			tf::Matrix3x3 m(q);
    			double roll, pitch, yaw;
    			m.getRPY(roll, pitch, yaw);
			this->yaw = yaw;}
	}
		this->old_vicon_pose_x = _pose.pose.position.x;
		this->old_vicon_pose_y = _pose.pose.position.y;
		this->old_vicon_pose_z = _pose.pose.position.z;

		this->state_last_time = thisTime;
	
}

void State::twistCallback(const geometry_msgs::TwistStamped &_twist) {
    double LV_x = _twist.twist.linear.x;
    double LV_y = _twist.twist.linear.y;
    double LV_z = _twist.twist.linear.z;
    ros::Time thisTime = ros::Time::now();
    double seconds = thisTime.toSec(); 
    vicon_twist_time.push_back(seconds);
    if (this->counter < wind) {
        filtered_signal_vicon_buffer_x.push_back(LV_x);
        filtered_signal_vicon_buffer_y.push_back(LV_y);
        filtered_signal_vicon_buffer_z.push_back(LV_z);
        
        this->counter++;
        this->twist_.linear.x = _twist.twist.linear.x * 0.25 + 0.75 * this->twist_.linear.x;
        this->twist_.linear.y = _twist.twist.linear.y * 0.25 + 0.75 * this->twist_.linear.y;
        this->twist_.linear.z = _twist.twist.linear.z * 0.25 + 0.75 * this->twist_.linear.z;
    } else {
        Eigen::VectorXd T(wind+1);
        T = Eigen::Map<Eigen::VectorXd>(&vicon_twist_time[0], vicon_twist_time.size());
        T = T.array() -  T[0];

        Eigen::MatrixXd Ax(wind, 5);
        for (int i = 0; i < wind; ++i) {
            Ax(i, 0) = 1;
            for (int j = 1; j < 5; ++j) {
                Ax(i, j) = pow(T(i), j);
            }
        }

        Eigen::MatrixXd Ay(wind, 5);
        Ay = Ax;

        Eigen::MatrixXd Az(wind, 5);
        Az = Ax;

        Eigen::VectorXd X = (Ax.transpose() * Ax).inverse() * Ax.transpose() * Eigen::Map<Eigen::VectorXd>(&filtered_signal_vicon_buffer_x[0], filtered_signal_vicon_buffer_x.size());
        Eigen::VectorXd Y = (Ay.transpose() * Ay).inverse() * Ay.transpose() * Eigen::Map<Eigen::VectorXd>(&filtered_signal_vicon_buffer_y[0], filtered_signal_vicon_buffer_y.size());
        Eigen::VectorXd Z = (Az.transpose() * Az).inverse() * Az.transpose() * Eigen::Map<Eigen::VectorXd>(&filtered_signal_vicon_buffer_z[0], filtered_signal_vicon_buffer_z.size());

        Eigen::MatrixXd Axx(wind + 1, 5);
        for (int i = 0; i < wind + 1; ++i) {
            Axx(i, 0) = 1;
            for (int j = 1; j < 5; ++j) {
                Axx(i, j) = pow(T(i), j);
            }
        }

        Eigen::MatrixXd Ayy(wind + 1, 5);
        Ayy = Axx;

        Eigen::MatrixXd Azz(wind + 1, 5);
        Azz = Axx;

        Eigen::VectorXd fitted_data_x = Axx * X;
        Eigen::VectorXd fitted_data_y = Ayy * Y;
        Eigen::VectorXd fitted_data_z = Azz * Z;


        double diff_x = std::abs(fitted_data_x(wind) - LV_x);
        double diff_y = std::abs(fitted_data_y(wind) - LV_y);
        double diff_z = std::abs(fitted_data_z(wind) - LV_z);

        if (diff_x > 0.5) {
            //filtered_signal_vicon_buffer_x.push_back(fitted_data_x(wind));
            filtered_signal_vicon_buffer_x.push_back(LV_x);
            this->twist_.linear.x = this->twist_.linear.x;//fitted_data_x(wind);
        } else {
            this->twist_.linear.x = LV_x;
            filtered_signal_vicon_buffer_x.push_back(LV_x);
        }

        if (diff_y > 0.5) {
            //filtered_signal_vicon_buffer_y.push_back(fitted_data_y(wind));
            filtered_signal_vicon_buffer_y.push_back(LV_y);
            this->twist_.linear.y = this->twist_.linear.y;//fitted_data_y(wind);
        } else {
            this->twist_.linear.y = LV_y;
            filtered_signal_vicon_buffer_y.push_back(LV_y);
        }

        if (diff_z > 0.5) {
            //filtered_signal_vicon_buffer_z.push_back(fitted_data_z(wind));
            filtered_signal_vicon_buffer_z.push_back(LV_z);
            this->twist_.linear.z = this->twist_.linear.z;//fitted_data_z(wind);
        } else {
            this->twist_.linear.z = LV_z;
            filtered_signal_vicon_buffer_z.push_back(LV_z);
        }

        filtered_signal_vicon_buffer_x.erase(filtered_signal_vicon_buffer_x.begin());
        filtered_signal_vicon_buffer_y.erase(filtered_signal_vicon_buffer_y.begin());
        filtered_signal_vicon_buffer_z.erase(filtered_signal_vicon_buffer_z.begin());
        vicon_twist_time.erase(vicon_twist_time.begin());
    }

	this->state_last_time = thisTime;

}


void State::IdleCmd(){
// Create idle motor command, and keep it for emergency
    uint8_t buffer[22];
	uint8_t dataSize = 16;
    uint8_t checksum = 0;

    buffer[0] = 36;
    buffer[1] = 77;
    buffer[2] = 60;
    buffer[3] = dataSize;
    buffer[4] = 214;

	
    checksum ^= dataSize;
    checksum ^= 214;

	
	for (size_t j =0; j < 8; j++){	 
    	memcpy(&buffer[5+j], &idle_pwm, sizeof(idle_pwm));
    	for (size_t i = 0; i < sizeof(idle_pwm); i++) {
       		checksum ^= buffer[i + 5 + 2*j];
    	}
	}

    memcpy(&buffer[21], &checksum, sizeof(checksum));

	memcpy(&MSP_motor_Idle_buffer,buffer,sizeof(buffer));
}

void State::MotorCmdCallback(const omni_firmware::MotorSpeed &_prop_cmd){
	

	std::vector<float> motor_speeds(_prop_cmd.velocity.begin(),_prop_cmd.velocity.end());
	this->motor_speeds = motor_speeds;
	
	int16_t motor1Speed, motor2Speed, motor3Speed, motor4Speed, motor5Speed, motor6Speed, motor7Speed, motor8Speed;
	ros::Time thisTime = ros::Time::now();
	if (!this->initialized){
		ramp_time = thisTime;
		this->initialized = true;
		motor1Speed = (int16_t)(1000);
		motor2Speed = (int16_t)(1000);
		motor3Speed = (int16_t)(1000);
		motor4Speed = (int16_t)(1000);
		motor5Speed = (int16_t)(1000);
		motor6Speed = (int16_t)(1000);
		motor7Speed = (int16_t)(1000);
		motor8Speed = (int16_t)(1000);
	
	} else{
		double seconds = (thisTime - ramp_time).toSec();
		if (seconds < 0.2){
			motor1Speed = (int16_t)(1000 + (_prop_cmd.velocity[0]- 1000)*(seconds/0.2));
			motor2Speed = (int16_t)(1000 + (_prop_cmd.velocity[1]- 1000)*(seconds/0.2));
			motor3Speed = (int16_t)(1000 + (_prop_cmd.velocity[2]- 1000)*(seconds/0.2));
			motor4Speed = (int16_t)(1000 + (_prop_cmd.velocity[3]- 1000)*(seconds/0.2));
			motor5Speed = (int16_t)(1000 + (_prop_cmd.velocity[4]- 1000)*(seconds/0.2));
			motor6Speed = (int16_t)(1000 + (_prop_cmd.velocity[5]- 1000)*(seconds/0.2));
			motor7Speed = (int16_t)(1000 + (_prop_cmd.velocity[6]- 1000)*(seconds/0.2));
			motor8Speed = (int16_t)(1000 + (_prop_cmd.velocity[7]- 1000)*(seconds/0.2));
		}else {	
			motor1Speed = (int16_t)(_prop_cmd.velocity[0]);
			motor2Speed = (int16_t)(_prop_cmd.velocity[1]);
			motor3Speed = (int16_t)(_prop_cmd.velocity[2]);
			motor4Speed = (int16_t)(_prop_cmd.velocity[3]);
			motor5Speed = (int16_t)(_prop_cmd.velocity[4]);
			motor6Speed = (int16_t)(_prop_cmd.velocity[5]);
			motor7Speed = (int16_t)(_prop_cmd.velocity[6]);
			motor8Speed = (int16_t)(_prop_cmd.velocity[7]);
		}
	}

	uint8_t dataSize = 16;
    uint8_t checksum = 0;

    // Create buffer for the serial data
    uint8_t buffer[22];

    buffer[0] = 36;
    buffer[1] = 77;
    buffer[2] = 60;
    buffer[3] = dataSize;
    buffer[4] = 214;

    checksum ^= dataSize;
    checksum ^= 214;

    memcpy(&buffer[5], &motor1Speed, sizeof(motor1Speed));
    for (size_t i = 0; i < sizeof(motor1Speed); i++) {
        checksum ^= buffer[i + 5];
    }

    memcpy(&buffer[7], &motor2Speed, sizeof(motor2Speed));
    for (size_t i = 0; i < sizeof(motor2Speed); i++) {
        checksum ^= buffer[i + 7];
    }

    memcpy(&buffer[9], &motor3Speed, sizeof(motor3Speed));
    for (size_t i = 0; i < sizeof(motor3Speed); i++) {
        checksum ^= buffer[i + 9];
    }

    memcpy(&buffer[11], &motor4Speed, sizeof(motor4Speed));
    for (size_t i = 0; i < sizeof(motor4Speed); i++) {
        checksum ^= buffer[i + 11];
    }

    memcpy(&buffer[13], &motor5Speed, sizeof(motor5Speed));
    for (size_t i = 0; i < sizeof(motor5Speed); i++) {
        checksum ^= buffer[i + 13];
    }

    memcpy(&buffer[15], &motor6Speed, sizeof(motor6Speed));
    for (size_t i = 0; i < sizeof(motor6Speed); i++) {
        checksum ^= buffer[i + 15];
    }

    memcpy(&buffer[17], &motor7Speed, sizeof(motor7Speed));
    for (size_t i = 0; i < sizeof(motor7Speed); i++) {
        checksum ^= buffer[i + 17];
    }

    memcpy(&buffer[19], &motor8Speed, sizeof(motor8Speed));
    for (size_t i = 0; i < sizeof(motor8Speed); i++) {
        checksum ^= buffer[i + 19];
    }

    memcpy(&buffer[21], &checksum, sizeof(checksum));
	std::lock_guard<std::mutex> lock(this->cmdMutex);

	memcpy(&MSP_motor_buffer,buffer,sizeof(buffer));
	this->cmd_pending = true;
}


void State::sendMSPRequest(){
    uint8_t data_[6] = {36, 77, '<', 0, 102,0^102};
	std::lock_guard<std::mutex> lock(this->writeMutex);
    ser.write(data_,sizeof(data_));
}


void State::sendMotorMSP(const bool emergency){
	// if emergency, send idle speeds
	// otherwise, send last calculate control speed
	std::lock(this->cmdMutex, this->writeMutex);
	std::lock_guard<std::mutex> lock1(this->writeMutex, std::adopt_lock);
	std::lock_guard<std::mutex> lock2(this->cmdMutex, std::adopt_lock);

	if (emergency) {
		ROS_INFO("Sending Idle Motor Speeds at %d PWM",idle_pwm);
		ser.write(this->MSP_motor_Idle_buffer, sizeof(this->MSP_motor_Idle_buffer));}	
	else ser.write(this->MSP_motor_buffer, sizeof(this->MSP_motor_buffer));
	this->cmd_pending = false;
}

void State::processMessage(const std::vector<uint8_t>& message) {

    if (message.size() >= dataLength && message[0] == 0x24 && message[4] == 0x66) {
        std::lock_guard<std::mutex> lock1(this->dataMutex);
        memcpy(this->buffer_read, message.data(), dataLength);
        this->new_data = 1;
		
	} else {
	}
}

void State::readData()
{
	uint8_t buffer[dataLength];   
	int bytesRead = 0;
	std::vector<uint8_t> dataBuffer;
	
	int max_buffer_size = dataLength*3;
	bool endIsDel = false;
		uint8_t dataSize = this->dataLength;
	while(ros::ok()){
	if(this->ser.available()){

	bytesRead = this->ser.read(buffer, this->dataLength);
	if (bytesRead > 0){
		if (dataBuffer.size() < max_buffer_size){ 
			dataBuffer.insert(dataBuffer.end(), buffer, buffer + bytesRead);
		}
		
		std::vector<unsigned char>::iterator startPos = dataBuffer.begin();
		std::vector<unsigned char>::iterator endPos = std::search(dataBuffer.begin(), dataBuffer.end(),
                    std::vector<uint8_t>{0x24, 0x4D}.begin(), std::vector<uint8_t>{0x24, 0x4D}.end());


		if (*endPos == 0x24){ 
			endIsDel = true;}
		else 
			endIsDel = false;
		
		if (*startPos == 0x24)	
		{
		if (endIsDel){ 
			// in this case the message is contained between startPos and endPos
			// and the message is a correct message
			size_t startIndex = std::distance(dataBuffer.begin(), startPos);
    		size_t endIndex = std::distance(dataBuffer.begin(), endPos);
			size_t length = endIndex - startIndex;
			// first check message type and size
			uint8_t checksum = (dataLength-6) ^ 0x66;

			if (length == dataLength && dataBuffer[4] == 0x66)
			{
				// its an IMU message
				std::vector<uint8_t> message(startPos, endPos);
				for (int i =5; i<this->dataLength-1; i++){checksum ^= dataBuffer[i];}

				if(checksum == dataBuffer[dataLength-1]) processMessage(message);
		
		
			}
			
		} else {
		// we did not yet find the 0x24 message
		// we should doubt the data, just delete
			size_t startIndex = std::distance(dataBuffer.begin(), startPos);
    		size_t endIndex = std::distance(dataBuffer.begin(), endPos);
			size_t length = endIndex - startIndex;
			// first check message type and size
			uint8_t checksum = (dataLength-6) ^ 0x66;
			
			if (length == dataLength && dataBuffer[4] == 0x66)
			{
				// its an IMU message
				std::vector<uint8_t> message(startPos, endPos);
				for (int i =5; i<this->dataLength-1; i++){checksum ^= dataBuffer[i];}

				if(checksum == dataBuffer[dataLength-1]) processMessage(message);
		
		
			}
			
		}
			
		}
		// Remove processed data from the buffer
        dataBuffer.erase(dataBuffer.begin(), endPos);

	}
	}	
		
		
		ros::spinOnce();
        ros::Duration(0.000001).sleep();  // Adjust the sleep duration as needed
}
}

void State::IMU_callback()
{
	int16_t buffer_data;
	int32_t buffer_data_long;
    this->new_data = 0;
	std::lock_guard<std::mutex> lock(this->dataMutex);
	buffer_data = this->buffer_read[5] | (this->buffer_read[6] << 8);  
	imu_data.linear_acceleration.x = ((float) buffer_data)*(GRAVITY_MSS/512);
	buffer_data = this->buffer_read[7] | (this->buffer_read[8] << 8);  
	imu_data.linear_acceleration.y = ((float) buffer_data)*(GRAVITY_MSS/512);
	buffer_data = this->buffer_read[9] | (this->buffer_read[10] << 8);  
	imu_data.linear_acceleration.z = ((float) buffer_data)*(GRAVITY_MSS/512);
	buffer_data = this->buffer_read[11] | (this->buffer_read[12] << 8);  
	imu_data.angular_velocity.x = ((float) buffer_data)*(4/16.4)/(57.3248);
	buffer_data = this->buffer_read[13] | (this->buffer_read[14] << 8);  
	imu_data.angular_velocity.y = ((float) buffer_data)*(4/16.4)/(57.3248);
	buffer_data = this->buffer_read[15] | (this->buffer_read[16] << 8);  
	imu_data.angular_velocity.z = ((float) buffer_data)*(4/16.4)/(57.3248);
   
    buffer_data = this->buffer_read[17] | (this->buffer_read[18] << 8); 
    float roll =  (float) buffer_data*0.1;
    buffer_data = this->buffer_read[19] | (this->buffer_read[20] << 8); 
    float pitch =  (float) buffer_data*0.1;
    
    const float cosRoll = cos(roll*0.5f/(57.3248));
    const float sinRoll = sin(roll*0.5f/(57.3248));

    const float cosPitch = cos(pitch*0.5f/(57.3248));
    const float sinPitch = sin(pitch*0.5f/(57.3248));

    const float cosYaw = cos(this->yaw*0.5f);
    const float sinYaw = sin(this->yaw*0.5f);

    imu_data.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    imu_data.orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    imu_data.orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    imu_data.orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
	
    ros::Time time = ros::Time::now();
    imu_data.header.stamp.sec =  time.sec;
    imu_data.header.stamp.nsec =  time.nsec;
    
    this->imu_pub.publish(imu_data);
	this->twist_.angular.x = imu_data.angular_velocity.x*0.5 + this->twist_.angular.x*0.5;
	this->twist_.angular.y = imu_data.angular_velocity.y*0.5 + this->twist_.angular.y*0.5;
	this->twist_.angular.z = imu_data.angular_velocity.z*0.5 + this->twist_.angular.z*0.5;
	this->pose_.orientation = imu_data.orientation;

	this->state_last_time = time;

    buffer_data_long = buffer_read[23] | (buffer_read[24] << 8)
    | buffer_read[25] << 16 | (buffer_read[26] << 24);
	double lat = (float) buffer_data_long;
    buffer_data_long = buffer_read[27] | (buffer_read[28] << 8)
    | buffer_read[29] << 16 | (buffer_read[30] << 24);
	double lon = (float) buffer_data_long;
    buffer_data_long = buffer_read[31] | (buffer_read[32] << 8)
    | buffer_read[33] << 16 | (buffer_read[34] << 24);
	double baro_alt = (float) buffer_data_long;
    buffer_data = buffer_read[35] | (buffer_read[36] << 8);
	double gps_dt = (float) buffer_data;
    buffer_data_long = buffer_read[37] | (buffer_read[38] << 8)
    | buffer_read[39] << 16 | (buffer_read[40] << 24);
	uint32_t gps_seq = (uint32_t) buffer_data_long;
	
	if (gps_seq > this->gps_counter){
		this->gps_counter = gps_seq;
		this->new_gps_data = true;
		this->lattitude = lat;
		this->longitude = lon;
		this->baro_alt  = baro_alt;
		gps_data.header.stamp.sec = time.sec;
		gps_data.header.stamp.nsec = time.nsec;
		gps_data.header.seq = gps_seq;
		gps_data.header.frame_id = "GPS_BARO";
		gps_data.latitude = lat * 1e-7;
		gps_data.longitude = lon * 1e-7;
		gps_data.altitude = baro_alt * 1e-2;
		gps_data.position_covariance_type = 0;
		

		this->gps_pub.publish(gps_data);
	
	}



}

// Replacement "shutdown" XMLRPC callback
void shutdownCallback(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  emergency = true;
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
    num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_request_shutdown = 1; // Set flag
  }

  result = ros::xmlrpc::responseInt(1, "", 0);
}





// Replacement SIGINT handler
void mySigIntHandler(int sig)
{
  emergency = true;
  g_request_shutdown = 1;
}

int main(int argc, char **argv) {

	ros::init(argc, argv, "state_publisher", ros::init_options::NoSigintHandler);
	
	// Override XMLRPC shutdown
  	ros::XMLRPCManager::instance()->unbind("shutdown");
	ros::XMLRPCManager::instance()->bind("shutdown", shutdownCallback);

	int count = 0;
    State state_;
	signal(SIGINT, mySigIntHandler);

	state_.IdleCmd();

    ros::Rate loop_rate(500);

	
	
	std::thread serialReadThread(&State::readData, &state_);

	while(!g_request_shutdown) {
		ros::spinOnce();
        if (state_.new_data == 1) {state_.IMU_callback();}
        state_.publishFullPose();
		if (state_.cmd_pending) state_.sendMotorMSP(emergency); 
        else state_.sendMSPRequest();
		
		loop_rate.sleep();
		++count;
		
	}

	state_.sendMotorMSP(true);
    serialReadThread.join();
	state_.ser.close();
	
	return 0;

}

