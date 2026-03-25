#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <mav_msgs/Actuators.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>
#include <string>
#include <time.h>
#include <sstream>
#include <Eigen/Dense>

#define RPM2RPS 0.10472

struct MotorControls {
	std::string namespc;
	int num_of_props;
	double slow_down;
	double norm_to_rpm;
	double thr_const;
	double thr_to_drag;
	std::string directions_string;
	Eigen::VectorXd turning_dir;
	MotorControls() : namespc("m100"), num_of_props(4), slow_down(10), norm_to_rpm(500), thr_const(0.0027), thr_to_drag(0.017),
	directions_string("1 0 1 0"), turning_dir(Eigen::VectorXd::Zero(4)) { } 	
};

namespace gazebo
{
class BetaMotorPlugin : public ModelPlugin
{

public: ros::NodeHandle nh;
public: MotorControls c_vars;
public: Eigen::VectorXd latest_msg;
public: ros::Subscriber cmd_sub;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  	this->model = _parent;
		if(_sdf->HasElement("nameSpace"))
			c_vars.namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();
		if(_sdf->HasElement("prop_num"))
			c_vars.num_of_props = _sdf->Get<int>("prop_num");		
		if(_sdf->HasElement("slowdown_sim"))
			c_vars.slow_down = _sdf->Get<double>("slowdown_sim");
		if(_sdf->HasElement("throttle"))
			c_vars.norm_to_rpm = _sdf->Get<double>("throttle");
		if(_sdf->HasElement("thrust_const"))
			c_vars.thr_const = _sdf->Get<double>("thrust_const");
		if(_sdf->HasElement("drag_const"))
			c_vars.thr_to_drag = _sdf->Get<double>("drag_const");	
		if(_sdf->HasElement("turning_directions"))
			c_vars.directions_string = _sdf->GetElement("turning_directions")->Get<std::string>();
						
		const char space_delimiter = ' ';
    		std::vector<std::string> words{};
    		
		std::stringstream ss(c_vars.directions_string);
		std::string temp_str;
		while( std::getline(ss, temp_str, space_delimiter) ){
			words.push_back(temp_str);
		}			

		c_vars.turning_dir = Eigen::VectorXd::Zero(c_vars.num_of_props);		
		for(int i = 0 ; i < c_vars.num_of_props ; i++){
			int val = std::stoi(words.at(i));		
			c_vars.turning_dir(i) = (val>0) ? 1 : -1 ;
		}
		
		latest_msg = Eigen::VectorXd::Zero(c_vars.num_of_props);
		std::string topicName = c_vars.namespc + "/command_motor_speed" ;

		cmd_sub = nh.subscribe(topicName, 1, &BetaMotorPlugin::cmd_callback, this); 


		
	  	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BetaMotorPlugin::onUpdate, this));
		
	}  

public: void onUpdate()
	{
		std::string preface = c_vars.namespc + "::" + "rotor_" ;
		std::string postface = "_joint" ;
		physics::JointPtr motor_joint;
		physics::LinkPtr motor_link;
//		std::cout << c_vars.num_of_props << std::endl;
//		std::cout << preface + std::to_string(0) + postface << std::endl;
		for(int i = 0 ; i < c_vars.num_of_props ; i++){
			//RPM VISUAL
			motor_joint = this->model->GetJoint(preface + std::to_string(i) + postface);
			motor_joint->SetVelocity(0, c_vars.turning_dir(i) * (latest_msg(i) * RPM2RPS/c_vars.slow_down) );
			//THRUST FORCE
			motor_link = this->model->GetLink(preface + std::to_string(i));
			double force = pow((latest_msg(i) * RPM2RPS),2) * c_vars.thr_const ;
			double torque = force * c_vars.thr_to_drag ;
			motor_link->AddRelativeForce(ignition::math::Vector3d (0, 0, force));
			//DRAG TORQUE
			physics::Link_V parent_links = motor_link->GetParentJointsLinks();
			ignition::math::Pose3d pose_difference = motor_link->WorldCoGPose() - parent_links.at(0)->WorldCoGPose();
			ignition::math::Vector3d drag_torque(0, 0, -c_vars.turning_dir(i) * torque );
			ignition::math::Vector3d drag_torque_parent_frame = pose_difference.Rot().RotateVector(drag_torque);
			parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

		}
		

	}

public: void cmd_callback(const mav_msgs::Actuators msg)
	{
		for(int i = 0 ; i < c_vars.num_of_props ; i++){
			double vel = msg.angular_velocities[i];
			vel = std::max(vel, 0.0);
			vel = std::min(vel, 1.0);
			vel *= c_vars.norm_to_rpm ;
			if(vel >= latest_msg(i)){
				latest_msg(i) = 0.6*latest_msg(i) + 0.4*vel;
			}else{
				latest_msg(i) = 0.6*latest_msg(i) + 0.4*vel;
			}
		}
	}

};
GZ_REGISTER_MODEL_PLUGIN(BetaMotorPlugin)
}

