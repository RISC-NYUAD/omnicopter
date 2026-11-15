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
#include <std_msgs/Float64MultiArray.h>
#include <limits>
#include <cmath>
#include <mutex>
#include <vector>
#include <algorithm>
#include "thrust_dynamics.hpp"
#include <stdexcept>
#include <functional>

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
public: ros::Subscriber cmd_sub, limits_sub;
public: std::vector<double> thrust_upper;
private: std::vector<double> actual_thrust;
private: double tau_up=0.15, tau_down=0.21;
private: common::Time last_update, last_publication;
private: ros::Publisher actual_thrust_pub;
private: std::mutex command_mutex;
public: void limits_callback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if(msg->data.size()!=2*static_cast<size_t>(c_vars.num_of_props)) return;
    for(int i=0;i<c_vars.num_of_props;i++) {
      const double lo=msg->data[i], hi=msg->data[c_vars.num_of_props+i];
      if(!std::isfinite(lo) || !std::isfinite(hi) || lo<0 || hi<lo) return;
    }
    std::lock_guard<std::mutex> guard(command_mutex);
    for(int i=0;i<c_vars.num_of_props;i++) thrust_upper[i]=msg->data[c_vars.num_of_props+i];
}
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
        thrust_upper.assign(c_vars.num_of_props,std::numeric_limits<double>::infinity());
        actual_thrust.assign(c_vars.num_of_props,0.0);
        nh.param("/omnicopter_sim/thrust_tau_up",tau_up,0.15);
        nh.param("/omnicopter_sim/thrust_tau_down",tau_down,0.21);
        if(!std::isfinite(tau_up) || !std::isfinite(tau_down) || tau_up<=0 || tau_down<=0)
          throw std::invalid_argument("Thrust time constants must be positive finite seconds");
        if(!std::isfinite(c_vars.thr_const) || c_vars.thr_const<=0 ||
           !std::isfinite(c_vars.slow_down) || c_vars.slow_down<=0)
          throw std::invalid_argument("Invalid thrust constant or visual slowdown");
        actual_thrust_pub=nh.advertise<std_msgs::Float64MultiArray>("/omnicopter/motor_thrust_actual",1);
        last_update=this->model->GetWorld()->SimTime();
        last_publication=last_update;
        ROS_INFO("First-order thrust dynamics: acceleration %.3f s, deceleration %.3f s (zero braking)",tau_up,tau_down);
        bool direct=false;
        nh.param("/omnicopter_sim/direct_thrust",direct,false);
        if(direct) {
          // Until the latched limits arrive, direct mode starts with motors off.
          thrust_upper.assign(c_vars.num_of_props,0.0);
          limits_sub=nh.subscribe("/omnicopter/motor_limits",1,&BetaMotorPlugin::limits_callback,this);
        } 


		
	  	this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&BetaMotorPlugin::onUpdate, this, std::placeholders::_1));
		
	}  

public: void onUpdate(const common::UpdateInfo& info)
	{
        const double dt=(info.simTime-last_update).Double();
        last_update=info.simTime;
        if(dt<0) {
          // Reset physical state on a world-time rewind; inputs remain latched.
          std::fill(actual_thrust.begin(),actual_thrust.end(),0.0);
          last_publication=info.simTime;
        }
        Eigen::VectorXd command; std::vector<double> caps;
        {std::lock_guard<std::mutex> guard(command_mutex); command=latest_msg; caps=thrust_upper;}
		std::string preface = c_vars.namespc + "::" + "rotor_" ;
		std::string postface = "_joint" ;
		physics::JointPtr motor_joint;
		physics::LinkPtr motor_link;
//		std::cout << c_vars.num_of_props << std::endl;
//		std::cout << preface + std::to_string(0) + postface << std::endl;
		for(int i = 0 ; i < c_vars.num_of_props ; i++){
            const double raw_target=std::pow(command(i)*RPM2RPS,2)*c_vars.thr_const;
            // Limit the TARGET, not the physical state: residual thrust can
            // exceed a newly reduced command limit during passive spin-down.
            const double target=std::min(raw_target,caps[i]);
            actual_thrust[i]=omni_sim::advanceThrust(actual_thrust[i],target,
                std::max(0.0,dt),tau_up,tau_down);
            const double actual_speed=std::sqrt(actual_thrust[i]/c_vars.thr_const);
			//RPM VISUAL
			motor_joint = this->model->GetJoint(preface + std::to_string(i) + postface);
			motor_joint->SetVelocity(0, c_vars.turning_dir(i) * (actual_speed/c_vars.slow_down) );
			//THRUST FORCE
			motor_link = this->model->GetLink(preface + std::to_string(i));
            const double force=actual_thrust[i];
			double torque = force * c_vars.thr_to_drag ;
			motor_link->AddRelativeForce(ignition::math::Vector3d (0, 0, force));
			//DRAG TORQUE
			physics::Link_V parent_links = motor_link->GetParentJointsLinks();
			auto parent_rotation=parent_links.at(0)->WorldCoGPose().Rot();
            auto motor_rotation=motor_link->WorldCoGPose().Rot();
			ignition::math::Vector3d drag_torque(0, 0, -c_vars.turning_dir(i) * torque );
			ignition::math::Vector3d drag_torque_parent_frame = parent_rotation.RotateVectorReverse(motor_rotation.RotateVector(drag_torque));
			parent_links.at(0)->AddRelativeTorque(drag_torque_parent_frame);

		}
        if((info.simTime-last_publication).Double()>=0.004 || dt<0) {
          std_msgs::Float64MultiArray measured;
          measured.data=actual_thrust;
          actual_thrust_pub.publish(measured);
          last_publication=info.simTime;
        }
	}

public: void cmd_callback(const mav_msgs::Actuators msg)
	{
        if(msg.angular_velocities.size()!=static_cast<size_t>(c_vars.num_of_props)) return;
        for(double v:msg.angular_velocities) if(!std::isfinite(v)) return;
        std::lock_guard<std::mutex> guard(command_mutex);
		for(int i = 0 ; i < c_vars.num_of_props ; i++){
			double vel = msg.angular_velocities[i];
			vel = std::max(vel, 0.0);
			vel = std::min(vel, 1.0);
			vel *= c_vars.norm_to_rpm ;
            // Callback stores the target only. Dynamics run at physics time.
            latest_msg(i) = vel;
		}
	}

};
GZ_REGISTER_MODEL_PLUGIN(BetaMotorPlugin)
}

