#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math.hh>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <string>
#include <time.h>


namespace gazebo
{
class OdometrySensor : public ModelPlugin
{

public: ros::NodeHandle nh;
public: ros::Publisher odometry_pub;
public: std::string namespc;
public: double time;
public: uint32_t sequence;
private: physics::ModelPtr model;
private: event::ConnectionPtr updateConnection;

public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->model = _parent;
    if(_sdf->HasElement("nameSpace"))
      namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();
			
    sequence = 0;
	
    std::string topicName = namespc + "/odometry" ;

    odometry_pub = nh.advertise<nav_msgs::Odometry>(topicName, 1); 
	
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OdometrySensor::onUpdate, this));
		

  }  

public:	void onUpdate()
  {
		
    // Get simulation time and initialize sensor message header //
    sequence++;
    ros::Time curr_time = ros::Time::now();
		
    std_msgs::Header header;	
    header.seq = sequence;
    header.frame_id = "world" ;
    header.stamp = curr_time ;
		
    nav_msgs::Odometry full_msg;
    geometry_msgs::Pose p_msg;
    full_msg.header = header;

    std::string linkName = namespc+"::"+"base_link";
    physics::LinkPtr  quad = this->model->GetLink(linkName);

    ignition::math::Pose3d pose = quad->WorldCoGPose();

    ignition::math::Vector3d position = pose.Pos();
    ignition::math::Quaternion orientation = pose.Rot();

    ignition::math::Vector3d ang_vel = quad->RelativeAngularVel();
    ignition::math::Vector3d lin_vel = quad->WorldLinearVel();
    
    p_msg.position.x = position.X();
    p_msg.position.y = position.Y();
    p_msg.position.z = position.Z();
    p_msg.orientation.x = orientation.X();
    p_msg.orientation.y = orientation.Y();
    p_msg.orientation.z = orientation.Z();
    p_msg.orientation.w = orientation.W();

	geometry_msgs::Twist t_msg;
	t_msg.linear.x = lin_vel.X();
	t_msg.linear.y = lin_vel.Y();
	t_msg.linear.z = lin_vel.Z();
	t_msg.angular.x = ang_vel.X();
	t_msg.angular.y = ang_vel.Y();
	t_msg.angular.z = ang_vel.Z();

	full_msg.pose.pose = p_msg;
	full_msg.twist.twist = t_msg;

    odometry_pub.publish(full_msg);
  }



};
  GZ_REGISTER_MODEL_PLUGIN(OdometrySensor)
}

