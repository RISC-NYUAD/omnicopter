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
#include <mutex>


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

// ADD (ROS I/O):
public: ros::Publisher odometry_pub_adjusted;   // new adjusted topic
public: ros::Subscriber rtab_sub;               // subscriber to RTAB-Map odom

// ADD (RTAB cache):
public: nav_msgs::Odometry last_rtab;
public: ros::Time last_rtab_stamp;
public: bool have_rtab = false;
public: double stale_timeout = 0.75;            // seconds, reuse RTAB within this window
public: std::mutex mtx;

// ADD (optional config):
public: std::string rtab_topic = "/rtabmap/odom";
public: std::string output_topic = "/odometry_adjusted";

public: nav_msgs::Odometry last_adj;  // cache of adjusted (pos + linear)
public: bool have_adj = false;        // true after first RTAB-based adjusted publish


public: void RtabCb(const nav_msgs::Odometry::ConstPtr& msg)
{
  std::lock_guard<std::mutex> lk(this->mtx);
  this->last_rtab = *msg;
  this->last_rtab_stamp = ros::Time::now();   // arrival time
  this->have_rtab = true;
}


public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    this->model = _parent;
    if(_sdf->HasElement("nameSpace"))
      namespc  = _sdf->GetElement("nameSpace")->Get<std::string>();
			
    sequence = 0;
	
    std::string topicName = namespc + "/odometry" ;

    odometry_pub = nh.advertise<nav_msgs::Odometry>(topicName, 1); 
	
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&OdometrySensor::onUpdate, this));
		
    // Advertise adjusted odometry on namespaced topic
    std::string adjustedTopicName = this->namespc + this->output_topic;
    this->odometry_pub_adjusted = nh.advertise<nav_msgs::Odometry>(adjustedTopicName, 1);

    // Subscribe to RTAB-Map odom (absolute topic by default)
    this->rtab_sub = nh.subscribe(this->rtab_topic, 1, &OdometrySensor::RtabCb, this);

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

  // ADD (after publishing the original 'full_msg'):
  nav_msgs::Odometry adj_msg = full_msg;                 // start from the original
  adj_msg.header.stamp = full_msg.header.stamp;          // same timestamp
  adj_msg.child_frame_id = "base_link";
  // Keep orientation and angular velocity from Gazebo (already in full_msg)

  // Decide whether to use RTAB-Map data
  bool use_rtab = false;
  nav_msgs::Odometry rtab_snapshot;
  ros::Time now = curr_time;
  {
    std::lock_guard<std::mutex> lk(this->mtx);
    if (this->have_rtab && (now - this->last_rtab_stamp).toSec() <= this->stale_timeout)
    {
      rtab_snapshot = this->last_rtab;
      use_rtab = true;
    }
  }

  if (use_rtab)
  {
    // Position from RTAB -> flip x,y
    adj_msg.pose.pose.position = rtab_snapshot.pose.pose.position;
    adj_msg.pose.pose.position.x = -adj_msg.pose.pose.position.x;
    adj_msg.pose.pose.position.y = -adj_msg.pose.pose.position.y;

    const geometry_msgs::Quaternion &q = rtab_snapshot.pose.pose.orientation; // odom<-base rotation
    ignition::math::Quaterniond qOB(q.w, q.x, q.y, q.z);

    ignition::math::Vector3d vB(
        rtab_snapshot.twist.twist.linear.x,
        rtab_snapshot.twist.twist.linear.y,
        rtab_snapshot.twist.twist.linear.z);

    // body -> odom
    ignition::math::Vector3d vO = qOB.RotateVector(vB);

    // odom -> world (Rz(pi) => x=-x, y=-y)
    ignition::math::Vector3d vW(-vO.X(), -vO.Y(), vO.Z());

    adj_msg.twist.twist.linear.x = vW.X();
    adj_msg.twist.twist.linear.y = vW.Y();
    adj_msg.twist.twist.linear.z = vW.Z();

    // cache adjusted values
    this->last_adj.pose.pose.position = adj_msg.pose.pose.position;
    this->last_adj.twist.twist.linear = adj_msg.twist.twist.linear;
    this->have_adj = true;
  }

  else
  {
    // No fresh RTAB. Reuse last adjusted pos+linear if available.
    if (this->have_adj)
    {
      adj_msg.pose.pose.position = this->last_adj.pose.pose.position;
      adj_msg.twist.twist.linear = this->last_adj.twist.twist.linear;
    }
    else
    {
      // Before we ever received RTAB once: publish flipped Gazebo just so topic is valid.
      adj_msg.pose.pose.position.x = 0;
      adj_msg.pose.pose.position.y = 0;
      adj_msg.pose.pose.position.z = 0;

      adj_msg.twist.twist.linear.x = 0;
      adj_msg.twist.twist.linear.y = 0;
      adj_msg.twist.twist.linear.z = 0;
    }
  }


    // Publish the adjusted odometry
    this->odometry_pub_adjusted.publish(adj_msg);

  }



};
  GZ_REGISTER_MODEL_PLUGIN(OdometrySensor)
}

