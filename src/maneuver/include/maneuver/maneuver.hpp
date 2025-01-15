#ifndef MANEUVER_HPP
#define MANEUVER_HPP
#include <std_msgs/msg/bool.hpp>
#include "rclcpp/rclcpp.hpp"
#include "maneuver/msg/full_pose.hpp"
#include "maneuver/srv/lift_off.hpp"
#include "maneuver/srv/arm_disarm.hpp"
#include "maneuver/srv/goto_point.hpp"
#include "maneuver/srv/goto6_d_point.hpp"
#include "maneuver/srv/ellipse5_d.hpp"
#include "maneuver/srv/rotate_to.hpp"
#include "maneuver/srv/land.hpp"
#include "maneuver/srv/full_flip.hpp"
#include "libkdtp/libkdtp/libkdtp.h"

#include <sstream>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Core>

class Maneuver : public rclcpp::Node {
protected:
	// containers for published and subscribed threads
	maneuver::msg::FullPose pose_d;
	maneuver::msg::FullPose pose_;
	Eigen::Matrix3d R;

public:
        Maneuver();

	~Maneuver();

	kdtp::Robot robot;

	struct pose_sequence {
		size_t _maximum;
		size_t _length;
		std::vector<maneuver::msg::FullPose> _buffer;
	} path;

	bool lift_off_flag = false;
	double time;
	double end_time;
	int period;
	bool Armed = false;
	bool first_pose_received;
	bool log_data_flag;
	double lift_off_time = 2;
	double lift_off_height = 1;
	bool moving = false;
	size_t moving_step = 0;
	bool lift_off_callback(const std::shared_ptr<maneuver::srv::LiftOff::Request> req,
                       std::shared_ptr<maneuver::srv::LiftOff::Response> res);

	bool ArmDisarm_callback(const std::shared_ptr<maneuver::srv::ArmDisarm::Request> req,
                         std::shared_ptr<maneuver::srv::ArmDisarm::Response> res);

	bool goto_callback(const std::shared_ptr<maneuver::srv::GotoPoint::Request> req,
                   std::shared_ptr<maneuver::srv::GotoPoint::Response> res);

	bool goto_6D_callback(const std::shared_ptr<maneuver::srv::Goto6DPoint::Request> req,
                      std::shared_ptr<maneuver::srv::Goto6DPoint::Response> res);

	bool goto_5DEllipse_callback(const std::shared_ptr<maneuver::srv::Ellipse5D::Request> req,
                              std::shared_ptr<maneuver::srv::Ellipse5D::Response> res);

	bool rotateTo_callback(const std::shared_ptr<maneuver::srv::RotateTo::Request> req,
                        std::shared_ptr<maneuver::srv::RotateTo::Response> res);

	bool land_callback(const std::shared_ptr<maneuver::srv::Land::Request> req,
                   std::shared_ptr<maneuver::srv::Land::Response> res);

	bool full_flip_callback(const std::shared_ptr<maneuver::srv::FullFlip::Request> req,
                        std::shared_ptr<maneuver::srv::FullFlip::Response> res);

    	rclcpp::Service<maneuver::srv::LiftOff>::SharedPtr lift_off_service_;
    	rclcpp::Service<maneuver::srv::ArmDisarm>::SharedPtr arm_disarm_service_;
    	rclcpp::Service<maneuver::srv::GotoPoint>::SharedPtr goto_service_;
    	rclcpp::Service<maneuver::srv::Goto6DPoint>::SharedPtr goto_6D_service_;
    	rclcpp::Service<maneuver::srv::Ellipse5D>::SharedPtr ellipse_5D_service_;
    	rclcpp::Service<maneuver::srv::RotateTo>::SharedPtr rotate_to_service_;
    	rclcpp::Service<maneuver::srv::Land>::SharedPtr land_service_;
        rclcpp::Service<maneuver::srv::FullFlip>::SharedPtr full_flip_service_;

	void publishDesired();

private:
    void timer_callback();
    rclcpp::Publisher<maneuver::msg::FullPose>::SharedPtr pose_d_pub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr toggle_pub;
    rclcpp::Subscription<maneuver::msg::FullPose>::SharedPtr pose_sub;
    rclcpp::TimerBase::SharedPtr timer_;
    void poseCallback(const std::shared_ptr<maneuver::msg::FullPose> _pose);
};

#endif
