#include "maneuver/maneuver.hpp"
#include "helpers.cc"
#include "maneuver_codes.cc"


Maneuver::Maneuver() : Node("maneuver"), robot("rotorcraft") {
    pose_d_pub = this->create_publisher<maneuver::msg::FullPose>("pose_d", 1000);   
    toggle_pub = this->create_publisher<std_msgs::msg::Bool>("/toggle_topic", 1000);
    this->path._length = 0;
    this->path._maximum = 15000;
    mv_plan_start(this);
    pose_sub = this->create_subscription<maneuver::msg::FullPose>("pose_full", 10, std::bind(&Maneuver::poseCallback,this,std::placeholders::_1));
    timer_ = this->create_wall_timer(
        std::chrono::seconds(2),
        std::bind(&Maneuver::timer_callback, this));
    this->first_pose_received = false;
    lift_off_service_ = this->create_service<maneuver::srv::LiftOff>(
    "lift_off",
    std::bind(&Maneuver::lift_off_callback, this, std::placeholders::_1, std::placeholders::_2));

    arm_disarm_service_ = this->create_service<maneuver::srv::ArmDisarm>(
    "arm_disarm",
    std::bind(&Maneuver::ArmDisarm_callback, this, std::placeholders::_1, std::placeholders::_2));

    goto_service_ = this->create_service<maneuver::srv::GotoPoint>(
    "goto_point",
    std::bind(&Maneuver::goto_callback, this, std::placeholders::_1, std::placeholders::_2));

    goto_6D_service_ = this->create_service<maneuver::srv::Goto6DPoint>(
    "goto_6d_point",
    std::bind(&Maneuver::goto_6D_callback, this, std::placeholders::_1, std::placeholders::_2));

    ellipse_5D_service_ = this->create_service<maneuver::srv::Ellipse5D>(
    "goto_5d_ellipse",
    std::bind(&Maneuver::goto_5DEllipse_callback, this, std::placeholders::_1, std::placeholders::_2));

    rotate_to_service_ = this->create_service<maneuver::srv::RotateTo>(
    "rotate_to",
    std::bind(&Maneuver::rotateTo_callback, this, std::placeholders::_1, std::placeholders::_2));

    land_service_ = this->create_service<maneuver::srv::Land>(
    "land",
    std::bind(&Maneuver::land_callback, this, std::placeholders::_1, std::placeholders::_2));

    full_flip_service_ = this->create_service<maneuver::srv::FullFlip>(
    "full_flip",
    std::bind(&Maneuver::full_flip_callback, this, std::placeholders::_1, std::placeholders::_2));
}

bool Maneuver::lift_off_callback(const std::shared_ptr<maneuver::srv::LiftOff::Request> req,
                                 std::shared_ptr<maneuver::srv::LiftOff::Response> res) {
    double height = req->height;
    double duration = req->duration;
    if (this->lift_off_flag || !Armed) {
        res->status = false;
        return false;
    } else {
        end_time = this->now().seconds() + this->lift_off_time;

        bool status = mv_plan_take_off(this, this->pose_, height, duration, &this->path);
        res->status = status;
        if (status) {
            this->lift_off_flag = true;
            this->moving = true;
            this->moving_step = 0;
            return true;
        } else {
            return false;
        }
    }
}

bool Maneuver::land_callback(const std::shared_ptr<maneuver::srv::Land::Request> req,
                             std::shared_ptr<maneuver::srv::Land::Response> res) {
    double height_1 = req->height_1;
    double duration_1 = req->duration_1;
    double height_2 = req->height_2;
    double duration_2 = req->duration_2;
    if (!this->lift_off_flag || !Armed) {
        res->status = false;
        return false;
    } else {
        end_time = this->now().seconds() + this->lift_off_time;

        bool status = mv_plan_land(this, this->pose_, height_1, duration_1, height_2, duration_2, &this->path, this->moving);
        res->status = status;
        if (status) {
            if (!this->moving) {
                this->moving = true;
                this->moving_step = 0;
            }
            return true;
        } else {
            return false;
        }
    }
}

bool Maneuver::full_flip_callback(const std::shared_ptr<maneuver::srv::FullFlip::Request> req,
                                  std::shared_ptr<maneuver::srv::FullFlip::Response> res) {
    bool roll_bool = req->roll_bool;
    bool pitch_bool = req->pitch_bool;
    double duration = req->duration;
    if (!this->lift_off_flag || !Armed) {
        res->status = false;
        return false;
    } else {
        end_time = this->now().seconds() + this->lift_off_time;

        bool status = mv_plan_flip(this, this->pose_, roll_bool, pitch_bool, duration, &this->path);
        res->status = status;
        if (status) {
            if (!this->moving) {
                this->moving = true;
                this->moving_step = 0;
            }
            return true;
        } else {
            return false;
        }
    }
}

bool Maneuver::ArmDisarm_callback(const std::shared_ptr<maneuver::srv::ArmDisarm::Request> req,
                                  std::shared_ptr<maneuver::srv::ArmDisarm::Response> res) {
    (void) req;
    Armed = !Armed;
    RCLCPP_ERROR(this->get_logger(), "Armed Maneuver %s", Armed ? "true" : "false");
    if (!Armed) {
        this->lift_off_flag = false;
    }
    std_msgs::msg::Bool msg;
    msg.data = Armed;
    this->toggle_pub->publish(msg); // ROS2-compliant publish
    res->success = true;
    return true;
}

bool Maneuver::goto_callback(const std::shared_ptr<maneuver::srv::GotoPoint::Request> req,
                             std::shared_ptr<maneuver::srv::GotoPoint::Response> res) {
    double x = req->x;
    double y = req->y;
    double z = req->z;
    double w = req->yaw;
    double duration = req->duration;
    if (!this->lift_off_flag) {
        res->status = false;
        return false;
    } else {
        end_time = this->now().seconds() + this->lift_off_time;

        bool status = mv_goto(this, this->pose_, x, y, z, w, duration, &this->path, this->moving);
        res->status = status;
        if (status) {
            if (!this->moving) {
                this->moving = true;
                this->moving_step = 0;
            }
            return true;
        } else {
            return false;
        }
    }
}

bool Maneuver::goto_6D_callback(const std::shared_ptr<maneuver::srv::Goto6DPoint::Request> req,
                                std::shared_ptr<maneuver::srv::Goto6DPoint::Response> res) {
    double x = req->x;
    double y = req->y;
    double z = req->z;
    double roll = req->roll;
    double pitch = req->pitch;
    double yaw = req->yaw;
    double duration = req->duration;
    if (!this->lift_off_flag) {
        res->status = false;
        return false;
    } else {
        end_time = this->now().seconds() + this->lift_off_time;

        bool status = mv_goto_6D(this, this->pose_, x, y, z, roll, pitch, yaw, duration, &this->path, this->moving);
        res->status = status;
        if (status) {
            if (!this->moving) {
                this->moving = true;
                this->moving_step = 0;
            }
            return true;
        } else {
            return false;
        }
    }
}

bool Maneuver::goto_5DEllipse_callback(const std::shared_ptr<maneuver::srv::Ellipse5D::Request> req,
                                       std::shared_ptr<maneuver::srv::Ellipse5D::Response> res) {
    double x_min = req->x_min;
    double x_max = req->x_max;
    double y_min = req->y_min;
    double y_max = req->y_max;
    double z_min = req->z_min;
    double z_max = req->z_max;
    double roll_start = req->roll_start;
    double roll_mid = req->roll_mid;
    double roll_end = req->roll_end;
    double pitch_start = req->pitch_start;
    double pitch_mid = req->pitch_mid;
    double pitch_end = req->pitch_end;
    double yaw = req->yaw;
    double duration = req->duration;
    if (!this->lift_off_flag || !Armed) {
        res->status = false;
        return false;
    } else {
        end_time = this->now().seconds() + this->lift_off_time;

        bool status = mv_plan_5DEllipse(this, this->pose_,
                                       x_min, x_max, y_min, y_max, z_min, z_max,
                                       roll_start, roll_mid, roll_end,
                                       pitch_start, pitch_mid, pitch_end,
                                       yaw, duration, &this->path, this->moving);
        res->status = status;
        if (status) {
            if (!this->moving) {
                this->moving = true;
                this->moving_step = 0;
            }
            return true;
        } else {
            return false;
        }
    }
}

bool Maneuver::rotateTo_callback(const std::shared_ptr<maneuver::srv::RotateTo::Request> req,
                                 std::shared_ptr<maneuver::srv::RotateTo::Response> res) {
    double roll = req->roll;
    double pitch = req->pitch;
    double yaw = req->yaw;
    double duration = req->duration;
    if (!this->lift_off_flag) {
        res->status = false;
        return false;
    } else {
        end_time = this->now().seconds() + this->lift_off_time;

        bool status = mv_rotateTo(this, this->pose_, roll, pitch, yaw, duration, &this->path, this->moving);
        res->status = status;
        if (status) {
            if (!this->moving) {
                this->moving = true;
                this->moving_step = 0;
            }
            return true;
        } else {
            return false;
        }
    }
}


Maneuver::~Maneuver() {
}

void Maneuver::timer_callback() {
    //RCLCPP_INFO(this->get_logger(), "Published: %f", this->pose_.pose.position.x);
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
	this->pose_d_pub->publish(this->pose_d);
}

void Maneuver::poseCallback(const std::shared_ptr<maneuver::msg::FullPose> _pose) {
    this->pose_ = *_pose;
	Eigen::Quaternion<double> q;
	q.coeffs() << this->pose_.pose.orientation.x, this->pose_.pose.orientation.y, this->pose_.pose.orientation.z, this->pose_.pose.orientation.w;
	this->R = q.toRotationMatrix();
	Eigen::Vector3d gravity;
	gravity << 0., 0., 9.81;
	gravity = this->R * gravity;
	this->pose_.acc.linear.x = this->pose_.acc.linear.x + gravity(0);
	this->pose_.acc.linear.y = this->pose_.acc.linear.y + gravity(1);
	this->pose_.acc.linear.z = this->pose_.acc.linear.z + gravity(2);
	this->first_pose_received = true;
}

int main(int argc, char **argv) {

    rclcpp::init(argc, argv);
    auto node = std::make_shared<Maneuver>();

    //Maneuver maneuver_;
    //maneuver_.period = 100; // Period in milliseconds
    //maneuver_.log_data_flag = false;
    node->period = 100;
    node->log_data_flag = false;
    rclcpp::Rate loop_rate(node->period); // Convert period to Hz

    int count = 0;

    while (rclcpp::ok()) {
        rclcpp::spin_some(node); // Process callbacks
        //node->time = node->now().seconds(); // Equivalent to ros::Time::now().toSec()

        if (node->lift_off_flag) {
            node->publishDesired();
        }

        loop_rate.sleep();
        ++count;
    }

    rclcpp::shutdown();
    return 0;
}
