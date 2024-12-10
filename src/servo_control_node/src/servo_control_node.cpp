#include <ros/ros.h>
#include <gpiod.h>
#include <servo_control_node/SetAngles.h>  // Include the generated service header
#include <servo_control_node/ReleaseArms.h>
#include <std_msgs/Int32.h>  // Include standard message type for publishing angles
#include <iostream>
#include <chrono>
#include <thread>

// Create global variables for the angles
int currentRightAngle = 0;
int currentLeftAngle = 0;

// Create pointers for GPIO lines
gpiod_line* line1 = nullptr;
gpiod_line* line2 = nullptr;

// Publisher for the angles
ros::Publisher right_angle_pub;
ros::Publisher left_angle_pub;

// Function to map angle (0 to 180) to PWM duty cycle (10% to 90%)
int map_degree_to_pwm_duty(int degree) {
    if (degree < 0) degree = 0;
    if (degree > 115) degree = 115;
    return (degree * (90 - 10) / 180) + 10;
}

// Function to generate PWM signal
void set_pwm(gpiod_line* line, int dutyCycle) {
    const int period = 10000;  // 20ms period for standard servos (50Hz)
    int highTime = period * dutyCycle / 100;
    int lowTime = period - highTime;
    gpiod_line_set_value(line, 1);  
    std::this_thread::sleep_for(std::chrono::microseconds(highTime));
    gpiod_line_set_value(line, 0);  
    std::this_thread::sleep_for(std::chrono::microseconds(lowTime));
}

// Service callback to release both servos and set angles to 0
bool release_arms(servo_control_node::ReleaseArms::Request& req, servo_control_node::ReleaseArms::Response& res) {
    // Set both angles to 0 for releasing the arms
    currentRightAngle = 0;
    currentLeftAngle = 0;

    // Map the 0 degrees to PWM duty cycle (this will be the neutral position)
    int pwmDutyRight = map_degree_to_pwm_duty(currentRightAngle);
    int pwmDutyLeft = map_degree_to_pwm_duty(currentLeftAngle);

    // Publish the updated angles (both should be 0)
    std_msgs::Int32 right_angle_msg;
    std_msgs::Int32 left_angle_msg;

    right_angle_msg.data = currentRightAngle;
    left_angle_msg.data = currentLeftAngle;

    right_angle_pub.publish(right_angle_msg);
    left_angle_pub.publish(left_angle_msg);

    // Set PWM for the servos to 0 for 2 seconds
    ros::Time start_time = ros::Time::now();
    ros::Duration duration(2.0); // 2 seconds duration to hold the position

    while (ros::Time::now() - start_time < duration) {
        // Continuously send PWM signal during the 2 seconds
        set_pwm(line1, pwmDutyRight);  // Send PWM to right motor
        set_pwm(line2, pwmDutyLeft);   // Send PWM to left motor
        ros::spinOnce();  // Allow ROS to process incoming messages
    }

    // After 2 seconds, stop sending PWM by setting the lines to LOW
    gpiod_line_set_value(line1, 0);
    gpiod_line_set_value(line2, 0);

    return true;
}


// Service callback to set angles for 2 seconds
bool set_angles(servo_control_node::SetAngles::Request& req, servo_control_node::SetAngles::Response& res) {
    // Update the global angle variables with the new values from the service request
    currentRightAngle = req.right;
    currentLeftAngle = req.left;

    int pwmDutyRight = map_degree_to_pwm_duty(currentRightAngle);
    int pwmDutyLeft = map_degree_to_pwm_duty(currentLeftAngle);

    // Publish the updated angles
    std_msgs::Int32 right_angle_msg;
    std_msgs::Int32 left_angle_msg;

    right_angle_msg.data = currentRightAngle;
    left_angle_msg.data = currentLeftAngle;

    right_angle_pub.publish(right_angle_msg);
    left_angle_pub.publish(left_angle_msg);

    // Set PWM for the servos for 2 seconds
    ros::Time start_time = ros::Time::now();
    ros::Duration duration(2.0); // 2 seconds duration

    while (ros::Time::now() - start_time < duration) {
        // Continuously send PWM signal during the 2 seconds
        set_pwm(line1, pwmDutyRight);  // Send PWM to right motor
        set_pwm(line2, pwmDutyLeft);   // Send PWM to left motor
        ros::spinOnce();  // Allow ROS to process incoming messages
    }

    // After 2 seconds, stop sending PWM by setting the lines to LOW
    gpiod_line_set_value(line1, 0);
    gpiod_line_set_value(line2, 0);

    res.success = true;
    return true;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "servo_control_node");
    ros::NodeHandle nh;

    // Load parameters from the parameter server (from the YAML file)
    int gpio_pin_right, gpio_pin_left;
    nh.param("servo_control/gpio_pin_right", gpio_pin_right, 12);  // Default to 12 if not set
    nh.param("servo_control/gpio_pin_left", gpio_pin_left, 13);    // Default to 13 if not set

    // Open the GPIO chip
    struct gpiod_chip *chip = gpiod_chip_open("/dev/gpiochip0");
    if (!chip) {
        std::cerr << "Failed to open GPIO chip\n";
        return 1;
    }

    // Get the lines for the servos (based on the parameters)
    line1 = gpiod_chip_get_line(chip, gpio_pin_right);
    line2 = gpiod_chip_get_line(chip, gpio_pin_left);
    if (!line1 || !line2) {
        std::cerr << "Failed to get lines\n";
        gpiod_chip_close(chip);
        return 1;
    }

    // Request lines as output
    gpiod_line_request_output(line1, "Motor1_PWM", 0);
    gpiod_line_request_output(line2, "Motor2_PWM", 0);

    // Advertise the service
    ros::ServiceServer service = nh.advertiseService("set_angles", set_angles);
    ros::ServiceServer service2 = nh.advertiseService("release_arms", release_arms);

    // Initialize publishers for angles
    right_angle_pub = nh.advertise<std_msgs::Int32>("right_angle", 10);
    left_angle_pub = nh.advertise<std_msgs::Int32>("left_angle", 10);

    // Initial publication of angles before any service calls
    std_msgs::Int32 right_angle_msg;
    std_msgs::Int32 left_angle_msg;

    right_angle_msg.data = currentRightAngle;
    left_angle_msg.data = currentLeftAngle;

    right_angle_pub.publish(right_angle_msg);
    left_angle_pub.publish(left_angle_msg);

    // Continuously publish the angles (even after service calls)
    ros::Rate loop_rate(10);  // 10 Hz to continuously update the angle topics
    while (ros::ok()) {
        // Publish the current angles to the topics
        right_angle_msg.data = currentRightAngle;
        left_angle_msg.data = currentLeftAngle;

        right_angle_pub.publish(right_angle_msg);
        left_angle_pub.publish(left_angle_msg);

        ros::spinOnce();  // Allow ROS to process incoming messages
        loop_rate.sleep();  // Sleep for the next cycle
    }

    // Release resources and close the GPIO chip
    gpiod_chip_close(chip);
    return 0;
}

