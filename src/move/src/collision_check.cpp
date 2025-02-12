#include <ros/ros.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_check_node");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // Load the robot model
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    if (!robot_model)
    {
        ROS_ERROR("Failed to load robot model.");
        return 1;
    }

    // Create a PlanningScene
    planning_scene::PlanningScene planning_scene(robot_model);

    // Get the current robot state
    robot_state::RobotState& robot_state = planning_scene.getCurrentStateNonConst();
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("drone_base");

    if (!joint_model_group)
    {
        ROS_ERROR("Joint group 'drone_base' not found in the robot model.");
        return 1;
    }

    // Hardcoded position and orientation in quaternion for the base link
    std::vector<double> joint_positions = {
        0.5, // x position
        0.5, // y position
        0.2, // z position
        0.0, // qx (quaternion x)
        0.0, // qy (quaternion y)
        0.0, // qz (quaternion z)
        1.0  // qw (quaternion w)
    };

    robot_state.setJointGroupPositions(joint_model_group, joint_positions);
    planning_scene.setCurrentState(robot_state);

    // Perform collision checking
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;   // Enable contact reporting
    collision_request.max_contacts = 10; // Limit the number of contacts
    collision_request.verbose = true;    // Enable detailed output

    collision_detection::CollisionResult collision_result;
    planning_scene.checkCollision(collision_request, collision_result);

    if (collision_result.collision)
    {
        ROS_WARN("Collision detected!");
        for (const auto& contact : collision_result.contacts)
        {
            ROS_WARN_STREAM("Contact between: " << contact.first.first << " and " << contact.first.second);
        }
    }
    else
    {
        ROS_INFO("No collision detected.");
    }

    ros::shutdown();
    return 0;
}
