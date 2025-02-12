#include <ros/ros.h>
#include <ros/package.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>  // Include the RRTConnect header
#include <ompl/geometric/planners/rrt/RRTstar.h>  // Include the RRTConnect header
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>  // Include PathLengthOptimizationObjective
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/collision_detection/collision_tools.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <vector>
#include <limits>
#include <cmath>
#include <iostream>

namespace ob = ompl::base;
namespace og = ompl::geometric;

// Global PlanningScene pointer for collision checking
planning_scene::PlanningScenePtr planning_scene_instance;

bool directional = true;
// Axis-Aligned Bounding Box structure
struct AABB {
    Eigen::Vector3f min_bounds;  // Minimum x, y, z
    Eigen::Vector3f max_bounds;  // Maximum x, y, z
    int triangle_index;          // Index of the triangle in the mesh
};


std::vector<AABB> precomputeAABBs(const pcl::PolygonMesh& mesh) {
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);

    std::vector<AABB> aabbs;
    for (size_t i = 0; i < mesh.polygons.size(); ++i) {
        const auto& triangle = mesh.polygons[i];

        // Extract vertices of the triangle
        Eigen::Vector3f v0(vertices.points[triangle.vertices[0]].x,
                           vertices.points[triangle.vertices[0]].y,
                           vertices.points[triangle.vertices[0]].z);
        Eigen::Vector3f v1(vertices.points[triangle.vertices[1]].x,
                           vertices.points[triangle.vertices[1]].y,
                           vertices.points[triangle.vertices[1]].z);
        Eigen::Vector3f v2(vertices.points[triangle.vertices[2]].x,
                           vertices.points[triangle.vertices[2]].y,
                           vertices.points[triangle.vertices[2]].z);

        // Compute bounding box for the triangle
        AABB box;
        box.min_bounds = v0.cwiseMin(v1).cwiseMin(v2);
        box.max_bounds = v0.cwiseMax(v1).cwiseMax(v2);
        box.triangle_index = i;

        aabbs.push_back(box);
    }
    return aabbs;
}

bool isPointNearAABB(const Eigen::Vector3f& point, const AABB& box, float max_distance, Eigen::Vector3f& closest_point) {
    closest_point = point.cwiseMax(box.min_bounds).cwiseMin(box.max_bounds);
    float dist_squared = (point - closest_point).squaredNorm();
    return dist_squared <= max_distance * max_distance;
}

float pointToTriangleDistance(const Eigen::Vector3f& point,
                              const Eigen::Vector3f& v0,
                              const Eigen::Vector3f& v1,
                              const Eigen::Vector3f& v2) {
    // Edges of the triangle
    Eigen::Vector3f edge0 = v1 - v0;
    Eigen::Vector3f edge1 = v2 - v0;
    Eigen::Vector3f v0_to_point = point - v0;

    // Compute dot products
    float a = edge0.dot(edge0);
    float b = edge0.dot(edge1);
    float c = edge1.dot(edge1);
    float d = edge0.dot(v0_to_point);
    float e = edge1.dot(v0_to_point);

    // Compute barycentric coordinates
    float det = a * c - b * b;
    float s = b * e - c * d;
    float t = b * d - a * e;

    // Check if point is in the triangle
    if (s + t <= det && s >= 0 && t >= 0 && det > 0) {
        float invDet = 1.0f / det;
        s *= invDet;
        t *= invDet;
        return (v0_to_point - s * edge0 - t * edge1).norm();
    }

    // Otherwise, compute the distance to the edges or vertices
    auto distToSegment = [](const Eigen::Vector3f& p, const Eigen::Vector3f& v1, const Eigen::Vector3f& v2) {
        Eigen::Vector3f edge = v2 - v1;
        float t = std::max(0.0f, std::min(1.0f, (p - v1).dot(edge) / edge.dot(edge)));
        Eigen::Vector3f projection = v1 + t * edge;
        return (p - projection).norm();
    };

    return std::min({
        distToSegment(point, v0, v1),
        distToSegment(point, v1, v2),
        distToSegment(point, v2, v0)
    });
}

float computeMinPointToSurfaceDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr& points,
                                       const pcl::PolygonMesh& mesh,
                                       const std::vector<AABB>& aabbs,
                                       const Eigen::Vector3f motion_dir) {
    pcl::PointCloud<pcl::PointXYZ> vertices;
    pcl::fromPCLPointCloud2(mesh.cloud, vertices);

    // Step 1: Compute center and max radius
    Eigen::Vector3f center = Eigen::Vector3f::Zero();
    float max_radius = 0.0f;

    for (const auto& point : points->points) {
        center += Eigen::Vector3f(point.x, point.y, point.z);
    }
    center /= static_cast<float>(points->size());  // Compute average (centroid)

    for (const auto& point : points->points) {
        float dist = (Eigen::Vector3f(point.x, point.y, point.z) - center).norm();
        if (dist > max_radius) {
            max_radius = dist;  // Store max distance from center
        }
    }

    // Step 2: Find the furthest point **opposite** to the motion direction
    Eigen::Vector3f furthest_point = center;
    float min_dot_product = std::numeric_limits<float>::max();  // Store min dot product

    for (const auto& point : points->points) {
        Eigen::Vector3f p = Eigen::Vector3f(point.x, point.y, point.z);
        float dot_product = (p - center).dot(motion_dir);
        if (dot_product < min_dot_product) {
            min_dot_product = dot_product;
            furthest_point = p;
        }
    }

    float min_distance = std::numeric_limits<float>::max();

    // Step 3: Check relevant bounding boxes using `furthest_point`
    for (const auto& box : aabbs) {
        Eigen::Vector3f box_closest_point;
        float max_search_distance = std::min(4.0f, min_distance) + 2*max_radius;  // Adjusted search distance

        if (isPointNearAABB(furthest_point, box, max_search_distance, box_closest_point)) {
            if (motion_dir.isZero() || (box_closest_point - furthest_point).dot(motion_dir) > 0) {  
                // Extract triangle vertices
                const auto& triangle = mesh.polygons[box.triangle_index];
                Eigen::Vector3f v0(vertices.points[triangle.vertices[0]].x,
                                   vertices.points[triangle.vertices[0]].y,
                                   vertices.points[triangle.vertices[0]].z);
                Eigen::Vector3f v1(vertices.points[triangle.vertices[1]].x,
                                   vertices.points[triangle.vertices[1]].y,
                                   vertices.points[triangle.vertices[1]].z);
                Eigen::Vector3f v2(vertices.points[triangle.vertices[2]].x,
                                   vertices.points[triangle.vertices[2]].y,
                                   vertices.points[triangle.vertices[2]].z);

                // Step 4: Check distance to triangle for all points if the last point is within range
                for (const auto& point : points->points) {
                    Eigen::Vector3f p(point.x, point.y, point.z);
                    if (isPointNearAABB(p, box, max_search_distance - 2*max_radius, box_closest_point)) {
                    float dist = pointToTriangleDistance(p, v0, v1, v2);
                    if (dist < min_distance) {
                        min_distance = dist;
                    }
                    }
                }
            }
        }
    }

    return min_distance;
}
   
// State validity checker using MoveIt collision detection
bool isStateValid(const ob::State *state)
{
    const auto *compound_state = state->as<ob::CompoundStateSpace::StateType>();
    const auto *position = compound_state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *orientation = compound_state->as<ob::SO3StateSpace::StateType>(1);

    // Update robot state in the PlanningScene
    robot_state::RobotState& robot_state = planning_scene_instance->getCurrentStateNonConst();
    const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("drone_base");

    if (!joint_model_group)
    {
        ROS_ERROR("Joint group 'drone_base' not found in the robot model.");
        return false;
    }

    std::vector<double> joint_positions = {
        position->values[0], // x position
        position->values[1], // y position
        position->values[2], // z position
        orientation->x,      // qx (quaternion x)
        orientation->y,      // qy (quaternion y)
        orientation->z,      // qz (quaternion z)
        orientation->w       // qw (quaternion w)
    };

    robot_state.setJointGroupPositions(joint_model_group, joint_positions);
    planning_scene_instance->setCurrentState(robot_state);

    // Perform collision checking
    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_request.max_contacts = 10;
    collision_detection::CollisionResult collision_result;

    planning_scene_instance->checkCollision(collision_request, collision_result);

    if (collision_result.collision)
    {
        //ROS_WARN("Collision detected.");
        return false;
    }

    return true;
}

// Initialize PlanningScene
void initializePlanningScene()
{
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

    if (!robot_model)
    {
        ROS_ERROR("Failed to load robot model.");
        ros::shutdown();
        return;
    }

    planning_scene_instance = std::make_shared<planning_scene::PlanningScene>(robot_model);
}

void planWithSimpleSetup(ros::Publisher &path_pub, ros::Publisher &marker_pub, ros::Publisher &distance_pub)
{
    // Create the position space (RealVectorStateSpace) for (x, y, z)
    auto positionSpace = std::make_shared<ob::RealVectorStateSpace>(3);  // 3D vector space

    // Set bounds for the position (x, y, z)
    ob::RealVectorBounds bounds(3);
    bounds.setLow(0, -10);   // Set the lower bound of x (index 2) to 0
    bounds.setLow(1, -10); // Set the lower bound of y (index 0) to -20
    bounds.setLow(2, -0.5); // Set the lower bound of z (index 1) to -20
    bounds.setHigh(0, 10); // Set the upper bound of x (index 0) to 20
    bounds.setHigh(1, 10); // Set the upper bound of y (index 1) to 20
    bounds.setHigh(2, 4); // Set the upper bound of z (index 2) to 20
    positionSpace->setBounds(bounds);
    // Create the orientation space (SO3StateSpace) for quaternion representation
    auto orientationSpace = std::make_shared<ob::SO3StateSpace>();  

    // Combine position and orientation into a CompoundStateSpace
    auto combinedSpace = std::make_shared<ob::CompoundStateSpace>();
    combinedSpace->addSubspace(positionSpace, 1);  
    combinedSpace->addSubspace(orientationSpace, 1);  

    // Create an instance of ompl::geometric::SimpleSetup
    og::SimpleSetup ss(combinedSpace);

    // Set the state validity checker
    ss.setStateValidityChecker([](const ob::State *state) { return isStateValid(state); });

    // Create and set the planner (RRTConnect)
    //std::shared_ptr<og::RRTConnect> planner = std::make_shared<og::RRTConnect>(ss.getSpaceInformation());
    std::shared_ptr<og::RRTstar> planner = std::make_shared<og::RRTstar>(ss.getSpaceInformation());
    ss.setPlanner(planner);

    // Create the PathLengthOptimizationObjective and set it to the SimpleSetup
    ob::OptimizationObjectivePtr opt_obj = std::make_shared<ob::PathLengthOptimizationObjective>(ss.getSpaceInformation());
    ss.setOptimizationObjective(opt_obj);

    // Create a start state with position and orientation (quaternion)
    ob::ScopedState<> start(combinedSpace);
    start[0] = 0.0;  
    start[1] = -9.0;  
    start[2] = 2.0;  
    start[3] = 0.0;  
    start[4] = 0.0;  
    start[5] = 0.0;  
    start[6] = 1.0;  

    // Create a goal state with position and orientation (quaternion)
    ob::ScopedState<> goal(combinedSpace);
    goal[0] = 0.0;  
    goal[1] = 9.0;  
    goal[2] = 2.0;  
    goal[3] = 0.0;  //sqrt(2) / 2
    goal[4] = 0.0;  
    goal[5] = 0.0;  
    goal[6] = 1.0;  

    // Set start and goal states using SimpleSetup directly
    ss.setStartAndGoalStates(start, goal);

    // Try to solve the problem
    ob::PlannerStatus solved = ss.solve(3000.0);

    if (solved)
    {
        ROS_INFO("Found solution:");
        
        ss.simplifySolution();
        
        og::PathGeometric &path = ss.getSolutionPath();
        double path_length = path.length();
        ROS_INFO("Path length: %f", path_length);

        // Interpolate the path
        path.interpolate(1000); // Add intermediate points for smoother and feasible path

        // Load the STL file for the map
        pcl::PolygonMesh map_mesh;
        std::string package_path = ros::package::getPath("my_ompl_planner");
        std::string map_path = package_path + "/meshes/map.stl";
        if (pcl::io::loadPolygonFileSTL(map_path, map_mesh) == -1)
        {
            ROS_ERROR("Failed to load map STL file from path: %s", map_path.c_str());
            return;
        }

        // Resolve and load the robot STL file
        pcl::PolygonMesh robot_mesh;
        std::string robot_path = package_path + "/meshes/drone_with_cylinders.stl";
        if (pcl::io::loadPolygonFileSTL(robot_path, robot_mesh) == -1)
        {
            ROS_ERROR("Failed to load robot STL file from path: %s", robot_path.c_str());
            return;
        }

        // Convert the robot mesh to a point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr robot_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::fromPCLPointCloud2(robot_mesh.cloud, *robot_cloud);


        std_msgs::Float32MultiArray distance_msg;
        std::vector<float> distances;
        
        // Publish the path as a nav_msgs/Path
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";  
        path_msg.header.stamp = ros::Time::now(); 

        // Create a Marker message for axes visualization
        visualization_msgs::Marker marker_msg;
        marker_msg.header.frame_id = "map";  
        marker_msg.header.stamp = ros::Time::now();
        marker_msg.ns = "ompl_solution";
        marker_msg.id = 0;
        marker_msg.type = visualization_msgs::Marker::LINE_LIST;
        marker_msg.action = visualization_msgs::Marker::ADD;
        marker_msg.scale.x = 0.01;  
        marker_msg.scale.y = 0.01;  
        marker_msg.scale.z = 0.01;  
        marker_msg.color.a = 1.0f;

        // Convert OMPL path to ROS path message
        for (size_t i = 0; i < ss.getSolutionPath().getStateCount(); ++i)
        {
            geometry_msgs::PoseStamped pose;
            const ob::State *state = path.getState(i);
            // Extract position and orientation from the state
            const ob::CompoundStateSpace::StateType *compound_state = state->as<ob::CompoundStateSpace::StateType>();
            const ob::RealVectorStateSpace::StateType *real_state = compound_state->as<ob::RealVectorStateSpace::StateType>(0);
            pose.pose.position.x = real_state->values[0];
            pose.pose.position.y = real_state->values[1];
            pose.pose.position.z = real_state->values[2];

            const ob::SO3StateSpace::StateType *so3_state = compound_state->as<ob::SO3StateSpace::StateType>(1);
            pose.pose.orientation.x = so3_state->x;
            pose.pose.orientation.y = so3_state->y;
            pose.pose.orientation.z = so3_state->z;
            pose.pose.orientation.w = so3_state->w;

            path_msg.poses.push_back(pose);

            // Create a tf::Quaternion from the pose orientation
            tf::Quaternion quat(
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
                pose.pose.orientation.w);

            // Convert to a rotation matrix
            tf::Matrix3x3 rotation_matrix(quat);

            // Define unit vectors for X, Y, and Z axes
            tf::Vector3 x_axis(0.25, 0.0, 0.0);  // Red axis (X)
            tf::Vector3 y_axis(0.0, 0.5, 0.0);  // Green axis (Y)
            tf::Vector3 z_axis(0.0, 0.0, 1.0);  // Blue axis (Z)

            // Apply the rotation matrix to the axes to account for orientation
            tf::Vector3 rotated_x = rotation_matrix * x_axis;
            tf::Vector3 rotated_y = rotation_matrix * y_axis;
            tf::Vector3 rotated_z = rotation_matrix * z_axis;

            // X-axis (Red)
            marker_msg.color.r = 1.0f;  // Set color to red for X-axis
            marker_msg.points.push_back(pose.pose.position);
            geometry_msgs::Point x_end;
            x_end.x = pose.pose.position.x + rotated_x.x();
            x_end.y = pose.pose.position.y + rotated_x.y();
            x_end.z = pose.pose.position.z + rotated_x.z();
            marker_msg.points.push_back(x_end);

            // Y-axis (Green)
            marker_msg.color.g = 1.0f;  // Set color to green for Y-axis
            marker_msg.points.push_back(pose.pose.position);
            geometry_msgs::Point y_end;
            y_end.x = pose.pose.position.x + rotated_y.x();
            y_end.y = pose.pose.position.y + rotated_y.y();
            y_end.z = pose.pose.position.z + rotated_y.z();
            marker_msg.points.push_back(y_end);

            // Z-axis (Blue)
            marker_msg.color.b = 1.0f;  // Set color to blue for Z-axis
            marker_msg.points.push_back(pose.pose.position);
            geometry_msgs::Point z_end;
            z_end.x = pose.pose.position.x + rotated_z.x();
            z_end.y = pose.pose.position.y + rotated_z.y();
            z_end.z = pose.pose.position.z + rotated_z.z();
            marker_msg.points.push_back(z_end);
            float min_distance = 1 ;

            if (i < ss.getSolutionPath().getStateCount() - 1)
            {
                // Distance Calculation
                geometry_msgs::PoseStamped pose_2;
                const ob::State *state_2 = path.getState(i+1);
                // Extract position and orientation from the state
                const ob::CompoundStateSpace::StateType *compound_state_2 = state_2->as<ob::CompoundStateSpace::StateType>();
                const ob::RealVectorStateSpace::StateType *real_state_2 = compound_state_2->as<ob::RealVectorStateSpace::StateType>(0);
                pose_2.pose.position.x = real_state_2->values[0];
                pose_2.pose.position.y = real_state_2->values[1];
                pose_2.pose.position.z = real_state_2->values[2];
                pcl::PointCloud<pcl::PointXYZ>::Ptr robot_cloud(new pcl::PointCloud<pcl::PointXYZ>());
                pcl::PointXYZ robot_point(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                robot_cloud->push_back(robot_point);

                Eigen::Vector3f p1(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
                Eigen::Vector3f p2(pose_2.pose.position.x, pose_2.pose.position.y, pose_2.pose.position.z);

                std::vector<AABB> aabbs = precomputeAABBs(map_mesh);
                Eigen::Vector3f motion_dir = (p2 - p1).normalized();
                // Compute the distance from the robot point to the map mesh
                min_distance = computeMinPointToSurfaceDistance(robot_cloud, map_mesh, aabbs, motion_dir);
                distances.push_back(min_distance);
            }
            else
                distances.push_back(min_distance);
        }

        ROS_INFO("Publishing path with %lu poses", path_msg.poses.size());
        path_pub.publish(path_msg);  
        marker_pub.publish(marker_msg); 
        distance_msg.data = distances;
        distance_pub.publish(distance_msg);
    }
    else
    {
        ROS_INFO("No solution found");
    }
}

void timerCallback(const ros::TimerEvent &, ros::Publisher path_pub, ros::Publisher marker_pub, ros::Publisher distance_pub)
{
    planWithSimpleSetup(path_pub, marker_pub, distance_pub);
}


void publishMapSTL(const ros::Publisher &marker_pub)
{
    visualization_msgs::Marker map_marker;

    // Set the marker properties
    map_marker.header.frame_id = "map"; // Adjust this frame if necessary
    map_marker.header.stamp = ros::Time::now();
    map_marker.ns = "map_stl";
    map_marker.id = 0; // Unique ID
    map_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    map_marker.action = visualization_msgs::Marker::ADD;

    // Specify the STL file path (use package:// if it's in a package)
    map_marker.mesh_resource = "package://my_ompl_planner/meshes/map.stl";

    // Set position and orientation
    map_marker.pose.position.x = 0.0;
    map_marker.pose.position.y = 0.0;
    map_marker.pose.position.z = 0.0;
    map_marker.pose.orientation.x = 0.0;
    map_marker.pose.orientation.y = 0.0;
    map_marker.pose.orientation.z = 0.0;
    map_marker.pose.orientation.w = 1.0;

    // Set scale
    map_marker.scale.x = 1.0; // Adjust scale as needed
    map_marker.scale.y = 1.0;
    map_marker.scale.z = 1.0;

    // Set color
    map_marker.color.a = 1.0; // Alpha (transparency)
    map_marker.color.r = 0.5;
    map_marker.color.g = 0.5;
    map_marker.color.b = 0.5;

    // Publish the marker
    marker_pub.publish(map_marker);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_planner_node");
    ros::NodeHandle nh;

    initializePlanningScene();

    // Publisher for path visualization
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("ompl_solution_path", 1, true);

    // Publisher for Marker visualization
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("ompl_solution_marker", 1, true);
    ros::Publisher marker_pub_map = nh.advertise<visualization_msgs::Marker>("ompl_map_marker", 1, true);
    ros::Publisher distance_pub = nh.advertise<std_msgs::Float32MultiArray>("path_distances", 1, true);
    
    // Publish the map STL
    publishMapSTL(marker_pub_map);

    // Timer to call the planner function at intervals
    ros::Timer timer = nh.createTimer(ros::Duration(3100.0), 
        boost::bind(timerCallback, _1, path_pub, marker_pub, distance_pub));  
    //planWithSimpleSetup(path_pub, marker_pub);
    ros::spin();

    return 0;
}