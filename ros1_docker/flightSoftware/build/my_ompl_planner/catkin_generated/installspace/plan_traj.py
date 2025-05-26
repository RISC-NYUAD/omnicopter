#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Twist, Accel
from std_msgs.msg import Header, Float32MultiArray
import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
from omni_firmware.msg import FullPose
import math


success = False  
last_published_pose = None  # To store the last published trajectory
working = False

def quaternion_distance(q1, q2):
    """Compute the quaternion distance."""
    dot_product = abs(q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3])
    return 1 - dot_product  # Smaller means closer orientations

def pose_distance(p1, p2):
    """Compute Euclidean distance between two positions."""
    return np.linalg.norm(np.array(p1) - np.array(p2))


# Global variable to store obstacle distances
obstacle_distances = None
v_max = np.array([
    [-1.0, 1.0],  # X-axis linear velocity limits
    [-1.0, 1.0],  # Y-axis linear velocity limits
    [-1.0, 1.0],  # Z-axis linear velocity limits
    [0.0, 0.0],   # Quaternion w velocity limits (no movement)
    [0.0, 0.0],   # Quaternion x velocity limits (no movement)
    [0.0, 0.0],   # Quaternion y velocity limits (no movement)
    [0.0, 0.0]    # Quaternion z velocity limits (no movement)
])

def quaternion_to_angular_velocity(quaternion, quaternion_dot):
    """
    Convert quaternion and its time derivative to angular velocity in the body frame.

    Args:
        quaternion (numpy.ndarray): A 4-element array representing [q0, q1, q2, q3].
        quaternion_dot (numpy.ndarray): A 4-element array representing the time derivative of the quaternion [q0_dot, q1_dot, q2_dot, q3_dot].

    Returns:
        numpy.ndarray: A 3-element array representing angular velocity [wx, wy, wz].
    """
    q0, q1, q2, q3 = quaternion

    # Define the 4x3 transformation matrix W
    W = np.array([
        [-q1, -q2, -q3],
        [ q0, -q3,  q2],
        [ q3,  q0, -q1],
        [-q2,  q1,  q0]
    ])  # Shape (4, 3)

    # Compute angular velocity: ω = 2 * W^T * q_dot
    omega = -2 * W.T @ quaternion_dot  # (3x4) @ (4x1) → (3x1)
    omega = omega * np.array([-1, 1, -1])  # Ensure element-wise multiplication
    return omega

def quaternion_to_angular_acceleration(quaternion, quaternion_dot, quaternion_ddot):
    """
    Converts quaternion second derivatives to angular acceleration.
    
    Args:
        quaternion: List or array of quaternion components [q0, q1, q2, q3].
        quaternion_dot: List or array of quaternion derivatives [dq0, dq1, dq2, dq3].
        quaternion_ddot: List or array of quaternion second derivatives [ddq0, ddq1, ddq2, ddq3].
    
    Returns:
        Angular acceleration as [alpha_x, alpha_y, alpha_z].
    """
    q0, q1, q2, q3 = quaternion
    dq0, dq1, dq2, dq3 = quaternion_dot
    ddq0, ddq1, ddq2, ddq3 = quaternion_ddot

    # Compute angular velocity
    omega = 2 * np.array([
        -q1 * dq0 - q2 * dq1 - q3 * dq2,
        q0 * dq0 + q2 * dq2 - q3 * dq1,
        q0 * dq1 - q1 * dq2 + q3 * dq0,
        q0 * dq2 + q1 * dq1 - q2 * dq0
    ])

    # Compute time derivative of angular velocity (dot(omega))
    dot_omega = 2 * np.array([
        -q1 * ddq0 - q2 * ddq1 - q3 * ddq2 - dq1 * dq0 - dq2 * dq1 - dq3 * dq2,
        q0 * ddq0 + q2 * ddq2 - q3 * ddq1 + dq0 * dq0 + dq2 * dq2 - dq3 * dq1,
        q0 * ddq1 - q1 * ddq2 + q3 * ddq0 + dq0 * dq1 - dq1 * dq2 + dq3 * dq0,
        q0 * ddq2 + q1 * ddq1 - q2 * ddq0 + dq0 * dq2 + dq1 * dq1 - dq2 * dq0
    ])

    # Angular acceleration (only take the vector part)
    angular_acceleration = dot_omega[1:]  # [alpha_x, alpha_y, alpha_z]
    return angular_acceleration

# Helper function to normalize quaternions
def normalize_quaternion(q):
    return q / np.linalg.norm(q)

# Callback for the `path_distances` topic
def distances_callback(msg):
    global obstacle_distances
    obstacle_distances = np.array(msg.data)

def velocity_limit_varying(s):
    global v_max
    """Adaptive velocity limit based on obstacle distances."""
    global obstacle_distances
    if obstacle_distances is None:
        # Default velocity limits if distances are not yet received
        return (v_max)
    min_index = np.argmax(obstacle_distances)
    min_value = obstacle_distances[min_index]
    S = s * (obstacle_distances.size -1 )
    min_distance = obstacle_distances[math.ceil(S)]
    max_velocity = np.clip(min_distance / 1, 0.1, 1.0)  # Adjust scaling as needed
    return (v_max * max_velocity)

# Function to compute trajectory
def compute_trajectory(path_positions, path_orientations):
    waypoints_pos = np.array(path_positions)
    waypoints_ori = np.array(path_orientations)
    for i in range(1, waypoints_ori.shape[0]):  # Start from the second quaternion
        # Check if the dot product is negative (indicating a significant difference)
        if np.dot(waypoints_ori[i], waypoints_ori[i - 1]) < 0:
            waypoints_ori[i:] *= -1         
    

    n_points = 1000
    s = np.linspace(0, 1, len(waypoints_pos))
    dense_s = np.linspace(0, 1, n_points)

    dense_pos = np.array([np.interp(dense_s, s, waypoints_pos[:, i]) for i in range(3)]).T
    key_rots = R.from_quat(waypoints_ori)
    slerp = Slerp(s, key_rots)
    dense_rots = slerp(dense_s)
    dense_ori = dense_rots.as_quat()

    dense_path = np.hstack((dense_pos, dense_ori))

    #max_linear_velocity = np.array([[-1.0, 1.0],[-1.0, 1.0], [-1.0, 1.0]])
    max_linear_velocity = np.array([1.0, 1.0, 1.0])
    max_linear_acceleration = np.array([0.5, 0.5, 0.5])
    q_rate_limit = 0.5

 
    # Define angular velocity and acceleration limits
    max_angular_velocity = np.array([0.1, 0.1, 0.1])  # rad/s
    max_angular_acceleration = np.array([0.1, 0.1, 0.1])  # rad/s^2

    # Convert angular velocity to quaternion rate limits
    def angular_velocity_to_quaternion_rate(angular_velocity, quaternion):
        wx, wy, wz = angular_velocity
        q0, q1, q2, q3 = quaternion
        q_dot = 0.5 * np.array([
            -q1 * wx - q2 * wy - q3 * wz,
             q0 * wx + q2 * wz - q3 * wy,
             q0 * wy - q1 * wz + q3 * wx,
             q0 * wz + q1 * wy - q2 * wx
        ])
        return np.abs(q_dot)

    q_rate_limit = np.max([
        angular_velocity_to_quaternion_rate(max_angular_velocity, [1, 0, 0, 0]),
        angular_velocity_to_quaternion_rate(max_angular_velocity, [0, 1, 0, 0]),
        angular_velocity_to_quaternion_rate(max_angular_velocity, [0, 0, 1, 0]),
        angular_velocity_to_quaternion_rate(max_angular_velocity, [0, 0, 0, 1])
    ], axis=0)
    # Convert angular acceleration to quaternion rate derivative limits
    def angular_acceleration_to_quaternion_rate_derivative(angular_acceleration, quaternion):
        ax, ay, az = angular_acceleration
        q0, q1, q2, q3 = quaternion
        q_ddot = 0.5 * np.array([
            -q1 * ax - q2 * ay - q3 * az,
             q0 * ax + q2 * az - q3 * ay,
             q0 * ay - q1 * az + q3 * ax,
             q0 * az + q1 * ay - q2 * ax
        ])
        return np.abs(q_ddot)

    q_rate_derivative_limit = np.max([
        angular_acceleration_to_quaternion_rate_derivative(max_angular_acceleration, [1, 0, 0, 0]),
        angular_acceleration_to_quaternion_rate_derivative(max_angular_acceleration, [0, 1, 0, 0]),
        angular_acceleration_to_quaternion_rate_derivative(max_angular_acceleration, [0, 0, 1, 0]),
        angular_acceleration_to_quaternion_rate_derivative(max_angular_acceleration, [0, 0, 0, 1])
    ], axis=0)

    q_rate_limit_lu = np.array([[-abs(q_rate_limit[0]), abs(q_rate_limit[0])],[-abs(q_rate_limit[1]), abs(q_rate_limit[1])],[-abs(q_rate_limit[2]), abs(q_rate_limit[2])],[-abs(q_rate_limit[3]), abs(q_rate_limit[3])]])
    global v_max
    v_max = np.array([[-max_linear_velocity[0],max_linear_velocity[0]],[-max_linear_velocity[1],max_linear_velocity[1]],[-max_linear_velocity[2],max_linear_velocity[2]], *q_rate_limit_lu])
    a_max = np.array([*max_linear_acceleration, *q_rate_derivative_limit])

    velocity_constraint = constraint.JointVelocityConstraintVarying(velocity_limit_varying)
    acceleration_constraint = constraint.JointAccelerationConstraint(a_max)
    path = ta.SplineInterpolator(dense_s, dense_path)
    constraints = [velocity_constraint, acceleration_constraint]
    instance = algo.TOPPRA(constraints, path, solver_wrapper='seidel')

    instance.compute_parameterization(0, 0)
    trajectory = instance.compute_trajectory()
    #plot_trajectory(trajectory)

    return trajectory

# Publish the trajectory in real-time
def publish_trajectory(trajectory, publisher):
    global last_published_pose
    rate = rospy.Rate(100)  # Publishing at 100 Hz
    start_time = rospy.Time.now()

    #file = open("/home/ama10362/my_workspace/quaternion_dot.txt", "w")
    #file.write("dq0 dq1 dq2 dq3\n")  # Header

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = (current_time - start_time).to_sec()

        # Break the loop if the trajectory duration has elapsed
        if elapsed_time > trajectory.duration:
            break

        # Evaluate the trajectory at the elapsed time
        pose = trajectory.eval(elapsed_time)
        vel = trajectory.evald(elapsed_time)
        acc = trajectory.evaldd(elapsed_time)

        # Extract quaternion and its derivatives
        quaternion = pose[3:7]  # [q0, q1, q2, q3]
        quaternion_dot = vel[3:7]  # [dq0, dq1, dq2, dq3]
        quaternion_ddot = acc[3:7]  # [ddq0, ddq1, ddq2, ddq3]

        #file.write(f"{quaternion_dot[0]} {quaternion_dot[1]} {quaternion_dot[2]} {quaternion_dot[3]}\n")

        # Compute angular velocity
        angular_velocity = quaternion_to_angular_velocity(quaternion, quaternion_dot)

        # Compute angular acceleration (optional)
        angular_acceleration = quaternion_to_angular_acceleration(quaternion, quaternion_dot, quaternion_ddot)

        # Assign to messages
        pose_msg = Pose()
        pose_msg.position.x = pose[0]
        pose_msg.position.y = pose[1]
        pose_msg.position.z = pose[2]
        pose_msg.orientation.x = pose[3]
        pose_msg.orientation.y = pose[4]
        pose_msg.orientation.z = pose[5]
        pose_msg.orientation.w = pose[6]
        last_published_pose = pose_msg

        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = vel[1]
        vel_msg.linear.z = vel[2]
        vel_msg.angular.x = angular_velocity[0]
        vel_msg.angular.y = angular_velocity[1]
        vel_msg.angular.z = angular_velocity[2]

        acc_msg = Accel()
        acc_msg.linear.x = acc[0]
        acc_msg.linear.y = acc[1]
        acc_msg.linear.z = acc[2]
        # Assign angular acceleration if computed
        acc_msg.angular.x = angular_acceleration[0]
        acc_msg.angular.y = angular_acceleration[1]
        acc_msg.angular.z = angular_acceleration[2]

        full_pose_msg = FullPose(
            header=Header(stamp=current_time),
            pose=pose_msg,
            vel=vel_msg,
            acc=acc_msg,
        )

        # Publish the current trajectory point
        publisher.publish(full_pose_msg)

        # Sleep to maintain the desired publishing rate
        rate.sleep()

# Callback function to process the path message
def path_callback(msg):
    global success
    global working
    rospy.loginfo("Received path with %d poses", len(msg.poses))

    path_positions = []
    path_orientations = []
    for pose_stamped in msg.poses:
        position = pose_stamped.pose.position
        orientation = pose_stamped.pose.orientation
        path_positions.append([position.x, position.y + 2.7, position.z])
        path_orientations.append([orientation.x, orientation.y, orientation.z, orientation.w])

    path_orientations = [normalize_quaternion(q) for q in path_orientations]
    #plot_path(path_positions,path_orientations)
    trajectory = compute_trajectory(path_positions, path_orientations)
    #print(trajectory)
    #plot_trajectory(trajectory)
    if trajectory:
        success = True  # Set success to True only after publishing
        working = True
        publish_trajectory(trajectory, trajectory_publisher)
        working = False

def pose_ref_callback(msg):
    global success
    global last_published_pose
    global working
    if last_published_pose:
        pos_ref = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        pos_published = [last_published_pose.position.x, 
                        last_published_pose.position.y, 
                        last_published_pose.position.z]

        ori_ref = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        ori_published = [last_published_pose.orientation.x, 
                        last_published_pose.orientation.y, 
                        last_published_pose.orientation.z, 
                        last_published_pose.orientation.w]

        pos_error = pose_distance(pos_ref, pos_published)
        ori_error = quaternion_distance(ori_ref, ori_published)
        # Define small error thresholds
        pos_threshold = 0.05  # Adjust based on system precision
        ori_threshold = 0.01  # Small quaternion error threshold

        if pos_error < pos_threshold and ori_error < ori_threshold and not working:
            success = False  # Reset success only when error is small
    if not success:
        trajectory_publisher.publish(msg)
        
# Main function
def main():
    global trajectory_publisher
    rospy.init_node("trajectory_planner_node", anonymous=True)
    rospy.Subscriber("ompl_solution_path", Path, path_callback)
    rospy.Subscriber("/pose_ref", FullPose, pose_ref_callback)
    rospy.Subscriber("path_distances", Float32MultiArray, distances_callback)  # Subscribe to obstacle distances
    trajectory_publisher = rospy.Publisher("pose_d", FullPose, queue_size=10)
    rospy.loginfo("Trajectory planner node started. Waiting for path messages...")
    rospy.spin()

if __name__ == "__main__":
    main()
