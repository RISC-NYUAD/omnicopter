#include "ros/ros.h"
#include <cmath>
#include <deque>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <omni_firmware/FullPose.h>
#include "omni_firmware/ZeroUKF.h"

#include "ukf.h"


u_int32_t seqID = 0;
// SEN (State Estimation Node)
namespace sen{

    struct filter_context_s{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    ukf::state_s zero;
    ukf::state_s::dcov_s zerocov;

    ukf::state_s state;
    ukf::state_s::dcov_s cov;

    ukf::filter_s filter;

    ros::Time filter_time;

    std::deque<Eigen::Quaternion<double>> IMU_orientation_history;
    std::deque<Eigen::Quaternion<double>> VICON_orientation_history;
    Eigen::Quaternion<double> bias_q_imu;
    Eigen::Quaternion<double> bias_q_vicon;
    Eigen::Quaternion<double> zero_q_vicon;
    };

    struct sen_snapshot_s{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    mutable bool fused;

    mutable ukf::state_s state;
    mutable ukf::state_s::dcov_s cov;
    };


    void process_IMU_measurement(filter_context_s **context, const sensor_msgs::Imu &msg){
        ukf::state_s temp_state;
        ukf::state_s::dcov_s temp_cov;
        // check if current time before measurement time, skip measurement
        ros::Time current_time = ros::Time::now();
        ros::Time msg_time = msg.header.stamp;
        if (current_time >= msg_time){
        ros::Duration time_diff =  msg_time - (*context)->filter_time;
        //(*context)->filter.set((*context)->state, (*context)->cov);
        (*context)->filter.predict(time_diff.toSec());
        // else update filter with IMU measurements  
            /* angular velocity */
    if (!std::isnan(msg.angular_velocity.x)) {
       Eigen::Vector3d ang_v;
       ang_v << 
       (fabs(msg.angular_velocity.x) < 0.005 ? 0.0 : msg.angular_velocity.x),
       (fabs(msg.angular_velocity.y) < 0.005 ? 0.0 : msg.angular_velocity.y),
       (fabs(msg.angular_velocity.z) < 0.005 ? 0.0 : msg.angular_velocity.z);
       (*context)->filter.measure.iw() << ang_v(0), ang_v(1), ang_v(2);
        //msg.angular_velocity.x*4,
        //msg.angular_velocity.y*4,
        //msg.angular_velocity.z*4;


        /* no covariance was added to the IMU data, provide a value from datasheet */
        // IMU: ICM42688, error is 1.7453e-5, too low, may cause computation errors,
        // bit conversion error is 0.003328 (assuming range of 125dps), put 0.004
         (*context)->filter.measure.iw_cov() <<
          0.025,  0.,    0.,
          0.,    0.025,  0.,
          0.,    0.,    0.025;
      }

      //temp_state = ukf::state_s::Identity();
      //temp_cov = ukf::state_s::dcov_s::Identity();
      
    // update filter state after each measurement
      //(*context)->filter.update(temp_state, temp_cov); 
    /* update state for next iteration */
     // (*context)->filter.set(temp_state, temp_cov);

    Eigen::Quaternion<double> q(msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z);
    (*context)->IMU_orientation_history.push_back(q);
    q = (*context)->bias_q_imu.conjugate() * q;

    Eigen::Vector3d a;
    a << msg.linear_acceleration.x,
        msg.linear_acceleration.y,
        msg.linear_acceleration.z;
    //a = a + Eigen::Matrix<double, 3, 1>(0, 0, 9.81 - 9.44);
    a = q * a;
    a = a - Eigen::Matrix<double, 3, 1>(0, 0, 9.80665);
    /* linear acceleration - XXX when intrisic, assume accelerometer */
    if (!std::isnan(msg.linear_acceleration.x)) {
       //(*context)->filter.measure.iawg()  <<
        //msg.linear_acceleration.x,
        //msg.linear_acceleration.y,
        //msg.linear_acceleration.z;
       (*context)->filter.measure.a()  <<
       a(0), a(1), a(2);

    
        /* no covariance was added to the IMU data, provide a value from datasheet */
        // IMU: ICM42688, error is 0.001N
        // bit conversion error is higher, 0.004786, put 0.005
         (*context)->filter.measure.iawg_cov()  <<
          0.025, 0., 0.,
          0., 0.025, 0.,
          0., 0., 0.025;
    }

      //temp_state = ukf::state_s::Identity();
      //temp_cov = ukf::state_s::dcov_s::Identity();
      
    // update filter state after each measurement
      (*context)->filter.update(temp_state, temp_cov); 
    /* update state for next iteration */
      (*context)->filter.set(temp_state, temp_cov);

      (*context)->filter_time = msg_time;
      (*context)->state = temp_state;
      (*context)->cov = temp_cov;
        }
    }

    void process_VICON_measurement(filter_context_s **context, const geometry_msgs::PoseStamped &msg){
        ukf::state_s temp_state;
        ukf::state_s::dcov_s temp_cov;
        // check if current time before measurement time, skip measurement
        ros::Time current_time = ros::Time::now();
        ros::Time msg_time = msg.header.stamp;
        if (current_time >= msg_time){
        
        ros::Duration time_diff = (*context)->filter_time - msg_time;
        //(*context)->filter.set((*context)->state, (*context)->cov);
        (*context)->filter.predict(time_diff.toSec());
        // else update filter with vicon measurements  
            /* orientation */
        Eigen::Quaternion<double> q(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    (*context)->VICON_orientation_history.push_back(q);
    q = (*context)->bias_q_vicon.conjugate() * q;


    if (!std::isnan(msg.pose.orientation.x)) {
      (*context)->filter.measure.q() << q.x(), q.y(), q.z(), q.w();

        /* Vicon msg does not provide any covariance */
          (*context)->filter.measure.q_cov() <<
          1e-6, 1e-8, 1e-8, 1e-8,
          1e-8, 1e-6, 1e-8, 1e-8,
          1e-8, 1e-8, 1e-6, 1e-8,
          1e-8, 1e-8, 1e-8, 1e-6;
          (*context)->filter.measure.q_cov() = (*context)->filter.measure.q_cov()*1000;
      }
    

      /* position */
    if (!std::isnan(msg.pose.position.x)) {
      (*context)->filter.measure.p() <<
        msg.pose.position.x,
        msg.pose.position.y,
        msg.pose.position.z;

      /* Vicon msg does not provide any covariance */
          (*context)->filter.measure.p_cov() <<
          1e-6,  1e-8,  1e-8,
          1e-8,  1e-6,  1e-8,
          1e-8,  1e-8,  1e-6;
      }

      temp_state = ukf::state_s::Identity();
      temp_cov = ukf::state_s::dcov_s::Identity();
      
    // update filter state after each measurement
      (*context)->filter.update(temp_state, temp_cov); 

    /* update state for next iteration */
      (*context)->filter.set(temp_state, temp_cov);
      (*context)->filter_time = msg_time;
      (*context)->state = temp_state;
      (*context)->cov = temp_cov;
      
        }
    }

// Callback function for IMU subscriber
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg, filter_context_s **context) {
        process_IMU_measurement(context, *msg);
    }

    // Callback function for VICON subscriber
    void viconCallback(const geometry_msgs::PoseStamped::ConstPtr& msg, filter_context_s **context) {
        process_VICON_measurement(context, *msg);
    }

    void publishFullPose(ros::Publisher &pub, filter_context_s **context) {
      omni_firmware::FullPose full_pose;
      full_pose.pose.position.x = (*context)->state.p()(0);
      full_pose.pose.position.y = (*context)->state.p()(1);
      full_pose.pose.position.z = (*context)->state.p()(2);
      full_pose.pose.orientation.x = (*context)->state.q()(0);
      full_pose.pose.orientation.y = (*context)->state.q()(1);
      full_pose.pose.orientation.z = (*context)->state.q()(2);
      full_pose.pose.orientation.w = (*context)->state.q()(3);
      
      full_pose.vel.linear.x = (*context)->state.v()(0);
      full_pose.vel.linear.y = (*context)->state.v()(1);
      full_pose.vel.linear.z = (*context)->state.v()(2);
      full_pose.vel.angular.x = (*context)->state.w()(0);
      full_pose.vel.angular.y = (*context)->state.w()(1);
      full_pose.vel.angular.z = (*context)->state.w()(2);


      full_pose.acc.linear.x = (*context)->state.a()(0);
      full_pose.acc.linear.y = (*context)->state.a()(1);
      full_pose.acc.linear.z = (*context)->state.a()(2);

      full_pose.header.stamp = (*context)->filter_time;
      full_pose.header.frame_id = "UKF_filter";
      full_pose.header.seq = seqID;
      seqID++;
      pub.publish(full_pose);
}

bool zero_IMU_orientation(filter_context_s **context){
  
  
    size_t num_elements = 1000;
    if ((*context)->IMU_orientation_history.size() < num_elements) {
      ROS_INFO("not enough IMU measurements");
      return false;
      }

    
    auto start_it = (*context)->IMU_orientation_history.size() - num_elements;
    // Start with the first quaternion
    Eigen::Quaternion<double> avg_q = (*context)->IMU_orientation_history[start_it];
    
    // Average the quaternions
    for (size_t i = start_it; i < (*context)->IMU_orientation_history.size(); ++i) {
        avg_q = avg_q.slerp(1.0 / (i + 1), (*context)->IMU_orientation_history[i]);
    }
    
    (*context)->bias_q_imu = avg_q.normalized();
     
    (*context)->IMU_orientation_history.clear();
    return true;
}
bool zero_VICON_orientation(filter_context_s **context){
  
  
    size_t num_elements = 200;
    if ((*context)->VICON_orientation_history.size() < num_elements) {
      ROS_INFO("not enough vicon measurements");
      return false;
      }

    
    auto start_it = (*context)->VICON_orientation_history.size() - num_elements;
    // Start with the first quaternion
    Eigen::Quaternion<double> avg_q = (*context)->VICON_orientation_history[start_it];
    
    // Average the quaternions
    for (size_t i = start_it; i < (*context)->VICON_orientation_history.size(); ++i) {
        avg_q = avg_q.slerp(1.0 / (i + 1), (*context)->VICON_orientation_history[i]);
    }

    avg_q.normalized() = avg_q.normalized();
    // Convert the averaged quaternion to Euler angles (roll, pitch, yaw)
    Eigen::Vector3d euler_angles = avg_q.toRotationMatrix().eulerAngles(0, 1, 2);

    
    // Convert back to quaternion (roll, pitch, yaw=0)
    Eigen::Quaterniond bias_q, zero_q;
    bias_q = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX()) *  // roll
             Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) *  // pitch
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());  // yaw = 0
    bias_q = bias_q.normalized();
    zero_q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()) *  // roll = 0
             Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *  // pitch = 0
             Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());  // yaw
    zero_q = zero_q.normalized();

    (*context)->bias_q_vicon = bias_q;
    (*context)->zero_q_vicon = zero_q;
    std::cout << zero_q.x() << " " <<
                 zero_q.y() << " "<<
                  zero_q.z() << " " << zero_q.w() << " "
                  << std::endl;
    //std::cout << (*context)->bias_q_vicon.x() << " " 
    //          << (*context)->bias_q_vicon.y() << " " 
    //          << (*context)->bias_q_vicon.z() << " " 
    //          << (*context)->bias_q_vicon.w() << std::endl;
    
    (*context)->VICON_orientation_history.clear();
    return true;
}

};

class sen_instance {
public:
  sen_instance(ros::NodeHandle& nh): node_handle(nh){
  // Create an instance of the filter context
  
  context.state = ukf::state_s::Identity();
  context.cov = ukf::state_s::dcov_s::Identity() * 2e-2;
  context.filter.set(context.state, context.cov);
  context.filter.predict(0.);  
  // IMU subscriber
  imu_sub = nh.subscribe<sensor_msgs::Imu>(
        "/Raw_IMU", 5, boost::bind(sen::imuCallback, _1, &context_ptr));

  // VICON subscriber
  vicon_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "/vrpn_client_node/Omnicopter/pose", 1, boost::bind(sen::viconCallback, _1, &context_ptr));

  // Publisher for the current filter state
  state_pub = nh.advertise< omni_firmware::FullPose > ("/pose_full_filtered", 20);

  // Service for to zero ukf
  service = nh.advertiseService("zero_ukf", &sen_instance::zero_ukf,this);

  context.filter_time = ros::Time::now();
  context.bias_q_imu = Eigen::Quaternion<double>::Identity();
  context.bias_q_vicon = Eigen::Quaternion<double>::Identity();
  }

  bool zero_ukf(omni_firmware::ZeroUKF::Request & req,
              omni_firmware::ZeroUKF::Response & res){
              bool zero_bool =  sen::zero_IMU_orientation(&context_ptr) && sen::zero_VICON_orientation(&context_ptr);
              //bool zero_bool =  sen::zero_VICON_orientation(&context_ptr);

              if (zero_bool){
                context.state = ukf::state_s::Identity();
                context.state.q() = context.zero_q_vicon.coeffs();
                context.cov = ukf::state_s::dcov_s::Identity() * 2e-2;
                context.filter.set(context.state, context.cov);
                context.filter.predict(0.); 
                context.filter_time = ros::Time::now();
              }
              res.success = zero_bool;
              return zero_bool;
  }

  void publishCurrentPose(){
    sen::publishFullPose(state_pub, &context_ptr);
  }

  sen::filter_context_s context;
  sen::filter_context_s* context_ptr = &context;
private:
  ros::NodeHandle& node_handle;
  ros::Subscriber imu_sub;
  ros::Subscriber vicon_sub;
  ros::Publisher state_pub;
  ros::ServiceServer service;
};


int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "state_estimation_node");
    ros::NodeHandle nh;

    // Create an instance of the filter context
    sen_instance UKF_filter_instance(nh);

  
    // Loop rate for publishing the state at 500 Hz
    ros::Rate rate(500); // 500 Hz

    int i = 0;
    // Main loop
    while (ros::ok()) {
        // Handle callbacks (IMU and VICON measurements)
        ros::spinOnce();
        
        ros::Time current_time = ros::Time::now();
        //ros::Duration time_diff = current_time - context.filter_time; 
 
        //if (i==1500) std::cout << sen::zero_orientation(&context_ptr) << std::endl;
  
 
        // Publish the current state
        //sen::publishFullPose(state_pub, &context_ptr);
        UKF_filter_instance.publishCurrentPose();
        i++;
        // Sleep to maintain 500 Hz
        rate.sleep();
    }

    return 0;
}
    