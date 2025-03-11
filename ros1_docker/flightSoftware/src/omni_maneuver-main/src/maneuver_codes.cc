#include <cmath>

#include <libkdtp/libkdtp.h>

#include <tf/tf.h>

#include "maneuver.hpp"

#include <omni_firmware/FullPose.h>

// TODO: add jerk and snap to trajectory pose

/*
 * --- go to a specific position and yaw -----------------------------------------------------
 */
bool mv_rotateTo(const Maneuver *maneuver, const omni_firmware::FullPose Pose, double roll, double pitch, double yaw, double duration, Maneuver::pose_sequence *path, const bool moving) {
	bool status;

	kdtp::State from(maneuver->robot);
	kdtp::State to(maneuver->robot);
	if (!moving){
		from.position()[0] = Pose.pose.position.x;
		from.position()[1] = Pose.pose.position.y;
		from.position()[2] = Pose.pose.position.z;

		tf::Quaternion q(Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z,Pose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = 0;//Pose.vel.linear.x;
		from.velocity()[1] = 0;//Pose.vel.linear.y;
		from.velocity()[2] = 0;//Pose.vel.linear.z;
		from.velocity()[3] = 0;//Pose.vel.angular.x;
		from.velocity()[4] = 0;//Pose.vel.angular.y;
		from.velocity()[5] = 0;//Pose.vel.angular.z;

		from.acceleration()[0] = 0;//Pose.acc.linear.x;
		from.acceleration()[1] = 0;//Pose.acc.linear.y;
		from.acceleration()[2] = 0;//Pose.acc.linear.z;
		from.acceleration()[3] = 0;//Pose.acc.angular.x;
		from.acceleration()[4] = 0;//Pose.acc.angular.y;
		from.acceleration()[5] = 0;//Pose.acc.angular.z;
	}
	else
	{
		omni_firmware::FullPose lastPose = path->_buffer.back();
		from.position()[0] = lastPose.pose.position.x;
		from.position()[1] = lastPose.pose.position.y;
		from.position()[2] = lastPose.pose.position.z;

		tf::Quaternion q(lastPose.pose.orientation.x,lastPose.pose.orientation.y,lastPose.pose.orientation.z,lastPose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = lastPose.vel.linear.x;
		from.velocity()[1] = lastPose.vel.linear.y;
		from.velocity()[2] = lastPose.vel.linear.z;
		from.velocity()[3] = lastPose.vel.angular.x;
		from.velocity()[4] = lastPose.vel.angular.y;
		from.velocity()[5] = lastPose.vel.angular.z;

		from.acceleration()[0] = lastPose.acc.linear.x;
		from.acceleration()[1] = lastPose.acc.linear.y;
		from.acceleration()[2] = lastPose.acc.linear.z;
		from.acceleration()[3] = lastPose.acc.angular.x;
		from.acceleration()[4] = lastPose.acc.angular.y;
		from.acceleration()[5] = lastPose.acc.angular.z;
	}
	to.position() = from.position();

	to.position()[3] = roll;
	to.position()[4] = pitch;
	to.position()[5] = yaw;

	to.velocity()[0] = 0.;
	to.velocity()[1] = 0.;
	to.velocity()[2] = 0.;
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = 0.;
	to.acceleration()[1] = 0.;
	to.acceleration()[2] = 0.;
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;

	//ROS_ERROR("P:[%f, %f, %f, %f], [%f, %f, %f, %f]", Pose.pose.position.x, Pose.pose.position.y, Pose.pose.position.z, pose_yaw, x, y, z, yaw);
	
	kdtp::LocalPath lpath(maneuver->robot, from, to, duration);
	
	status = mv_check_duration(lpath, duration, 1000.0 / maneuver->period);
	
	if(status == false) {
		ROS_ERROR("path duration error");
		return false;
	}
	if(moving)
	{
		status = mv_sample_path_conc(lpath, path, 1000.0 / maneuver->period);
	}
	else
	{
		status = mv_sample_path(lpath, path, 1000.0 / maneuver->period);
	}
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	return status;
}



/*
 * --- go to a specific position and yaw -----------------------------------------------------
 */
bool mv_goto(const Maneuver *maneuver, const omni_firmware::FullPose Pose, double x, double y, double z, double yaw, double duration, Maneuver::pose_sequence *path, const bool moving) {
	bool status;

	kdtp::State from(maneuver->robot);
	kdtp::State to(maneuver->robot);

	if (!moving){
		from.position()[0] = Pose.pose.position.x;
		from.position()[1] = Pose.pose.position.y;
		from.position()[2] = Pose.pose.position.z;

		tf::Quaternion q(Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z,Pose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = 0;//Pose.vel.linear.x;
		from.velocity()[1] = 0;//Pose.vel.linear.y;
		from.velocity()[2] = 0;//Pose.vel.linear.z;
		from.velocity()[3] = 0;//Pose.vel.angular.x;
		from.velocity()[4] = 0;//Pose.vel.angular.y;
		from.velocity()[5] = 0;//Pose.vel.angular.z;

		from.acceleration()[0] = 0;//Pose.acc.linear.x;
		from.acceleration()[1] = 0;//Pose.acc.linear.y;
		from.acceleration()[2] = 0;//Pose.acc.linear.z;
		from.acceleration()[3] = 0;//Pose.acc.angular.x;
		from.acceleration()[4] = 0;//Pose.acc.angular.y;
		from.acceleration()[5] = 0;//Pose.acc.angular.z;
	}
	else
	{
		omni_firmware::FullPose lastPose = path->_buffer.back();
		from.position()[0] = lastPose.pose.position.x;
		from.position()[1] = lastPose.pose.position.y;
		from.position()[2] = lastPose.pose.position.z;

		tf::Quaternion q(lastPose.pose.orientation.x,lastPose.pose.orientation.y,lastPose.pose.orientation.z,lastPose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = lastPose.vel.linear.x;
		from.velocity()[1] = lastPose.vel.linear.y;
		from.velocity()[2] = lastPose.vel.linear.z;
		from.velocity()[3] = lastPose.vel.angular.x;
		from.velocity()[4] = lastPose.vel.angular.y;
		from.velocity()[5] = lastPose.vel.angular.z;

		from.acceleration()[0] = lastPose.acc.linear.x;
		from.acceleration()[1] = lastPose.acc.linear.y;
		from.acceleration()[2] = lastPose.acc.linear.z;
		from.acceleration()[3] = lastPose.acc.angular.x;
		from.acceleration()[4] = lastPose.acc.angular.y;
		from.acceleration()[5] = lastPose.acc.angular.z;
	}


	to.position() = from.position();

	to.position()[0] = x;
	to.position()[1] = y;
	to.position()[2] = z;
	to.position()[3] = 0;
	to.position()[4] = 0;
	to.position()[5] = yaw;

	to.velocity()[0] = 0.;
	to.velocity()[1] = 0.;
	to.velocity()[2] = 0.;
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = 0.;
	to.acceleration()[1] = 0.;
	to.acceleration()[2] = 0.;
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;

	//ROS_ERROR("P:[%f, %f, %f, %f], [%f, %f, %f, %f]", Pose.pose.position.x, Pose.pose.position.y, Pose.pose.position.z, pose_yaw, x, y, z, yaw);
	
	kdtp::LocalPath lpath(maneuver->robot, from, to, duration);
	
	status = mv_check_duration(lpath, duration, 1000.0 / maneuver->period);
	
	if(status == false) {
		ROS_ERROR("path duration error");
		return false;
	}
	if(moving)
	{
		status = mv_sample_path_conc(lpath, path, 1000.0 / maneuver->period);
	}
	else
	{
		status = mv_sample_path(lpath, path, 1000.0 / maneuver->period);
	}
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	return status;
}

/*
 * --- go to a specific position and orientation -----------------------------------------------------
 */
bool mv_goto_6D(const Maneuver *maneuver, const omni_firmware::FullPose Pose, double x, double y, double z, 
double roll, double pitch, double yaw, double duration, Maneuver::pose_sequence *path, const bool moving) {
	bool status;

	kdtp::State from(maneuver->robot);
	kdtp::State to(maneuver->robot);

	if (!moving){
		from.position()[0] = Pose.pose.position.x;
		from.position()[1] = Pose.pose.position.y;
		from.position()[2] = Pose.pose.position.z;

		tf::Quaternion q(Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z,Pose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = 0;//Pose.vel.linear.x;
		from.velocity()[1] = 0;//Pose.vel.linear.y;
		from.velocity()[2] = 0;//Pose.vel.linear.z;
		from.velocity()[3] = 0;//Pose.vel.angular.x;
		from.velocity()[4] = 0;//Pose.vel.angular.y;
		from.velocity()[5] = 0;//Pose.vel.angular.z;

		from.acceleration()[0] = 0;//Pose.acc.linear.x;
		from.acceleration()[1] = 0;//Pose.acc.linear.y;
		from.acceleration()[2] = 0;//Pose.acc.linear.z;
		from.acceleration()[3] = 0;//Pose.acc.angular.x;
		from.acceleration()[4] = 0;//Pose.acc.angular.y;
		from.acceleration()[5] = 0;//Pose.acc.angular.z;
	}
	else
	{
		omni_firmware::FullPose lastPose = path->_buffer.back();
		from.position()[0] = lastPose.pose.position.x;
		from.position()[1] = lastPose.pose.position.y;
		from.position()[2] = lastPose.pose.position.z;

		tf::Quaternion q(lastPose.pose.orientation.x,lastPose.pose.orientation.y,lastPose.pose.orientation.z,lastPose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = lastPose.vel.linear.x;
		from.velocity()[1] = lastPose.vel.linear.y;
		from.velocity()[2] = lastPose.vel.linear.z;
		from.velocity()[3] = lastPose.vel.angular.x;
		from.velocity()[4] = lastPose.vel.angular.y;
		from.velocity()[5] = lastPose.vel.angular.z;

		from.acceleration()[0] = lastPose.acc.linear.x;
		from.acceleration()[1] = lastPose.acc.linear.y;
		from.acceleration()[2] = lastPose.acc.linear.z;
		from.acceleration()[3] = lastPose.acc.angular.x;
		from.acceleration()[4] = lastPose.acc.angular.y;
		from.acceleration()[5] = lastPose.acc.angular.z;
	}


	to.position() = from.position();

	to.position()[0] = x;
	to.position()[1] = y;
	to.position()[2] = z;
	to.position()[3] = roll;
	to.position()[4] = pitch;
	to.position()[5] = yaw;

	to.velocity()[0] = 0.;
	to.velocity()[1] = 0.;
	to.velocity()[2] = 0.;
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = 0.;
	to.acceleration()[1] = 0.;
	to.acceleration()[2] = 0.;
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;

	//ROS_ERROR("P:[%f, %f, %f, %f], [%f, %f, %f, %f]", Pose.pose.position.x, Pose.pose.position.y, Pose.pose.position.z, pose_yaw, x, y, z, yaw);
	
	kdtp::LocalPath lpath(maneuver->robot, from, to, duration);
	
	status = mv_check_duration(lpath, duration, 1000.0 / maneuver->period);
	
	if(status == false) {
		ROS_ERROR("path duration error");
		return false;
	}
	if(moving)
	{
		status = mv_sample_path_conc(lpath, path, 1000.0 / maneuver->period);
	}
	else
	{
		status = mv_sample_path(lpath, path, 1000.0 / maneuver->period);
	}
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	return status;
}

/*
 * --- Plan a 5D ellipse -----------------------------------------------------
 * --- Passes through points, P1, P2, P3, P4 ---------------------------------
 * --- Platform first goes to P1, and starts from P1 with --------------------
 * --- zero velocity and acceleration ----------------------------------------
 * --- P1: min_y, min_z, mean_x, min_roll, min_pitch -------------------------
 * --- P2: mean_y, mean_z, max_x, mean_roll, mean_pitch ----------------------
 * --- P3: max_y, max_z, mean_x, max_roll, max_pitch -------------------------
 * --- P4: mean_y, mean_z, min_x, mean_roll, mean_pitch ----------------------
 * --- Roll and Pitch change semi-linearly throughout ------------------------
 * --- velocity and acceleration at P1 is assume to be equal to zero ---------
 * --- if repass is true, the platform repasses by P1 ------------------------
 * --- with required velocity and acceleration -------------------------------
 */
bool mv_plan_5DEllipse(const Maneuver *maneuver, const omni_firmware::FullPose Pose, 
						double x_min, double x_max, 
						double y_min, double y_max, 
						double z_min, double z_max, 
						double roll_start, double roll_mid, double roll_end, 
						double pitch_start, double pitch_mid, double pitch_end,
						double yaw_d, double duration, Maneuver::pose_sequence *path, const bool moving) {
	bool status;

	kdtp::State from(maneuver->robot);
	kdtp::State to(maneuver->robot);

	from.position()[0] = Pose.pose.position.x;
	from.position()[1] = Pose.pose.position.y;
	from.position()[2] = Pose.pose.position.z;

	tf::Quaternion q(Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z,Pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	from.position()[3] = roll;
	from.position()[4] = pitch;
	from.position()[5] = yaw;

	//int n = 16; // number of points along the ellipse
	Eigen::Array3d u,v,X,V,A,X_mean; // u and v are the axes of the ellipse, X,V,A are the position/velocity/acceleration buffers
    	double omega = 2 * M_PI /duration; // angular velocity, or relation between angle and time
	double t; // angle along ellipse
	double x_mean = (x_max + x_min) / 2.0;
	double y_mean = (y_max + y_min) / 2.0;
	double z_mean = (z_max + z_min) / 2.0;
	
	bool x_axis = true;
	if (fabs(x_max - x_min) < 0.001) x_axis = false;
		
	if (x_axis){
	u << 0, y_min - y_mean, z_min - z_mean;
	v << x_max - x_mean, 0, 0;}
	else
	{
	u << x_min - x_mean, 0, z_min - z_mean;
	v << 0., y_max - y_mean, 0;}
	X_mean << x_mean, y_mean, z_mean;
	
	// move to start location
	t = 0;
	X = X_mean + u * cos(t) + v * sin(t);
	V = (-u * sin(t) + v * cos(t)) * omega;
	A = (-u * cos(t) - v * sin(t)) * omega * omega;
	
	from.velocity()[0] = 0;//Pose.vel.linear.x;
	from.velocity()[1] = 0;//Pose.vel.linear.y;
	from.velocity()[2] = 0;//Pose.vel.linear.z;
	from.velocity()[3] = 0;//Pose.vel.angular.z;
	from.velocity()[4] = 0;//Pose.vel.angular.z;
	from.velocity()[5] = 0;//Pose.vel.angular.z;

	from.acceleration()[0] = 0; //Pose.acc.linear.x;
	from.acceleration()[1] = 0; //Pose.acc.linear.y;
	from.acceleration()[2] = 0; //Pose.acc.linear.z;
	from.acceleration()[3] = 0; //Pose.acc.angular.z;
	from.acceleration()[4] = 0; //Pose.acc.angular.z;
	from.acceleration()[5] = 0; //Pose.acc.angular.z;

	to.position() = from.position();
	to.position()[0] = x_min;
   	to.position()[1] = y_min;
   	to.position()[2] = z_min;
    	to.position()[3] = roll_start;
    	to.position()[4] = pitch_start;
    	to.position()[5] = yaw;

	to.velocity()[0] = 0.;
	to.velocity()[1] = 0.;
	to.velocity()[2] = 0.;
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = 0.;
	to.acceleration()[1] = 0.;
	to.acceleration()[2] = 0.;
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;
	
	kdtp::LocalPath lpath(maneuver->robot, from, to, duration/4.0);

	status = mv_check_duration(lpath, duration/4.0, 1000.0 / maneuver->period);

	if(status == false) {
		ROS_ERROR("path duration error at 0.");
		//return false;
	}
	status = mv_sample_path(lpath, path, 1000.0 / maneuver->period);
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	
	// move to first point on circle
	from.position() = to.position();
	from.velocity() = to.velocity();
	from.acceleration() = to.acceleration();
   	to.position()[0] = X(0);
   	to.position()[1] = X(1);
   	to.position()[2] = X(2);
    	to.position()[3] = roll_start;
    	to.position()[4] = pitch_start;
    	to.position()[5] = yaw;

	to.velocity()[0] = V(0);
	to.velocity()[1] = V(1);
	to.velocity()[2] = V(2);
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = A(0);
	to.acceleration()[1] = A(1);
	to.acceleration()[2] = A(2);
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;
	//kdtp::LocalPath lpath(maneuver->robot, from, to, duration/4.0);

	//status = mv_check_duration(lpath, duration/4.0, 1000.0 / maneuver->period);

	kdtp::LocalPath lpath_2(maneuver->robot, from, to, duration/4.0);

	status = mv_check_duration(lpath_2, duration/4.0, 1000.0 / maneuver->period);

	if(status == false) {
		ROS_ERROR("path duration error");
		//return false;
	}
	status = mv_sample_path_conc(lpath_2, path, 1000.0 / maneuver->period);
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	
	
	
	// execute ellipse
	status = mv_sample_5DEllipse_path(path, 1000.0 / maneuver->period, u, v, X_mean, roll_start, roll_mid, roll_end, pitch_start, pitch_mid, pitch_end, yaw, duration);
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	
	// move away from ellipse
	from.position() = to.position();
	from.velocity() = to.velocity();
	from.acceleration() = to.acceleration();
	if (x_axis){
	to.position()[0] = x_max;
   	to.position()[1] = y_min;}
   	else
   	{
	to.position()[0] = x_min;
   	to.position()[1] = y_max;}
   	to.position()[2] = z_min;
    	to.position()[3] = roll_end;
    	to.position()[4] = pitch_end;
    	to.position()[5] = yaw;

	to.velocity()[0] = 0.;
	to.velocity()[1] = 0.;
	to.velocity()[2] = 0.;
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = 0.;
	to.acceleration()[1] = 0.;
	to.acceleration()[2] = 0.;
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;
	
	kdtp::LocalPath lpath_3(maneuver->robot, from, to, duration/4.0);

	status = mv_check_duration(lpath_3, duration/4.0, 1000.0 / maneuver->period);

	if(status == false) {
		ROS_ERROR("path duration error");
		//return false;
	}
	status = mv_sample_path_conc(lpath_3, path, 1000.0 / maneuver->period);
	if(status == false) {
		ROS_ERROR("path construction error");
	}

	return status;
}


/*
 * --- full flip around roll or pitch ----------------------------------------------------
 */
bool mv_plan_flip(const Maneuver *maneuver, const omni_firmware::FullPose Pose, bool roll_bool, bool pitch_bool, double duration, Maneuver::pose_sequence *path) {
	bool status;
	Eigen::Array3d X;
	X(0) = Pose.pose.position.x;
	X(1) = Pose.pose.position.y;
	X(2) = Pose.pose.position.z;

	tf::Quaternion q(Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z,Pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	status = mv_sample_full_flip_path(path, 1000.0 / maneuver->period, 
	X, yaw, roll_bool, pitch_bool, duration);

	return status;
}

/*
 * --- take off path planning -----------------------------------------------------
 */
bool mv_plan_take_off(const Maneuver *maneuver, const omni_firmware::FullPose Pose, double height, double duration, Maneuver::pose_sequence *path) {
	bool status;

	kdtp::State from(maneuver->robot);
	kdtp::State to(maneuver->robot);

	from.position()[0] = Pose.pose.position.x;
	from.position()[1] = Pose.pose.position.y;
	from.position()[2] = Pose.pose.position.z;

	tf::Quaternion q(Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z,Pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
	from.position()[3] = roll;
	from.position()[4] = pitch;
	from.position()[5] = yaw;

	from.velocity()[0] = 0;//Pose.vel.linear.x;
	from.velocity()[1] = 0;//Pose.vel.linear.y;
	from.velocity()[2] = 0;//Pose.vel.linear.z;
	from.velocity()[3] = 0;//Pose.vel.angular.z;
	from.velocity()[4] = 0;//Pose.vel.angular.z;
	from.velocity()[5] = 0;//Pose.vel.angular.z;

	from.acceleration()[0] = 0; //Pose.acc.linear.x;
	from.acceleration()[1] = 0; //Pose.acc.linear.y;
	from.acceleration()[2] = 0; //Pose.acc.linear.z;
	from.acceleration()[3] = 0; //Pose.acc.angular.z;
	from.acceleration()[4] = 0; //Pose.acc.angular.z;
	from.acceleration()[5] = 0; //Pose.acc.angular.z;

	to.position() = from.position();
    
    to.position()[3] = 0;
    to.position()[4] = 0;

	to.velocity()[0] = 0.;
	to.velocity()[1] = 0.;
	to.velocity()[2] = 0.;
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = 0.;
	to.acceleration()[1] = 0.;
	to.acceleration()[2] = 0.;
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;
	to.position()[2] = height;

	kdtp::LocalPath lpath(maneuver->robot, from, to, duration);

	status = mv_check_duration(lpath, duration, 1000.0 / maneuver->period);

	if(status == false) {
		ROS_ERROR("path duration error");
		//return false;
	}
	status = mv_sample_path(lpath, path, 1000.0 / maneuver->period);
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	return status;
}

bool mv_plan_start(Maneuver *maneuver) {
	static const double vmax = 3.;
	static const double amax = 2.;
	static const double jmax = 2.;
	static const double wmax = 1.;

	maneuver->robot.addDof(kdtp::Dof(-10, 10, vmax, amax, jmax, 50 * jmax, false));
	maneuver->robot.addDof(kdtp::Dof(-10, 10, vmax, amax, jmax, 50 * jmax, false));
	maneuver->robot.addDof(kdtp::Dof(0., 5., vmax, amax, jmax, 50 * jmax, false));
	maneuver->robot.addDof(kdtp::Dof(-3 * M_PI, 3 * M_PI, wmax, 10 * wmax, 100 * wmax, 1000 * wmax, true));
	maneuver->robot.addDof(kdtp::Dof(-3 * M_PI, 3 * M_PI, wmax, 10 * wmax, 100 * wmax, 1000 * wmax, true));
	maneuver->robot.addDof(kdtp::Dof(-3 * M_PI, 3 * M_PI, wmax, 10 * wmax, 100 * wmax, 1000 * wmax, true));

	return true;
}

bool mv_plan_land(const Maneuver *maneuver, const omni_firmware::FullPose Pose, double height_1, double duration_1, double height_2, double duration_2, Maneuver::pose_sequence *path, const bool moving) {
	bool status;

	kdtp::State from(maneuver->robot);
	kdtp::State to(maneuver->robot);

	if (!moving){
		from.position()[0] = Pose.pose.position.x;
		from.position()[1] = Pose.pose.position.y;
		from.position()[2] = Pose.pose.position.z;

		tf::Quaternion q(Pose.pose.orientation.x,Pose.pose.orientation.y,Pose.pose.orientation.z,Pose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = 0;//Pose.vel.linear.x;
		from.velocity()[1] = 0;//Pose.vel.linear.y;
		from.velocity()[2] = 0;//Pose.vel.linear.z;
		from.velocity()[3] = 0;//Pose.vel.angular.x;
		from.velocity()[4] = 0;//Pose.vel.angular.y;
		from.velocity()[5] = 0;//Pose.vel.angular.z;

		from.acceleration()[0] = 0;//Pose.acc.linear.x;
		from.acceleration()[1] = 0;//Pose.acc.linear.y;
		from.acceleration()[2] = 0;//Pose.acc.linear.z;
		from.acceleration()[3] = 0;//Pose.acc.angular.x;
		from.acceleration()[4] = 0;//Pose.acc.angular.y;
		from.acceleration()[5] = 0;//Pose.acc.angular.z;
	}
	else
	{
		omni_firmware::FullPose lastPose = path->_buffer.back();
		from.position()[0] = lastPose.pose.position.x;
		from.position()[1] = lastPose.pose.position.y;
		from.position()[2] = lastPose.pose.position.z;

		tf::Quaternion q(lastPose.pose.orientation.x,lastPose.pose.orientation.y,lastPose.pose.orientation.z,lastPose.pose.orientation.w);
	    	tf::Matrix3x3 m(q);
	    	double roll_, pitch_, yaw_;
	    	m.getRPY(roll_, pitch_, yaw_);
		from.position()[3] = roll_;
		from.position()[4] = pitch_;
		from.position()[5] = yaw_;

		from.velocity()[0] = lastPose.vel.linear.x;
		from.velocity()[1] = lastPose.vel.linear.y;
		from.velocity()[2] = lastPose.vel.linear.z;
		from.velocity()[3] = lastPose.vel.angular.x;
		from.velocity()[4] = lastPose.vel.angular.y;
		from.velocity()[5] = lastPose.vel.angular.z;

		from.acceleration()[0] = lastPose.acc.linear.x;
		from.acceleration()[1] = lastPose.acc.linear.y;
		from.acceleration()[2] = lastPose.acc.linear.z;
		from.acceleration()[3] = lastPose.acc.angular.x;
		from.acceleration()[4] = lastPose.acc.angular.y;
		from.acceleration()[5] = lastPose.acc.angular.z;
	}


	to.position() = from.position();
   	to.position()[2] = height_1;
    to.position()[3] = 0;
    to.position()[4] = 0;

	to.velocity()[0] = 0.;
	to.velocity()[1] = 0.;
	to.velocity()[2] = 0.;
	to.velocity()[3] = 0.;
	to.velocity()[4] = 0.;
	to.velocity()[5] = 0.;

	to.acceleration()[0] = 0.;
	to.acceleration()[1] = 0.;
	to.acceleration()[2] = 0.;
	to.acceleration()[3] = 0.;
	to.acceleration()[4] = 0.;
	to.acceleration()[5] = 0.;

	kdtp::LocalPath lpath(maneuver->robot, from, to, duration_1);

	status = mv_check_duration(lpath, duration_1, 1000.0 / maneuver->period);

	if(status == false) {
		ROS_ERROR("path duration error");
		//return false;
	}
	if(moving)
	{
		status = mv_sample_path_conc(lpath, path, 1000.0 / maneuver->period);
	}
	else
	{
		status = mv_sample_path(lpath, path, 1000.0 / maneuver->period);
	}
	if(status == false) {
		ROS_ERROR("path construction error");
	}
	from.position() =  to.position();
	to.position()[2] = height_2;
	kdtp::LocalPath lpath_2(maneuver->robot, from, to, duration_2);

	status = mv_check_duration(lpath_2, duration_2, 1000.0 / maneuver->period);

	if(status == false) {
		ROS_ERROR("path duration error");
		//return false;
	}
	status = mv_sample_path_conc(lpath_2, path, 1000.0 / maneuver->period);
	if(status == false) {
		ROS_ERROR("path construction error");
	}

	return status;
}
