#include <cmath>

#include <libkdtp/libkdtp.h>

#include "maneuver.hpp"

#include <omni_firmware/FullPose.h>


bool mv_check_duration(const kdtp::LocalPath &p, const double duration, const int maneuver_control_period_ms) {
	/* consider limits */
	if(duration > 0. && p.duration() > duration + maneuver_control_period_ms / 1000.) {
		static const double dt = maneuver_control_period_ms / 1000.;
		ROS_ERROR("error: %f, %f", (double) p.duration(), 2 + p.duration() / dt);
		return false;
	}

	return true;
}

/*
 * --- sample local path according to the exec task period -----------------
 */
bool mv_sample_path(const kdtp::LocalPath &p, Maneuver::pose_sequence *path, const int maneuver_control_period_ms) {
	static const double dt = maneuver_control_period_ms / 1000.;

	omni_firmware::FullPose s;
	double q[6][5];
	size_t i;

	/* set path length, including start & end configurations */
	i = 2 + p.duration() / dt;
	if(path->_maximum < i) //|| path->_maximum > 2 * i)
	{
		ROS_ERROR("discretized path larger than allowable");
		return false;
	}
	path->_length = i;

	path->_buffer.clear();
	/* interpolate path */
	for(i = 0; i < path->_length; i++) {
		p.getAllAt(i * dt, q);

		s.pose.position.x = q[0][0];
		s.pose.position.y = q[1][0];
		s.pose.position.z = q[2][0];
        const float cosRoll = cos(q[3][0]*0.5f);
        const float sinRoll = sin(q[3][0]*0.5f);

        const float cosPitch = cos(q[4][0]*0.5f);
        const float sinPitch = sin(q[4][0]*0.5f);

        const float cosYaw = cos(q[5][0]*0.5f);
        const float sinYaw = sin(q[5][0]*0.5f);


        s.pose.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
		s.pose.orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
        s.pose.orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
        s.pose.orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

		s.vel.linear.x = q[0][1];
		s.vel.linear.y = q[1][1];
		s.vel.linear.z = q[2][1];
		s.vel.angular.x = q[3][1];
		s.vel.angular.y = q[4][1];
		s.vel.angular.z = q[5][1];

		s.acc.linear.x = q[0][2];
		s.acc.linear.y = q[1][2];
		s.acc.linear.z = q[2][2];
		s.acc.angular.x = q[3][2];
		s.acc.angular.y = q[4][2];
		s.acc.angular.z = q[5][2];

		path->_buffer.push_back(s);
	}
	return true;
}

/*
 * --- sample local path according to the exec task period & Conc with previous -----------------
 * ----------------------- call after calling mv_sample_path ---------------------
 */
bool mv_sample_path_conc(const kdtp::LocalPath &p, Maneuver::pose_sequence *path, const int maneuver_control_period_ms) {
	static const double dt = maneuver_control_period_ms / 1000.;

	omni_firmware::FullPose s;
	double q[6][5];
	size_t i,L;

	/* set path length, including start & end configurations */
	L = 1 + p.duration() / dt;
	if(path->_maximum < L) //|| path->_maximum > 2 * i)
	{
		ROS_ERROR("discretized path larger than allowable");
		return false;
	}
	int L_old = path->_length;
	path->_length = L + path->_length;

	/* interpolate path */
	for(i = L_old; i < path->_length; i++) {
		if (i == 0) continue;
		p.getAllAt((i-L_old) * dt, q);

		s.pose.position.x = q[0][0];
		s.pose.position.y = q[1][0];
		s.pose.position.z = q[2][0];
        const float cosRoll = cos(q[3][0]*0.5f);
        const float sinRoll = sin(q[3][0]*0.5f);

        const float cosPitch = cos(q[4][0]*0.5f);
        const float sinPitch = sin(q[4][0]*0.5f);

        const float cosYaw = cos(q[5][0]*0.5f);
        const float sinYaw = sin(q[5][0]*0.5f);


        s.pose.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
		s.pose.orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
        s.pose.orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
        s.pose.orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

		s.vel.linear.x = q[0][1];
		s.vel.linear.y = q[1][1];
		s.vel.linear.z = q[2][1];
		s.vel.angular.x = q[3][1];
		s.vel.angular.y = q[4][1];
		s.vel.angular.z = q[5][1];

		s.acc.linear.x = q[0][2];
		s.acc.linear.y = q[1][2];
		s.acc.linear.z = q[2][2];
		s.acc.angular.x = q[3][2];
		s.acc.angular.y = q[4][2];
		s.acc.angular.z = q[5][2];

		path->_buffer.push_back(s);
	}
	return true;
}


/*
 * --- sample full flip path -----------------
 */
bool mv_sample_full_flip_path(Maneuver::pose_sequence *path, const int maneuver_control_period_ms, 
const Eigen::Array3d X, const double yaw, const bool roll_bool, const bool pitch_bool,
const double T) {

double w_roll, w_pitch;
if (roll_bool)
	w_roll = 2*M_PI/T;
else
	w_roll = 0.0;
if (pitch_bool)
	w_pitch = 2*M_PI/T;
else
	w_pitch = 0.0;

static const double dt = maneuver_control_period_ms / 1000.;

omni_firmware::FullPose s;
size_t i, L;
double t;	
L = 1 + ceil (T / dt);
path->_length = L + 1;
double roll,pitch;
path->_buffer.clear();
for(i = 0; i <= L; i++) {
	t = i*dt;
	
	if (i==L){
	 	roll = 0.0;
	 	pitch = 0.0;
	}
	else{
		roll = t*w_roll;
		pitch = t*w_pitch;
	}
	s.pose.position.x = X(0);
	s.pose.position.y = X(1);
	s.pose.position.z = X(2);
        const float cosRoll = cos(roll*0.5f);
        const float sinRoll = sin(roll*0.5f);

        const float cosPitch = cos(pitch*0.5f);
        const float sinPitch = sin(pitch*0.5f);

        const float cosYaw = cos(yaw*0.5f);
        const float sinYaw = sin(yaw*0.5f);


        s.pose.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
	s.pose.orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
        s.pose.orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
        s.pose.orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

	s.vel.linear.x = 0.0;
	s.vel.linear.y = 0.0;
	s.vel.linear.z = 0.0;
	if (i == L){
		s.vel.angular.x = 0.0;
		s.vel.angular.y = 0.0;
	}
	else{
		s.vel.angular.x = w_roll;
		s.vel.angular.y = w_pitch;
	}
	s.vel.angular.z = 0.0;

	s.acc.linear.x = 0.0;
	s.acc.linear.y = 0.0;
	s.acc.linear.z = 0.0;
	s.acc.angular.x = 0.0;
	s.acc.angular.y = 0.0;
	s.acc.angular.z = 0.0;

	path->_buffer.push_back(s);

}


return true;
}


/*
 * --- sample local path according to the exec task period -----------------
 */
bool mv_sample_5DEllipse_path(Maneuver::pose_sequence *path, const int maneuver_control_period_ms, 
const Eigen::Array3d u, const Eigen::Array3d v, const Eigen::Array3d X_mean, 
const double roll_start, const double roll_mid, const double roll_end,  
const double pitch_start, const double pitch_mid, const double pitch_end, const double yaw,
const double T) {
	static const double dt = maneuver_control_period_ms / 1000.;

	omni_firmware::FullPose s;
	size_t i, L;
	double t;

	/* set path length, including start & end configurations */
	L = 1 + ceil(T / dt);
	int L_old = path->_length;
	path->_length = L + path->_length;


	// roll polynomials
	double roll_v_1 = -4*(roll_start - roll_mid)/T;
	double roll_a_1 = 8*(roll_start - roll_mid)/(T*T);
	double roll_p_1 = roll_start;

	double roll_v_2 = -4*(roll_end - roll_mid)/T;
	double roll_a_2 = 8*(roll_end - roll_mid)/(T*T);
	double roll_p_2 = roll_end;
	// pitch polynomials
	double pitch_v_1 = -4*(pitch_start - pitch_mid)/T;
	double pitch_a_1 = 8*(pitch_start - pitch_mid)/(T*T);
	double pitch_p_1 = pitch_start;

	double pitch_v_2 = -4*(pitch_end - pitch_mid)/T;
	double pitch_a_2 = 8*(pitch_end - pitch_mid)/(T*T);
	double pitch_p_2 = pitch_end;

	double roll, roll_dot, roll_ddot;
	double pitch, pitch_dot, pitch_ddot;
	
	Eigen::Array3d X,V,A; // X,V,A are the position/velocity/acceleration buffers
	double omega = 2 * M_PI /T; // angular velocity, or relation between angle and time
	/* interpolate path */
	for(i = 0; i < L; i++) {
		t = i*dt;
		if (t > T/2)
		{
		roll = roll_p_1 + roll_v_1*t + roll_a_1*0.5*t*t;
		roll_dot = roll_v_1 + roll_a_1*t;
		roll_ddot = roll_a_1;
		pitch = pitch_p_1 + pitch_v_1*t + pitch_a_1*0.5*t*t;
		pitch_dot = pitch_v_1 + pitch_a_1*t;
		pitch_ddot = pitch_a_1;
		}
		else
		{
		roll = roll_p_2 + roll_v_2*t + roll_a_2*0.5*t*t;
		roll_dot = roll_v_2 + roll_a_2*t;
		roll_ddot = roll_a_2;
		pitch = pitch_p_2 + pitch_v_2*t + pitch_a_2*0.5*t*t;
		pitch_dot = pitch_v_2 + pitch_a_2*t;
		pitch_ddot = pitch_a_2;
		}
		t = i*M_PI*2/L;	
			
		X = X_mean + u * cos(t) + v * sin(t);
		Eigen::Array3d uc,vc;
		uc = u * std::cos(t);
		vc = v * std::sin(t);


		V = (-u * sin(t) + v * cos(t)) * omega;
		A = (-u * cos(t) - v * sin(t)) * omega * omega;
		s.pose.position.x = X(0);
		s.pose.position.y = X(1);
		s.pose.position.z = X(2);
        	const float cosRoll = cos(roll*0.5f);
        	const float sinRoll = sin(roll*0.5f);

        	const float cosPitch = cos(pitch*0.5f);
        	const float sinPitch = sin(pitch*0.5f);

        	const float cosYaw = cos(yaw*0.5f);
        	const float sinYaw = sin(yaw*0.5f);


        	s.pose.orientation.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
		s.pose.orientation.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
        	s.pose.orientation.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
        	s.pose.orientation.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;

		s.vel.linear.x = V(0);
		s.vel.linear.y = V(1);
		s.vel.linear.z = V(2);
		s.vel.angular.x = roll_dot;
		s.vel.angular.y = pitch_dot;
		s.vel.angular.z = 0.0;

		s.acc.linear.x = A(0);
		s.acc.linear.y = A(1);
		s.acc.linear.z = A(2);
		s.acc.angular.x = roll_ddot;
		s.acc.angular.y = pitch_ddot;
		s.acc.angular.z = 0.0;

		path->_buffer.push_back(s);
	}
	return true;
}

