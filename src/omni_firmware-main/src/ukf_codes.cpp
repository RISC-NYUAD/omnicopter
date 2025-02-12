#include <ros/ros.h>
#include <vector>
#include <cmath> 
#include <Eigen/Dense>
#include <Eigen/Core>

class UKF {

public:
	UKF() {
	// Initialize state vector (16 states: [position (3), velocity (3), orientation (quaternion 4), acceleration (3), angular velocity (3)])
        state = Eigen::VectorXd::Zero(16);

        // Initialize covariance matrix (16x16)
        covariance = Eigen::MatrixXd::Identity(16, 16);
	
	// Gravity vector (world frame)
        gravity = Eigen::Vector3d(0, 0, -9.81);
        
	}

	/* --- UKF::transition ------------------------------------------------- */

	Eigen::VectorXd UKF::transition(double dt){
		// Get old orientation to be used in transformations
		Eigen::Quaterniond orientation(state(9), state(6), state(7), state(8));  // Orientation (qx, qy, qz, qw)
		// initialize new state
		Eigen::VectorXd state_new = Eigen::VectorXd::Zero(16);
		
		//correct acceleration to world frame and remove gravity
        	// Transform IMU acceleration to global frame
        	Eigen::Vector3d accel_world = orientation * imu_linear_acceleration;

        	// Subtract gravity
        	accel_world -= gravity;
        	
        	// set new acceleration
        	state_new.segment<3>(10) = accel_world;
        	//set new velocity
        	state_new.segment<3>(3) = state.segment<3>(3) + accel_world * dt;
        	//set new position
        	state_new.segment<3>(0) = state.segment<3>(0) + state.segment<3>(3) * dt + 0.5* accel_world * dt * dt; 
        	
        	//set new angular velocity
        	state_new.segment<3>(13) = imu_angular_velocity;
        	// compute attitude
        	Eigen::Quaternion<double> omega_q, q_new;
        	double a2 = dt * dt * (imu_angular_velocity).squaredNorm();
  		if (a2 < 1e-1) {
    		omega_q.w() = 1 - a2/8 /*std::cos(a/2)*/;
    		omega_q.vec() = (0.5 - a2/48 /*std::sin(a/2)/a*/) * dt * imu_angular_velocity;
  		} else {
    		double a = std::sqrt(a2);
    		omega_q.w() = std::cos(a/2);
    		omega_q.vec() = std::sin(a/2)/a * dt * imu_angular_velocity;
  		}
  		q_new = (omega_q * Eigen::Quaternion<double>(in.q())).coeffs();
  		// set new attitude
  		state_new.segment<3>(6) = q_new.vec();
  		state_new.segment<1>(9) = q_new.w();
  		return state_new;
	}
	
	/* --- UKF::process_noise ---------------------------------------------- */
	// dconv has format has format [dpx, dpy, dpz, dvx, dvy, dvz, dqx, dqy, dqz, dax, day, daz, dwx, dwy, dwz]^2
	
	double state_s::max_dadt = 200.;
	double state_s::max_dwdt = 50.;

	Eigen::MatrixXd
	UKF::process_noise(double dt) {
  		double astddev = dt*max_dadt/3.;
  		double wstddev = dt*max_dwdt/3.;

  		double avar = astddev * astddev;
  		double wvar = wstddev * wstddev;

  		double avar_dt = avar * dt;
  		double avar_dt2 = avar_dt * dt;
  		double avar_dt3 = avar_dt2 * dt;
  		double avar_dt4 = avar_dt3 * dt;

  		double wvar_dt = wvar * dt;
  		double wvar_dt2 = wvar_dt * dt;

  		Eigen::MatrixXd n;
  		n.setZero(this->processNoise.rows(), this->processNoise.cols());

  		n.block<3,3>(0,0).diagonal() <<
    		0.25 * avar_dt4, 0.25 * avar_dt4, 0.25 * avar_dt4;
  		n.block<3,3>(3,0).diagonal() <<
    		0.5 * avar_dt3, 0.5 * avar_dt3, 0.5 * avar_dt3;
  		n.block<3,3>(0,3).diagonal() <<
    		0.5 * avar_dt3, 0.5 * avar_dt3, 0.5 * avar_dt3;
  		n.block<3,3>(9,0).diagonal() <<
    		0.5 * avar_dt2, 0.5 * avar_dt2, 0.5 * avar_dt2;
  		n.block<3,3>(0,9).diagonal() <<
    		0.5 * avar_dt2, 0.5 * avar_dt2, 0.5 * avar_dt2;
  		n.block<3,3>(3,3).diagonal() <<
    		avar_dt2, avar_dt2, avar_dt2;
  		n.block<3,3>(9,3).diagonal() <<
    		avar_dt, avar_dt, avar_dt;
  		n.block<3,3>(3,9).diagonal() <<
    		avar_dt, avar_dt, avar_dt;
  		n.block<3,3>(9,9).diagonal() <<
    		avar, avar, avar;

  		n.block<4,4>(6,6).diagonal() <<
    		wvar_dt2, wvar_dt2, wvar_dt2 wvar_dt2;
  		n.block<3,3>(6,12).diagonal() <<
    		wvar_dt, wvar_dt, wvar_dt;
  		n.block<3,3>(12,6).diagonal() <<
    		wvar_dt, wvar_dt, wvar_dt;
  		n.block<3,3>(12,12).diagonal() <<
    		wvar, wvar, wvar;

  		return n;
	}
	
	/* --- UKF::delta ------------------------------------------------------ */
	// delta has format [dpx, dpy, dpz, dvx, dvy, dvz, dqx, dqy, dqz, dax, day, daz, dwx, dwy, dwz]
	Eigen::VectorXd
	UKF::delta(const Eigen::VectorXd &s2, const Eigen::VectorXd &s1)
	{
  		delta_s ds;
		
		Eigen::Quaternion q1(s1(9),s1(6),s1(7),s1(8)),q2(s2(9),s2(6),s2(7),s2(8);
		
  		Eigen::Quaternion<double> dq =
    		q2 *
    		q1.conjugate();

  		/* compute the error quaternion. There is a singularity at 2π: to avoid it,
   		* just use the opposite quaternion for any angle above π. In practice,
   		* updates with angles above π happen only during initialization. */
  		if (dq.w() >= 0)
   		 ds.segment<3>(6) = 4. * dq.vec()/(1.+dq.w());
  		else
    		ds.segment<3>(6) = 4. * -dq.vec()/(1.-dq.w());

  		ds.segment<3>(0) = s2.segment<3>(0) - s1.segment<3>(0);
  		ds.segment<3>(3) = s2.segment<3>(3) - s1.segment<3>(3);
  		ds.segment<3>(12) = s2.segment<3>(13) - s1.segment<3>(13);
  		ds.segment<3>(9) = s2.segment<3>(10) - s1.segment<3>(10);

  		return ds;
	}
	
	/* --- UKF::add_delta -------------------------------------------------- */

	Eigen::VectorXd
	UKF::add_delta(const Eigen::VectorXd &s, const Eigen::VectorXd &ds)
	{
  		Eigen::VectorXd sds;

  		Eigen::Matrix<double, 3, 1> dr = ds.segment<3>(6);
  		Eigen::Quaternion<double> dq;

  		/* compute the quaternion from the error quaternion */
  		double r2 = dr.squaredNorm();
  		dq.w() = (16. - r2) / (16. + r2);
  		dq.vec() = (1 + dq.w()) * dr / 4.;
  		
  		Eigen::Quaternion<double> q_new = (dq * Eigen::Quaternion<double>(s.q())).coeffs();
  		sds.segment<3>(6) = q_new.vec();
  		sds(9) = q_new.w();
  		
  		sds.segment<3>(0) = s.segment<3>(0) + ds.segment<3>(0);
  		sds.segment<3>(3) = s.segment<3>(3) + ds.segment<3>(3);
  		sds.segment<3>(10) = s.segment<3>(10) + ds.segment<3>(9);
  		sds.segment<3>(13) = s.segment<3>(13) + ds.segment<3>(12);

  		return sds;
	}

	/* --- UKF::predict -------------------------------------------------- */
	void UKF::predict(double dt) {
    	/* don't bother updating the state when dt is 0 */
    	if (std::fabs(dt) > 0.) {
      	for(int i = 0; i < 2*state_s::dof+1; i++)
        	sigma_points.col(i) = state_s::transition(sigma_points.col(i), dt);

      	prediction.noalias() =
        	weight_m0() * sigma_points.col(0) +
        	weight_i() * sigma_points.rightCols<2*state_s::dof>().rowwise().sum();
      	prediction.normalize();
    	}

    	for(int i = 0; i < 2*this->dof+1; i++)
      		prediction_deviation.col(i) =
        	UKF::delta(sigma_points.col(i), prediction);

    	/* don't update cov for tiny dt: in constant velocity model, the sigma
     	* point spread on the acceleration is 0 and the process noise approaches
    	* 0 when dt is too small, which leads to numerical unstability. */
    	if (std::fabs(dt) < 1e-5 /* 10 µs */) return;

    	prediction_cov.noalias() =
      		weight_c0() * (
        	prediction_deviation.col(0) * prediction_deviation.col(0).transpose()
        	);
    	for(int i = 1; i < 2*state_s::dof+1; i++)
      		prediction_cov.noalias() +=
        		weight_i() * (
          	prediction_deviation.col(i) * prediction_deviation.col(i).transpose()
          	);

    	prediction_cov += prediction.process_noise(dt);
  	}
	
	private:
    	Eigen::VectorXd state;          // State vector [px, py, pz, vx, vy, vz, qx, qy, qz, qw, ax, ay, az, wx, wy, wz]
    	Eigen::MatrixXd covariance;     // State covariance matrix
    	Eigen::MatrixXd processNoise;   // Process noise covariance matrix (16x16: velocity and angular velocity)
    	Eigen::MatrixXd measurementNoise;  // Measurement noise covariance matrix (7x7: position and orientation)

    	Eigen::Vector3d gravity;        // Gravity vector (world frame)
    	Eigen::Vector3d imu_linear_acceleration;  // IMU linear acceleration (body frame)
    	Eigen::Vector3d imu_angular_velocity;     // IMU angular velocity (body frame)
    	Eigen::Vector3d vicon_position;           // Vicon position (world frame)
    	Eigen::Quaterniond vicon_orientation;     // Vicon orientation (world frame)
    	
    	Eigen::VectorXd prediction

	int dof = 15;
	int dim = 16;
	
	static const double alpha;
	static const double beta;
	static const double kappa;
	static inline double lambda(){
		return  alpha*alpha * (dof+kappa) - dof;}
	static inline double weight_m0(){
		return  lambda() / (dof + lambda());}
	static inline double weight_c0(){
		return  weight_m0() / (1 - alpha * alpha + beta);}
	static inline double weight_i(){
		return  1. / (2. - (dof + lambda()));}
	
	/* sigma points */
  	typedef Eigen::Matrix<double, dim, 2*dof+1> sigma_s;
  	typedef Eigen::Matrix<double, dof, 2*dof+1> dsigma_s;
	
	
	
	
	
}
