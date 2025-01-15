#include "controller/controller.hpp"
#include "utils.cpp"

Controller::Controller() : Node("controller") {
    std::string prop_cmd_topic;
    std::string prop_act_topic;
    std::string log_data_topic = "controller_log";
    std::string pose_topic;
    std::string pose_d_topic;
	std::string oper_mode;

	this->declare_parameter<std::string>("controller/prop_cmd");
	this->declare_parameter<std::string>("controller/pose_full");
	this->declare_parameter<std::string>("controller/pose_d");
	this->declare_parameter<std::string>("controller/log_data");
	this->declare_parameter<std::string>("controller/mode");

    prop_cmd_topic = this->get_parameter_or<std::string>("controller/prop_cmd", "prop_cmd");
    pose_topic = this->get_parameter_or<std::string>("controller/pose_full", "pose_full");
    pose_d_topic = this->get_parameter_or<std::string>("controller/pose_d", "pose_d");
    log_data_topic = this->get_parameter_or<std::string>("controller/log_data", "controller_log");
	this->oper_mode = this->get_parameter_or<std::string>("controller/mode","sim");
	if (this->oper_mode != "sim" && this->oper_mode != "exp") {
		throw std::runtime_error("Invalid mode: " + oper_mode + ". Allowed modes are 'sim' and 'exp'.");
	}

    prop_cmd_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>(prop_cmd_topic, 1000);   
    controller_log_pub = this->create_publisher<controller::msg::ErrorMsg>(log_data_topic, 1000);   

    toggle_sub = this->create_subscription<std_msgs::msg::Bool>("toggle_topic", 10, std::bind(&Controller::toggleCallback,this,std::placeholders::_1));
    pose_sub = this->create_subscription<maneuver::msg::FullPose>(pose_topic, 10, std::bind(&Controller::poseCallback,this,std::placeholders::_1));
    pose_d_sub = this->create_subscription<maneuver::msg::FullPose>(pose_d_topic, 10, std::bind(&Controller::poseDCallback,this,std::placeholders::_1));

    linear_wrench_service_ = this->create_service<controller::srv::LinearWrench>(
        "linear_wrench",
        std::bind(&Controller::linear_wrench_callback, this, std::placeholders::_1, std::placeholders::_2));
    angular_wrench_service_ = this->create_service<controller::srv::AngularWrench>(
        "angular_wrench",
        std::bind(&Controller::angular_wrench_callback, this, std::placeholders::_1, std::placeholders::_2));

    // the following are containers to get params
    bool ground_start;
    double dz_pre_trans;
    double weight;
    double cmd_min = 0;
    double cmd_max = 0;
    double sat_ex_, sat_ev_, sat_iex_, sat_ier_;
    double yaw_moment_limit, rp_moment_limit;
    std::vector<double> att_kp_list;
    std::vector<double> att_kp_ground_list;
    std::vector<double> att_kd_list;
    std::vector<double> att_ki_list;
    std::vector<double> pos_kp_list;
    std::vector<double> pos_kd_list;
    std::vector<double> pos_ki_list;
    std::vector<double> allocation_list;
    std::vector<double> full_allocation_list;
    std::vector<double> positives_array_list;
    std::vector<double> null_space_list;
    std::vector<double> inertia_list;

	// Declare parameters with defaults
	this->declare_parameter<double>("controller_gains.sat.ev");
	this->declare_parameter<bool>("controller_gains.enable_ground");
	this->declare_parameter<double>("controller_gains.dz_pre_trans");
	this->declare_parameter<double>("controller_gains.weight");
	this->declare_parameter<double>("controller_gains.cmd_min");
	this->declare_parameter<double>("controller_gains.cmd_max");
	this->declare_parameter<double>("controller_gains.sat.iex");
	this->declare_parameter<double>("controller_gains.sat.ier");
	this->declare_parameter<double>("controller_gains.sat.ex");
	this->declare_parameter<std::vector<double>>("controller_gains.att_pid.kp");
	this->declare_parameter<std::vector<double>>("controller_gains.att_pid.kpGround");
	this->declare_parameter<std::vector<double>>("controller_gains.att_pid.kd");
	this->declare_parameter<std::vector<double>>("controller_gains.att_pid.ki");
	this->declare_parameter<std::vector<double>>("controller_gains.pos_pid.kp");
	this->declare_parameter<std::vector<double>>("controller_gains.pos_pid.kd");
	this->declare_parameter<std::vector<double>>("controller_gains.pos_pid.ki");
	this->declare_parameter<double>("controller_gains.yaw_moment_limit");
	this->declare_parameter<double>("controller_gains.rp_moment_limit");
	this->declare_parameter<std::vector<double>>("controller_gains.Allocation_matrix");
	this->declare_parameter<std::vector<double>>("controller_gains.null_space");
	this->declare_parameter<std::vector<double>>("controller_gains.Inertia_matrix");

	// Retrieve parameters
	this->sat_ev = this->get_parameter_or<double>("controller_gains.sat.ev", 0.5);
	this->ground_start = this->get_parameter_or<bool>("controller_gains.enable_ground", true);
	this->ground_config_flag = this->ground_start;
	this->dz_pre_trans = this->get_parameter_or<double>("controller_gains.dz_pre_trans", 0.2);
	this->weight = this->get_parameter_or<double>("controller_gains.weight", 1.0);
	this->cmd_min = this->get_parameter_or<double>("controller_gains.cmd_min", 0.0);
	this->cmd_max = this->get_parameter_or<double>("controller_gains.cmd_max", 125.0);
	this->sat_iex = this->get_parameter_or<double>("controller_gains.sat.iex", 2.0);
	this->sat_ier = this->get_parameter_or<double>("controller_gains.sat.ier", 2.0);
	this->sat_ex = this->get_parameter_or<double>("controller_gains.sat.ex", 0.3);

	// Eigen::Vector3d parameters
	std::vector<double> default_att_kp{1.25, 1.25, 3.0};
	att_kp_list = this->get_parameter_or<std::vector<double>>("controller_gains.att_pid.kp", default_att_kp);
	this->att_kp = Eigen::Vector3d::Map(att_kp_list.data());

	std::vector<double> default_att_kp_ground{1.25, 1.25, 3.0};
	att_kp_ground_list = this->get_parameter_or<std::vector<double>>("controller_gains.att_pid.kpGround", default_att_kp_ground);
	this->att_kp_ground = Eigen::Vector3d::Map(att_kp_ground_list.data());

	std::vector<double> default_att_kd{2.0, 2.0, 5.0};
	att_kd_list = this->get_parameter_or<std::vector<double>>("controller_gains.att_pid.kd", default_att_kd);
	this->att_kd = Eigen::Vector3d::Map(att_kd_list.data());

	std::vector<double> default_att_ki{0.25, 0.25, 0.25};
	att_ki_list = this->get_parameter_or<std::vector<double>>("controller_gains.att_pid.ki", default_att_ki);
	this->att_ki = Eigen::Vector3d::Map(att_ki_list.data());

	std::vector<double> default_pos_kp{1.25, 1.25, 4.0};
	pos_kp_list = this->get_parameter_or<std::vector<double>>("controller_gains.pos_pid.kp", default_pos_kp);
	this->pos_kp = Eigen::Vector3d::Map(pos_kp_list.data());

	std::vector<double> default_pos_kd{2.0, 2.0, 6.0};
	pos_kd_list = this->get_parameter_or<std::vector<double>>("controller_gains.pos_pid.kd", default_pos_kd);
	this->pos_kd = Eigen::Vector3d::Map(pos_kd_list.data());

	std::vector<double> default_pos_ki{0.25, 0.25, 0.22};
	pos_ki_list = this->get_parameter_or<std::vector<double>>("controller_gains.pos_pid.ki", default_pos_ki);
	this->pos_ki = Eigen::Vector3d::Map(pos_ki_list.data());

	this->YAW_MOMENT_LIMIT = this->get_parameter_or<double>("controller_gains.yaw_moment_limit", 2.0);
	this->RP_MOMENT_LIMIT = this->get_parameter_or<double>("controller_gains.rp_moment_limit", 3.0);

	// Allocation matrix parameter
	std::vector<double> default_allocation = { 0.2885, 0.8318, -0.7464, -0.3237, -0.0717, -0.7938, 0.8298, -0.0145, 0.9187, -0.3915, -0.2159, -0.4530, 0.9611, 0.0062, -0.2042, -0.6214, -0.2697, 0.3934, 0.6295, -0.8306, -0.2668, 0.6081, 0.5194, -0.7833, -0.0308, 0.2586, 0.2398, -0.2105, -0.0328, -0.2185, -0.2309, 0.2252, 0.1281, 0.1652, 0.0670, -0.2968, -0.1169, -0.1256, -0.1182, 0.2973, 0.4147, -0.3771, 0.3112, 0.2455, -0.4008, -0.2809, 0.3263, -0.2389};
	allocation_list = this->get_parameter_or<std::vector<double>>("controller_gains.Allocation_matrix", default_allocation);

	double G_buffer[allocation_list.size()];
	std::copy(allocation_list.begin(), allocation_list.end(), G_buffer);
	this->n = allocation_list.size() / 6;
	const int n = this->n;

	// Null Space parameter
	std::vector<double> default_null_space = {0.0345,-0.0318,0.3892,0.0892,0.5163,0.1171,0.5698,0.4832,-0.5091,-0.5882,-0.0860,-0.4439,0.0655,-0.4107,0.1293,0.0260};
	null_space_list = this->get_parameter_or<std::vector<double>>("controller_gains.null_space", default_null_space);

	// Convert to Eigen matrix
	double nB_buffer[null_space_list.size()];
	std::copy(null_space_list.begin(), null_space_list.end(), nB_buffer);
	Eigen::Map<Eigen::MatrixXd> nB(nB_buffer, null_space_list.size() / 2, 2); // Assuming n x 2 structure
	this->nB = nB;

	// Inertia Matrix parameter
	std::vector<double> default_inertia_matrix = {0.001, 0., 0., 0., 0.001, 0., 0., 0., 0.001};
	inertia_list = this->get_parameter_or<std::vector<double>>("controller_gains.Inertia_matrix", default_inertia_matrix);

    // Convert to Eigen matrix
    double J_buffer[inertia_list.size()];
    std::copy(inertia_list.begin(), inertia_list.end(), J_buffer);
    Eigen::Map<Eigen::MatrixXd> J(J_buffer, 3, 3); // 3x3 structure
    this->J = J;

        // the following is the storing of the allocation
    // matrix in the RowMajor form with dynamic matrix
    // and inverting of the allocation matrix
    Eigen::Map < Eigen::MatrixXd > G(G_buffer, n, 6);

    Eigen::MatrixXd Srg(3,3);
    Srg <<     0.0, -0.0134, 0.0048,
            0.0134,     0.0, 0.0010,
            -0.0048, -0.0010,    0.0;
    double drag_lift_ratio = 0.0130;//0.023;
    Eigen::MatrixXd G_;
    G_ = G.transpose();	

    #undef solve


    this->iG = G_.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).solve(Eigen::Matrix<
            double, 6, 6>::Identity());
    #define solve lu_solve
    double u_min, u_max;
    u_min = cmd_min * cmd_min;
    u_max = cmd_max * cmd_max;

    this->Iex << 0.0, 0.0, 0.0;
    this->Ier << 0.0, 0.0, 0.0;
    this->ex << 0.0, 0.0, 0.0;
    this->ev << 0.0, 0.0, 0.0;
    this->ew << 0.0, 0.0, 0.0;
    this->ea << 0.0, 0.0, 0.0;
    this->log_data_flag = false;

    this->thrust_dmd = 0.0f;

    this->accd_x_dmd = 0.0f;
    this->accd_y_dmd = 0.0f;

    this->state_timer = 0;
    this->active_integrator = false;
    this->full_actuation = false;

    this->prop_cmd_d  = Eigen::VectorXd::Zero(n);
    this->weight_ = 1;
    }

Controller::~Controller() {
}

bool Controller::linear_wrench_callback(const std::shared_ptr<controller::srv::LinearWrench::Request> req,
                    std::shared_ptr<controller::srv::LinearWrench::Response> res) {
    
    if(this->test_wrench_received) {
        res->status = false;
        return false;
    } else {
        this->wrench_max_time = req->duration + req->ramp;
        this->wrench_ramp_time = req->ramp;
        this->wrench_start_time = this->now();
        
        this->linearWrench_start << req->fx1, req->fy1, req->fz1;
        this->linearWrench_end   << req->fx2, req->fy2, req->fz2;
        this->test_wrench_received = true;
        this->linearWrench = true;
        res->status = true;
        return true;
    }
}
    
void Controller::apply_linear_wrench_test(){
	rclcpp::Time thisTime = this->now();
	double seconds = (thisTime - this->wrench_start_time).seconds();
	Eigen::VectorXd prop_cmd_d(this->n);

	if (seconds > wrench_max_time)
	{
		for(int i = 0; i < this->n; i++) {
			prop_cmd_d[i] = 0;
		}
		this->wrench << 0, 0, 0, 0., 0., 0.;
		this->prop_cmd_d = prop_cmd_d.array();
		this->test_wrench_received = false;
		this->linearWrench = false;
	} else if (seconds < wrench_ramp_time){
		Eigen::Vector3d force = linearWrench_start + (linearWrench_end - linearWrench_start)*(seconds/wrench_ramp_time);
		this->wrench << force[0], force[1], force[2], 0., 0., 0.;
		prop_cmd_d = (this->iG * this->wrench);
    	double min_prop_cmd = 100000;


		for(int i = 0; i < this->n; i++) {
      		if (prop_cmd_d[i] < min_prop_cmd)
        	{
				min_prop_cmd = prop_cmd_d[i];
        	}
      	}

	Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
	Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
	Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);

	prop_cmd_d = prop_cmd_d + this->nB * x;
	this->prop_cmd_d = prop_cmd_d.array().max(0);
	}
	else
	{
		Eigen::Vector3d force = linearWrench_end;
		this->wrench << force[0], force[1], force[2], 0., 0., 0.;
		prop_cmd_d = (this->iG * this->wrench);
    	double min_prop_cmd = 100000;


		for(int i = 0; i < this->n; i++) {
      		if (prop_cmd_d[i] < min_prop_cmd)
        	{
				min_prop_cmd = prop_cmd_d[i];
        	}
       }	
	Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
	Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
	Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);

	prop_cmd_d = prop_cmd_d + this->nB * x;

	this->prop_cmd_d = prop_cmd_d.array().max(0);	}
	last_wrench_sent = false;
}

bool Controller::angular_wrench_callback(const std::shared_ptr<controller::srv::AngularWrench::Request> req,
                       std::shared_ptr<controller::srv::AngularWrench::Response> res) {
	
	if(this->test_wrench_received) {
		res->status = false;
		return false;
	} else {
		this->wrench_max_time = req->duration;
		this->wrench_start_time = this->now();
		this-> angularWrench_f = req->fz;
		this->angularWrenchPhi_1 = req->phi1;
		this->angularWrenchPhi_2 = req->phi2;
		this->test_wrench_received = true;
		this->angularWrench = true;
		res->status = true;
		return true;
	}
}

void Controller::apply_angular_wrench_test(){
	rclcpp::Time thisTime = this->now();
	double seconds = (thisTime - this->wrench_start_time).seconds();
	Eigen::VectorXd prop_cmd_d(this->n);

	if (seconds > wrench_max_time*(1+(1./3.)))
	{
		for(int i = 0; i < this->n; i++) {
			prop_cmd_d[i] = 0;
		}
		this->prop_cmd_d = prop_cmd_d.array();
		this->test_wrench_received = false;
		this->angularWrench = false;
	} else {
	if (seconds > wrench_max_time*(1./3.))
	{
		double phi = angularWrenchPhi_1 + (angularWrenchPhi_2 - angularWrenchPhi_1)*((seconds - (1./3.)*wrench_max_time)/wrench_max_time);
		
		this->wrench <<  angularWrench_f*sin(phi), 0,  angularWrench_f*cos(phi), 0., 0., 0.;}
		else{
			double phi = angularWrenchPhi_1;
			double force = angularWrench_f* (seconds/wrench_max_time)*3.0;
			this->wrench << force*sin(phi), 0, force*cos(phi), 0., 0., 0.;

		}
			
		prop_cmd_d = (this->iG * this->wrench);
    	double min_prop_cmd = 100000;


		for(int i = 0; i < this->n; i++) {
      		if (prop_cmd_d[i] < min_prop_cmd)
        	{
				min_prop_cmd = prop_cmd_d[i];
        	}
      	}

		Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
		Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
		Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);

		prop_cmd_d = prop_cmd_d + this->nB * x;

		this->prop_cmd_d = prop_cmd_d.array().max(0);

	}
	last_wrench_sent = false;
}

void Controller::log_data() {

	unsigned int i;

	// publish data required to be logged
	controller::msg::ErrorMsg controller_log_pub;
	controller_log_pub.ex.x = this->ex.x();
	controller_log_pub.ex.y = this->ex.y();
	controller_log_pub.ex.z = this->ex.z();
	controller_log_pub.ev.x = this->ev.x();
	controller_log_pub.ev.y = this->ev.y();
	controller_log_pub.ev.z = this->ev.z();
	controller_log_pub.ea.x = this->ea.x();
	controller_log_pub.ea.y = this->ea.y();
	controller_log_pub.ea.z = this->ea.z();
	controller_log_pub.er.x = this->eR.x();
	controller_log_pub.er.y = this->eR.y();
	controller_log_pub.er.z = this->eR.z();
	controller_log_pub.ew.x = this->ew.x();
	controller_log_pub.ew.y = this->ew.y();
	controller_log_pub.ew.z = this->ew.z();
	controller_log_pub.iex.x = this->Iex.x();
	controller_log_pub.iex.y = this->Iex.y();
	controller_log_pub.iex.z = this->Iex.z();
	controller_log_pub.ier.x = this->Ier.x();
	controller_log_pub.ier.y = this->Ier.y();
	controller_log_pub.ier.z = this->Ier.z();
	controller_log_pub.accd.x = this->acc_d.x();
	controller_log_pub.accd.y = this->acc_d.y();
	controller_log_pub.accd.z = this->acc_d.z();
	std::vector<float> wrench(this->wrench.data(), this->wrench.data() + this->wrench.size());
	std::vector<float> prop_vel(prop_cmd_d.data(), prop_cmd_d.data() + prop_cmd_d.size());

	Eigen::VectorXd exWrench(6);
	exWrench << this->exF, this->exM;
	std::vector<float> exwrench(exWrench.data(), exWrench.data() + exWrench.size());
	
	controller_log_pub.wrench = wrench;
	controller_log_pub.exwrench = exwrench;
	controller_log_pub.prop = prop_vel;
	controller_log_pub.weight = this->weight_;

	this->controller_log_pub->publish(controller_log_pub);

	// wrench observer instantiation
	this->exF << 0,0,0;
	this->exM << 0,0,0;
	this->LF << 0.1, 0.1, 0.1;
	this->LM << 0.1, 0.1, 0.1;

}

void Controller::update_prop_cmd() {
	std_msgs::msg::Float64MultiArray prop_cmd;
	std::vector < std::string > prop_names = { "propeller1", "propeller2", "propeller3", "propeller4", "propeller5", "propeller6","propeller7","propeller8" };
	std::vector<double> prop_vel(prop_cmd_d.data(), prop_cmd_d.data() + prop_cmd_d.size());
	for(int i = 0; i < this->n; i++) {
		if(prop_vel[i] > this->cmd_max)
			prop_vel[i] = this->cmd_max;
		else if(prop_vel[i] < this->cmd_min)
			prop_vel[i] = this->cmd_min;
	}

	double a,b,c;
	a = 95.67;
	b = 8.34;
	c = 1012;
	int xcount = 0;
	Eigen::ArrayXd prop_PWM;

	if (this->oper_mode == "sim")
	{
		double aa = 1.0/4.4086e-06;
		prop_PWM = prop_cmd_d.array() * aa;
		prop_PWM = prop_PWM.array().sqrt();
		while (xcount <= 7)
		{
			prop_PWM[xcount] = prop_PWM[xcount]/(0.10472*30000);
			if (prop_PWM[xcount] <= 0.0)
				prop_PWM[xcount] = 0.0;
			if (prop_PWM[xcount] >= 1)
				prop_PWM[xcount] = 1;
			prop_vel[xcount] = prop_PWM[xcount];
			xcount++;			
		}
	}
	else
	{
		prop_PWM = a*prop_cmd_d.array().sqrt() + b*prop_cmd_d.array() + c;

		while (xcount <= 7)
		{
			if (prop_PWM[xcount] <= 1030)
				prop_PWM[xcount] = 1000;
			if (prop_PWM[xcount] >= 1900)
				prop_PWM[xcount] = 1900;
			prop_vel[xcount] = (prop_PWM[xcount]-1030)/(1900-1030);
			xcount++;			
		}
	}
	prop_cmd.data = prop_vel;
	this->prop_cmd_pub->publish(prop_cmd);

}

void Controller::send_zero_PWM() {
    std_msgs::msg::Float64MultiArray prop_cmd;
	std::vector<double> prop_vel;
    std::vector<std::string> prop_names = { 
        "propeller1", "propeller2", "propeller3", "propeller4", 
        "propeller5", "propeller6", "propeller7", "propeller8" 
    };
    

    if (this->oper_mode == "sim"){
        prop_vel = std::vector<double>(prop_names.size(), 0.0);
    }else{
        prop_vel = std::vector<double>(prop_names.size(), 1000.0);
	}
	prop_cmd.data = prop_vel;
    this->prop_cmd_pub->publish(prop_cmd);
}

void Controller::send_idle_PWM() {
    std_msgs::msg::Float64MultiArray prop_cmd;
	std::vector<double> prop_vel;
    std::vector<std::string> prop_names = { 
        "propeller1", "propeller2", "propeller3", "propeller4", 
        "propeller5", "propeller6", "propeller7", "propeller8" 
    };
	if (this->oper_mode == "sim"){
    	prop_vel = std::vector<double>(prop_names.size(), 0.05);
	}else{
		prop_vel = std::vector<double>(prop_names.size(), 1060.0);
	}
	prop_cmd.data = prop_vel;
    this->prop_cmd_pub->publish(prop_cmd);
}

void Controller::toggleCallback(const std::shared_ptr<std_msgs::msg::Bool> msg)
{
	this->armed = msg->data;
    RCLCPP_INFO(this->get_logger(), "Armed Controller: %s", this->armed ? "true" : "false");
	if (!this->armed) this->first_desired_received = false;
}

void Controller::poseCallback(const std::shared_ptr<maneuver::msg::FullPose> _pose) {
	this->pose_ = *_pose;
	Eigen::Quaternion<double> q;
	q.coeffs() << this->pose_.pose.orientation.x, this->pose_.pose.orientation.y, 
				  this->pose_.pose.orientation.z, this->pose_.pose.orientation.w;
	this->R = q.toRotationMatrix();
	this->first_pose_received = true;
}

void Controller::poseDCallback(const std::shared_ptr<maneuver::msg::FullPose> _pose_d) {
	this->pose_d = *_pose_d;
	Eigen::Quaternion<double> qd;
	qd.coeffs() << this->pose_d.pose.orientation.x, this->pose_d.pose.orientation.y,
				   this->pose_d.pose.orientation.z, this->pose_d.pose.orientation.w;
	this->Rd = qd.toRotationMatrix();
	if (!this->first_desired_received){
		this->first_desired_received = true;
		this->active_integrator = true;
		this->height_at_lift_off = this->pose_.pose.position.z;

	}
}

void Controller::position_controller() {
	rclcpp::Time thisTime = this->now();
	Eigen::Vector3d ex, ev, ea, Iex;
	Eigen::Vector3d e1;
	Eigen::Vector3d e2;
	Eigen::Vector3d e3;
	e1 << 1, 0, 0;
	e2 << 0, 1, 0;
	e3 << 0, 0, 1;

	Iex << 0.0, 0.0, 0.0;
	double weight_;
	double seconds = (thisTime - this->ramp_time).seconds();
	double launchTime, secondTime, thirdTime, fourthTime;
	if (this->ground_config_flag)
	{
		launchTime =  1.0;
		secondTime =  1.2;
		thirdTime  = 1.4;
		fourthTime  = 1.6;
	} else
	{
		launchTime =  1.0;
		secondTime =  5.0;
		thirdTime  = 9.0;
		fourthTime  = 10.0;
	} 

		

		ex << this->pose_d.pose.position.x - this->pose_.pose.position.x, 
              this->pose_d.pose.position.y - this->pose_.pose.position.y, 
              this->pose_d.pose.position.z - this->pose_.pose.position.z;
		ev << this->pose_d.vel.linear.x - this->pose_.vel.linear.x, 
              this->pose_d.vel.linear.y - this->pose_.vel.linear.y, 
              this->pose_d.vel.linear.z - this->pose_.vel.linear.z;
		ea << this->pose_d.acc.linear.x,
		this->pose_d.acc.linear.y, 
		this->pose_d.acc.linear.z; 

		int i;
		for(i = 0; i < 3; i++) {
			if(fabs(ex(i)) > this->sat_ex)
				ex(i) = copysign(this->sat_ex, ex(i));
		}

		for(i = 0; i < 3; i++) {
			if(fabs(ev(i)) > this->sat_ev) {
				ev(i) = copysign(this->sat_ev, ev(i));
			}
		}
		
		// Execute integrator only if active
		if(this->active_integrator & (seconds > fourthTime)){
			Iex << this->Iex.x(), this->Iex.y(), this->Iex.z();
			Iex += ex * 1.0/((float)this->period);
			
			for(i = 0; i < 3; i++) {
				if(fabs(Iex(i)) > (this->sat_iex)) {
					Iex(i) = copysign((this->sat_iex), Iex(i));
				}
			}
			this->Iex = Iex;
		}

		if(this->log_data_flag) {
			this->ex = ex;
			this->ev = ev;
			this->ea = ea;
		}
	
		if (seconds >thirdTime) weight_ = this->weight;
		else{
			if (seconds > secondTime)
			weight_ = this->weight*0.8  + this->weight*0.2*(seconds - secondTime)/(thirdTime - secondTime);
			else {
				if (seconds > launchTime)
					weight_ = this->weight*0.8;
				else
					weight_ = this->weight*0.25 + this->weight*0.55*(seconds/launchTime);
			}
		}
		this->weight_ = weight_;		

		
		if (seconds > fourthTime)	
			this->acc_d = this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array() + weight_ * (ea.array() + 9.81 * e3.array());
		else if (seconds > thirdTime)
		{       Eigen::Vector3d multiplier;
			multiplier(0) =  1.0;
			multiplier(1) = 1.0;
			multiplier(2) = 1.0;
			this->acc_d = multiplier.array()* (this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array()) +
                                         weight_ * (ea.array() + 9.81 * e3.array());
                }
		else if (seconds > secondTime )
                {
			Eigen::Vector3d multiplier;                 
                        multiplier(0) =  1.0;
                        multiplier(1) = 1.0;
                        multiplier(2) = (seconds - secondTime)/(thirdTime - secondTime);
			this->acc_d = multiplier.array()* (this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array()) +
                                         weight_ * (ea.array() + 9.81 * e3.array());
                }

		else if (seconds > launchTime )
		{
			Eigen::Vector3d multiplier;                 
			 multiplier(0) =  (seconds - launchTime)/(secondTime - launchTime);
                        multiplier(1) = multiplier(0);
                        multiplier(2) = 0.0;
			this->acc_d = multiplier.array()* (this->pos_kp * ex.array() + this->pos_kd * ev.array() + this->pos_ki * Iex.array()) +
                                         weight_ * (ea.array() + 9.81 * e3.array());
                }
		else
                        this->acc_d =  weight_ * (ea.array() + 9.81 * e3.array());
}

void Controller::attitude_controller() {

	Eigen::Matrix3d E;
	Eigen::Matrix3d Rd;
	Eigen::Vector3d eR;
	Eigen::Vector3d Ier;

	Eigen::Vector3d w;
	Eigen::Vector3d wd;
	Eigen::Vector3d ew;

	Ier << 0.0, 0.0, 0.0;

    Rd = this->Rd;
	
	E = 0.5 * (Rd.transpose() * this->R - this->R.transpose() * Rd);
	eR << (E(2, 1) - E(1, 2)) / 2., (E(0, 2) - E(2, 0)) / 2., (E(1, 0) - E(0, 1)) / 2.;

	w << pose_.vel.angular.x, pose_.vel.angular.y, pose_.vel.angular.z;
	wd << pose_d.vel.angular.x, pose_d.vel.angular.y, pose_d.vel.angular.z;
	wd = this->R.transpose() * wd;
	ew = w - wd;

	// Execute integrator only if active

	
    for(int i=0 ; i<3 ; i=i+1 ){
      if( isnan(this->Ier(i))){
            this->Ier << 0.0, 0.0, 0.0;
        }
    }
	// Execute integrator only if active
	if(this->active_integrator){
		
    Ier << this->Ier.x() , this->Ier.y(), this->Ier.z();
    Ier += eR * 1.0/(this->period * 1.0f);

    for(int i = 0; i < 3; i++) {
        if(fabs(Ier(i)) > (this->sat_ier)) {
            Ier(i) = copysign((this->sat_ier), Ier(i));
        }
    }
		}
    this->Ier = Ier;

	this->eR = eR;
	this->ew = ew;

}

void Controller::full_allocation_actuation() {
	
	Eigen::Array3d att_kp;
	rclcpp::Time thisTime = this->now();
	double alpha = 0;
	double trans_time = 1.0;
	double seconds = (thisTime - this->ramp_time).seconds();

	if (this->ground_start == true)
	{
		att_kp = this->att_kp_ground;
		this->Ier << 0., 0., 0.;
		if (this->pose_.pose.position.z - this->height_at_lift_off >= this->dz_pre_trans)
		{
			this->ground_start = false;
			this->time_at_lift_off = seconds;
		}
	}

	else if (seconds < this->time_at_lift_off + trans_time)
	{
		alpha = (seconds - this->time_at_lift_off)/trans_time;
		att_kp = alpha*this->att_kp + (1.0-alpha)*this->att_kp_ground;
	}

	else
	{
		att_kp = this->att_kp;
	}

	this->wrench.block<3, 1>(0, 0) << this->R.transpose() * this->acc_d;
	this->wrench.block<3, 1>(3, 0) << -att_kp * this->eR.array() 
      - this->att_kd * this->ew.array() 
      - this->att_ki * this->Ier.array();


	if(this->wrench[3] > this->RP_MOMENT_LIMIT){
		this->wrench[3] = this->RP_MOMENT_LIMIT;
	} else{
		if(this->wrench[3] < -this->RP_MOMENT_LIMIT){
			this->wrench[3] = -this->RP_MOMENT_LIMIT;
		}		
	}
	
	if(this->wrench[4] > this->RP_MOMENT_LIMIT){
		this->wrench[4] = this->RP_MOMENT_LIMIT;
	} else{
		if(this->wrench[4] < -this->RP_MOMENT_LIMIT){
			this->wrench[4] = -this->RP_MOMENT_LIMIT;
		}		
	}

	if(this->wrench[5] > this->YAW_MOMENT_LIMIT){
		this->wrench[5] = this->YAW_MOMENT_LIMIT;
	} else{
		if(this->wrench[5] < -this->YAW_MOMENT_LIMIT){
			this->wrench[5] = -this->YAW_MOMENT_LIMIT;
		}		
	}

    Eigen::VectorXd prop_cmd_d(this->n);

	prop_cmd_d = (this->iG * this->wrench);
    double min_prop_cmd = 100000;


	for(int i = 0; i < this->n; i++) {
      if (prop_cmd_d[i] < min_prop_cmd)
        {
			min_prop_cmd = prop_cmd_d[i];
        }
      }

	Eigen::Matrix2d H = Eigen::Matrix2d::Identity();
	Eigen::Vector2d f = prop_cmd_d.transpose() * this->nB;
	Eigen::VectorXd x = Utils::solveQP(H, f, this->nB, -1*prop_cmd_d);
	prop_cmd_d = prop_cmd_d + this->nB * x;
	this->prop_cmd_d = prop_cmd_d.array().max(0);
 
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto Node = std::make_shared<Controller>();

    // Initialize Controller parameters
    Node->period = 250; // Period in milliseconds
    Node->log_data_flag = true;

    // Create a loop rate (convert period to frequency in Hz)
    rclcpp::Rate loop_rate(Node->period);

    while (rclcpp::ok()) {
        rclcpp::spin_some(Node); // Process callbacks

        // Main control logic
        auto thisTime = Node->now();

        if (Node->armed) {
            if (Node->first_pose_received && Node->first_desired_received) {
                Node->position_controller();
                Node->attitude_controller();

                Node->full_allocation_actuation();
                Node->update_prop_cmd();

                if (Node->log_data_flag) {
                    Node->log_data();
                }
            } else {
                Node->ramp_time = thisTime;

                if (Node->test_wrench_received) {
                    if (Node->linearWrench) {
                        Node->apply_linear_wrench_test();
                    } else if (Node->angularWrench) {
                        Node->apply_angular_wrench_test();
                    }
                    Node->update_prop_cmd();
                    if (Node->log_data_flag) {
                        Node->log_data();
                    }
                } else {
                    Node->send_idle_PWM();
                    if (Node->log_data_flag) {
                        Node->log_data();
                    }
                }
            }
        } else {
            Node->send_zero_PWM();
            Node->first_desired_received = false;
            if (Node->log_data_flag) {
                Node->log_data();
            }
        }

        loop_rate.sleep(); // Maintain loop rate
    }

    rclcpp::shutdown();
    return 0;
}
