#include "vito_controllers/DualArms.h"

DualArms::DualArms() : nh_params_right("/right_arm"), nh_params_left("/left_arm") {
	ROS_INFO("DualArms constructor");
// 	control_type = Control::CARTESIAN_IMPEDANCE; // WORKING
	control_type = CARTESIAN_IMPEDANCE_ELBOW;
	// control_type = Control::JOINT_IMPEDANCE;
}

DualArms::~DualArms() 
{
	ROS_INFO("DualArms clean exit");
}

Eigen::Matrix<double,3,3> DualArms::skew(Eigen::Matrix<double,3,1> v) {
	Eigen::Matrix<double,3,3> temp;
	temp <<     0, -v(2),  v(1),
		v(2),     0, -v(0),
		-v(1),  v(0),    0;
	return temp;
}

void DualArms::init()
{
	if(!load_robot(RIGHT))
	{
		ROS_ERROR_STREAM("Error while loading robot model " << RIGHT << "!");
	}
	if(!load_robot(LEFT))
	{
		ROS_ERROR_STREAM("Error while loading robot model " << LEFT << "!");
	}

	sub_joint_state_right_arm_ 	= nh.subscribe("/right_arm/joint_states",1,&DualArms::joint_state_callback_right,this);
	sub_joint_state_left_arm_ 	= nh.subscribe("/left_arm/joint_states",1,&DualArms::joint_state_callback_left,this);
	sub_command_right_ = nh.subscribe("/right_arm/vito_bridge_controller/commandCart_right",1,&DualArms::commandCart_right_,this);

	q_right_meas_.resize(number_arm_joints);
	qdot_right_meas_.resize(number_arm_joints);
	tau_right_meas_.resize(number_arm_joints); // sensor readings
	tau_right_.resize(number_arm_joints); // actuator commands

	q_left_meas_.resize(number_arm_joints);
	qdot_left_meas_.resize(number_arm_joints);
	tau_left_meas_.resize(number_arm_joints); // sensor readings
	tau_left_.resize(number_arm_joints); // actuator commands

	q_right_ref_.resize(number_arm_joints);
	q_left_ref_.resize(number_arm_joints);

	pub_torques_right_arm_ = nh.advertise<sensor_msgs::JointState>("/right_arm/vito_bridge_controller/command_torques", 1);
	pub_torques_left_arm_ = nh.advertise<sensor_msgs::JointState>("/left_arm/vito_bridge_controller/command_torques", 1);
	pub_error_right_arm_ = nh.advertise<sensor_msgs::JointState>("/right_arm/vito_bridge_controller/error", 1);

	J_right_.resize(kdl_chain_right_.getNrOfJoints());
	J_right_last_.resize(kdl_chain_right_.getNrOfJoints());
	J_left_.resize(kdl_chain_left_.getNrOfJoints());

	M_right_.resize(kdl_chain_right_.getNrOfJoints());
	C_right_.resize(kdl_chain_right_.getNrOfJoints());
	G_right_.resize(kdl_chain_right_.getNrOfJoints());

//     CartesianForces_right_.resize(6);
//     CartesianForces_left_.resize(6);
// 
    K_joint.resize(kdl_chain_.getNrOfJoints());
    D_joint.resize(kdl_chain_.getNrOfJoints());

    K_joint(0) = 50.0;
    K_joint(1) = 50.0;
    K_joint(2) = 10.0;
    K_joint(3) = 5.0;
    K_joint(4) = 5.0;
    K_joint(5) = 5.0;
    K_joint(6) = 5.0;

    D_joint(0) = 10;
    D_joint(1) = 10;
    D_joint(2) = 7;
    D_joint(3) = 7;
    D_joint(4) = 1;
    D_joint(5) = 1;
    D_joint(6) = 1;

	
    K_cart.resize(6);
//     K_cart(0) = 100.0;
//     K_cart(1) = 1000.0;
//     K_cart(2) = 1000.0;
//     K_cart(3) = 0.0;
//     K_cart(4) = 0.0;
//     K_cart(5) = 0.0;
    
//     // early morning working oscillates -- works without hands *******
// 	K_cart(0) = 80.0;
//     K_cart(1) = 80.0;
//     K_cart(2) = 80.0;
//     K_cart(3) = 5.0;
//     K_cart(4) = 5.0;
//     K_cart(5) = 5.0;

    // 
	K_cart(0) = 100.0;
    K_cart(1) = 100.0;
    K_cart(2) = 100.0;
    K_cart(3) = 1.0;
    K_cart(4) = 1.0;
    K_cart(5) = 1.0;
    
    // early morning working tuned
//     K_cart(0) = 160.0;
//     K_cart(1) = 160.0;
//     K_cart(2) = 160.0;
//     K_cart(3) = 20.0;
//     K_cart(4) = 20.0;
//     K_cart(5) = 20.0;

//     K_cart(0) = 320.0;
//     K_cart(1) = 320.0;
//     K_cart(2) = 320.0;
//     K_cart(3) = 40.0;
//     K_cart(4) = 40.0;
//     K_cart(5) = 40.0;
    
    D_cart.resize(6);
//     D_cart(0) = 10.0;
//     D_cart(1) = 100.0;
//     D_cart(2) = 100.0;
//     D_cart(3) = 0.0;
//     D_cart(4) = 0.0;
//     D_cart(5) = 0.0;
    
	// D_cart(0) = 5.0;
    // D_cart(1) = 5.0;
    // D_cart(2) = 5.0;
    
    // lorale tuning
//     D_cart(3) = 2.0;
//     D_cart(4) = 2.0;
//     D_cart(5) = 2.0;
// 	
//     D_cart(0) = 14.4;
//     D_cart(1) = 14.4;
//     D_cart(2) = 14.4;
    
    // work without hands *******
//         D_cart(3) = 1.0;
//     D_cart(4) = 1.0;
//     D_cart(5) = 1.0;
// 	
//     D_cart(0) = 7.2;
//     D_cart(1) = 7.2;
//     D_cart(2) = 7.2;
    
     D_cart(3) = 1.0;
    D_cart(4) = 1.0;
    D_cart(5) = 1.0;
	
    D_cart(0) = 1.0;
    D_cart(1) = 1.0;
    D_cart(2) = 1.0;
    
    // D_cart(3) = 7.2;
    // D_cart(4) = 7.2;
    // D_cart(5) = 7.2;
	

	K_cart_imp_right.setZero();
	K_cart_imp_right(0,0) = 160;
	K_cart_imp_right(1,1) = 160;
	K_cart_imp_right(2,2) = 160;
	K_cart_imp_right(3,3) = 1.0;
	K_cart_imp_right(4,4) = 1.0;
	K_cart_imp_right(5,5) = 1.0;

	D_cart_imp_right.setZero();
	D_cart_imp_right(0,0) = 72;
	D_cart_imp_right(1,1) = 72;
	D_cart_imp_right(2,2) = 72;
	D_cart_imp_right(3,3) = 1.0;
	D_cart_imp_right(4,4) = 1.0;
	D_cart_imp_right(5,5) = 1.0;

	M_cart_imp_right.setZero();
	M_cart_imp_right(0,0) = 10;
	M_cart_imp_right(1,1) = 10;
	M_cart_imp_right(2,2) = 10;
	M_cart_imp_right(3,3) = 1.0;
	M_cart_imp_right(4,4) = 1.0;
	M_cart_imp_right(5,5) = 1.0;

	N = 1000;

	// empty messages
	msg_right_arm_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg_right_arm_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg_right_arm_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg_left_arm_.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg_left_arm_.velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	msg_left_arm_.effort = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	ros::Time time_prec_ = ros::Time::now();
}

void DualArms::run()
{
	ROS_INFO("Baby we were born to run!");
	ros::Rate f(100);
	while(ros::ok())
	{
		if(right_arm_alive)//&& left_arm_alive)
		{
			pub_ee_pose();
			compute_torques();
			pub_torques();
			pub_error();
		}
		ros::spinOnce();
		f.sleep();
	}
}

void DualArms::compute_torques()
{
	switch(control_type)
	{
		case Control::JOINT_IMPEDANCE:
			// ROS_INFO("JOINT_IMPEDANCE");
			joint_impedance();
			break;
		case Control::CARTESIAN_IMPEDANCE:
			// ROS_INFO("CARTESIAN_IMPEDANCE");
			cartesian_impedance();
			break;
		case Control::CARTESIAN_IMPEDANCE_ELBOW:
			// ROS_INFO("CARTESIAN_IMPEDANCE_ELBOW");
			cartesian_impedance_elbow();
			break;
		default:
			ROS_ERROR_STREAM("Unkown control type in DualArms::compute_torques!");
			break;
	}
}

void DualArms::joint_impedance() 
{
	if(right_arm_alive)
	{

		// ROS_INFO("JOINT_IMPEDANCE running");
		for(size_t i=0; i<number_arm_joints; i++)
		{
			// ROS_INFO_STREAM(K_joint(i) << " " << q_right_ref_(i)  << " " <<  q_right_meas_(i)  << " " <<  D_joint(i)  << " " <<  qdot_right_meas_(i));
			tau_right_(i) = K_joint(i)*(q_right_ref_(i) - q_right_meas_(i)) - D_joint(i) * qdot_right_meas_(i);
			// tau_left_(i) = K_joint(i)*(q_left_ref_(i) - q_left_meas_(i)) - D_joint(i) * qdot_left_meas_(i);
			ROS_INFO_STREAM(tau_right_(i));
		}
	}
	if(left_arm_alive)
	{
		// ROS_INFO("JOINT_IMPEDANCE running");
		for(size_t i=0; i<number_arm_joints; i++)
		{
			// ROS_INFO_STREAM(K_joint(i) << " " << q_left_ref_(i)  << " " <<  q_left_meas_(i)  << " " <<  D_joint(i)  << " " <<  qdot_left_meas_(i));
			tau_left_(i) = K_joint(i)*(q_left_ref_(i) - q_left_meas_(i)) - D_joint(i) * qdot_left_meas_(i);
			ROS_INFO_STREAM(tau_left_(i));
		}
	}
}

void DualArms::cartesian_impedance()
{
	ROS_INFO_STREAM(__LINE__ << " ### Cartesian Impedance Begin");
	// update dynamic and kinematic quantities
	id_solver_right_->JntToMass(q_right_meas_, M_right_);
	id_solver_right_->JntToGravity(q_right_meas_, G_right_);
	id_solver_right_->JntToCoriolis(q_right_meas_, qdot_right_meas_, C_right_);
	jnt_to_jac_solver_right_->JntToJac(q_right_meas_, J_right_);
	fk_pos_solver_right_->JntToCart(q_right_meas_, x_meas_right_);
	// printKDLFrame(x_meas_right_);
	// printKDLJacobian(J_right_);
	// printKDLInertia(M_right_);
	// printKDLJntArray(G_right_, "G_right_");
	// printKDLJntArray(C_right_, "C_right_");
	// tau = g(q) + C(q,\dot{q}) + J(q)'*( - K_d * \tilde{x});
	// printKDLJacobian(J_right_);
	// printJacobianTranspose(J_right_);
	x_tilde_ = KDL::diff(x_right_,x_ref_virtual_);
	x_tilde_dot_ = KDL::diff(x_right_,x_right_old_,period.toSec());
	
	printKDLTwist(x_tilde_, "pose error");
	printKDLTwist(x_tilde_dot_, "pose error_derivative");

	// KDL::JntArray x_tilde_dot_;
	// x_tilde_dot_.resize(6);
	// x_tilde_dot_.data = J_right_.data*qdot_right_meas_.data;
	// printKDLJntArray(x_tilde_dot_, "pose error derivative");
	
	tau_right_.data.setZero();
	for(int i=0; i<number_arm_joints; i++)
	{
		for(int j=0; j<6; j++)
		{
			//tau_right_(i) += J_right_(j,i)*(K_cart(j)*x_tilde(j)); // WORKING
			tau_right_(i) += J_right_(j,i)*(K_cart(j)*x_tilde_(j) + D_cart(j)*x_tilde_dot_(j)); // WORKING

		}
	}
	printKDLJntArray(tau_right_,"tau_right_");
	ROS_INFO_STREAM(__LINE__ << " ### Cartesian Impedance End");
}

void DualArms::cartesian_impedance_elbow()
{
	ROS_INFO_STREAM(__LINE__ << " ### Cartesian Impedance Elbow Begin");
	// update dynamic and kinematic quantities
	id_solver_right_->JntToMass(q_right_meas_, M_right_);
	id_solver_right_->JntToGravity(q_right_meas_, G_right_);
	id_solver_right_->JntToCoriolis(q_right_meas_, qdot_right_meas_, C_right_);
	jnt_to_jac_solver_right_->JntToJac(q_right_meas_, J_right_);
	fk_pos_solver_right_->JntToCart(q_right_meas_, x_meas_right_);
	// printKDLFrame(x_meas_right_);
	// printKDLJacobian(J_right_);
	// printKDLInertia(M_right_);
	// printKDLJntArray(G_right_, "G_right_");
	// printKDLJntArray(C_right_, "C_right_");
	// tau = g(q) + C(q,\dot{q}) + J(q)'*( - K_d * \tilde{x});
	// printKDLJacobian(J_right_);
	// printJacobianTranspose(J_right_);
	x_tilde_ = KDL::diff(x_right_,x_ref_virtual_);
	x_tilde_dot_ = KDL::diff(x_right_,x_right_old_,period.toSec());
	
	printKDLTwist(x_tilde_, "pose error");
	printKDLTwist(x_tilde_dot_, "pose error_derivative");

	// KDL::JntArray x_tilde_dot_;
	// x_tilde_dot_.resize(6);
	// x_tilde_dot_.data = J_right_.data*qdot_right_meas_.data;
	// printKDLJntArray(x_tilde_dot_, "pose error derivative");
	
	KDL::JntArray K_cart_second;
	
	      K_cart_second.resize(6);
    K_cart_second(0) = 5.0;
    K_cart_second(1) = 5.0;
    K_cart_second(2) = 5.0;
    K_cart_second(3) = 0.0;
    K_cart_second(4) = 0.0;
    K_cart_second(5) = 0.0;
    
    KDL::Twist err_elbow;
    err_elbow = KDL::diff(x_right_elbow_,x_ref_right_elbow_);
    
	// end effector task - higher priority
	tau_right_.data.setZero();
	for(int i=0; i<number_arm_joints; i++)
	{
		for(int j=0; j<6; j++)
		{
			//tau_right_(i) += J_right_(j,i)*(K_cart(j)*x_tilde(j)); // WORKING
			// tau_right_(i) += J_right_(j,i)*(K_cart(j)*x_tilde_(j) + D_cart(j)*x_tilde_dot_(j)); // WORKING
			tau_right_(i) += J_right_(j,i)*(K_cart(j)*x_tilde_(j) + D_cart(j)*x_tilde_dot_(j)); // WORKING
			double eye = (i==j)?1.0:0.0;
			double zero = (i>2)?0.0:1.0;
			tau_right_(i) += (eye - J_right_(j,i)*J_right_(i,j)) * 
                                                    ( zero*J_right_(j,i)*K_cart_second(j)*err_elbow(j) );  
		}
	}
	
	
	printKDLJntArray(tau_right_,"tau_right_");
	ROS_INFO_STREAM(__LINE__ << " ### Cartesian Impedance Elbow End");
}

void DualArms::cartesian_impedance_lorale() 
{
	if(false)
	{
	if (starting) {
		starting = false;
		counter = 2*N;
		fk_pos_solver_right_->JntToCart(q_right_meas_, x_meas_right_);
		xDES_step_right_(0) = x_meas_right_.p(0);
		xDES_step_right_(1) = x_meas_right_.p(1);
		xDES_step_right_(2) = x_meas_right_.p(2);
		jnt_to_jac_solver_right_->JntToJac(q_right_meas_, J_right_last_);
	}

	// ROS_INFO("CARTESIAN_IMPEDANCE running");
	id_solver_right_->JntToMass(q_right_meas_, M_right_);
	id_solver_right_->JntToGravity(q_right_meas_, G_right_);
	id_solver_right_->JntToCoriolis(q_right_meas_, qdot_right_meas_, C_right_);
	jnt_to_jac_solver_right_->JntToJac(q_right_meas_, J_right_);
	fk_pos_solver_right_->JntToCart(q_right_meas_, x_meas_right_);
	xVEC_right_(0) = x_meas_right_.p(0);
    xVEC_right_(1) = x_meas_right_.p(1);
    xVEC_right_(2) = x_meas_right_.p(2);
    xVEC_right_(3) = 0;
    xVEC_right_(4) = 0;
    xVEC_right_(5) = 0;
	//x_meas_right_.M.GetEulerZYX(xVEC_right_(5),xVEC_right_(4),xVEC_right_(3));
	//x_meas_right_.M.GetEulerZYX(rpy_right_(2),rpy_right_(1),rpy_right_(0));

	// velocity error
	e_ref_dot_right_ = J_right_.data*qdot_right_meas_.data;
	e_ref_dot_right_(3) = 0;
	e_ref_dot_right_(4) = 0;
	e_ref_dot_right_(5) = 0;

	if (counter < N) {
		// interpolazione per le posizioni
		xDES_step_right_ = xDES_step_right_ + (xDES_right_ - x0_right_)/N;
		// interpolazione per l'orientamento
		//quat_t_right_ = quat_0_right_.slerp(quat_f_right_,counter/N);
		counter++;
	} else if(counter != 2*N) {
		if (!primo_right_)
			ROS_INFO("End positioning.");
		xDES_step_right_ = xDES_right_;
		//quat_t_right_ = quat_f_right_;
		primo_right_ = true;
	}    
	//position error
	e_ref_right_ = xDES_step_right_ - xVEC_right_;

	period = ros::Time::now() - time_prec_;
	time_prec_ = ros::Time::now();

	// Quaternion
	//quat_des_vec_right_(0) = quat_t_right_.x();
	//quat_des_vec_right_(1) = quat_t_right_.y();
	//quat_des_vec_right_(2) = quat_t_right_.z();
	//quat_des_scal_right_ = quat_t_right_.w();
	//x_meas_right_.M.GetQuaternion(quat_vec_right_(0),quat_vec_right_(1),quat_vec_right_(2),quat_scal_right_);
	//quat_temp_right_ = quat_scal_right_*quat_des_vec_right_ - quat_des_scal_right_*quat_vec_right_ - skew(quat_des_vec_right_)*quat_vec_right_;
	e_ref_right_(3) = 0;//quat_temp_right_(0);
	e_ref_right_(4) = 0;//quat_temp_right_(1);
	e_ref_right_(5) = 0;//quat_temp_right_(2);
	e_ref_dot_right_(3) = 0;//(quat_temp_(0) - quat_old_(0))/period.toSec();
	e_ref_dot_right_(4) = 0;//(quat_temp_(1) - quat_old_(1))/period.toSec();
	e_ref_dot_right_(5) = 0;//(quat_temp_(2) - quat_old_(2))/period.toSec();
	//quat_old_right_ = quat_temp_right_;

	J_right_dot_.data = (J_right_.data - J_right_last_.data)/period.toSec();
	J_right_last_ = J_right_;

	L_right_ = J_right_.data*M_right_.data.inverse()*J_right_.data.transpose(); 
	L_right_ = L_right_.inverse();

	//tau_right_.data = G_right_.data + C_right_.data;

	//for(size_t i = 0; i < 7; i++)
	//	  tau_right_(i) = 0;
		
	//ROS_INFO_STREAM("e: " << e_ref_right_);
	//ROS_INFO_STREAM("edot: " << e_ref_dot_right_);
	tau_right_.data = J_right_.data.transpose()*L_right_*(
		M_cart_imp_right.inverse()*(K_cart_imp_right*e_ref_right_ - D_cart_imp_right*e_ref_dot_right_) - J_right_dot_.data*qdot_right_meas_.data
	);
	//tau_right_.data = G_right_.data + C_right_.data + J_right_.data.transpose()*(K_cart_imp_right*e_ref_right_ - D_cart_imp_right*e_ref_dot_right_);
    
	//ROS_INFO_STREAM("New Torque Request: ");
	//ROS_INFO_STREAM(tau_right_.data);
	ROS_INFO_STREAM(e_ref_right_(0) << " " << e_ref_right_(1) << " " << e_ref_right_(2));
	}
}

void DualArms::pub_torques()
{
	ROS_INFO_STREAM(__LINE__ << " ###");
	if(right_arm_alive)
	{
		for(size_t i=0; i < number_arm_joints; i++) 
			msg_right_arm_.effort[i] = tau_right_(i);
		msg_right_arm_.header.stamp = ros::Time::now();
		pub_torques_right_arm_.publish(msg_right_arm_);
	}

	if(left_arm_alive)
	{
		for(size_t i=0; i < number_arm_joints; i++)
			msg_left_arm_.effort[i] = tau_left_(i);
		msg_left_arm_.header.stamp = ros::Time::now();
		pub_torques_left_arm_.publish(msg_left_arm_);
	}
	ROS_INFO_STREAM(__LINE__ << " ###");
}

void DualArms::pub_error()
{
	if(right_arm_alive)
	{
		for(size_t i=0; i < 6; i++)
			msg_right_arm_.position[i] = x_tilde_(i);//e_ref_right_(i);
		msg_right_arm_.header.stamp = ros::Time::now();

		pub_error_right_arm_.publish(msg_right_arm_);
	}
}

void DualArms::pub_ee_pose()
{
	// world_to_base_right_arm = KDL::Rotation::RPY(3.1415926535897932384626, -3.1415926535897932384626/4.0, 0.0);
	// world_to_base_right_arm = KDL::Rotation::Identity();
	// x_right_.p = world_to_base_right_arm.Inverse()*x_right_.p;
	// x_right_.M = world_to_base_right_arm.Inverse()*x_right_.M;
    // <origin xyz="0.77 0.801 1.607" rpy="3.1415 -0.7854 0"/>   


	if(right_arm_alive)
	{
    	tf::transformKDLToTF( x_right_, tf_ee_pose_right_);
		// br_ee_pose_right_.sendTransform(tf::StampedTransform(tf_ee_pose_right_, ros::Time::now(), "right_arm_base_link", "ciao_destra"));
    	br_ee_pose_right_.sendTransform(tf::StampedTransform(tf_ee_pose_right_, ros::Time::now(), "right_arm_base_link", "ciao_destra"));
	
	tf::transformKDLToTF( x_ref_virtual_, tf_ee_ref_pose_right_);
	br_ee_ref_pose_right_.sendTransform(tf::StampedTransform(tf_ee_ref_pose_right_, ros::Time::now(), "right_arm_base_link", "ref_right"));
	
	  if(control_type == Control::CARTESIAN_IMPEDANCE_ELBOW)
	  {
	    tf::transformKDLToTF( x_right_elbow_, tf_elbow_pose_right_);
	    br_elbow_pose_right_.sendTransform(tf::StampedTransform(tf_elbow_pose_right_,ros::Time::now(), "right_arm_base_link", "elbow_right"));

	    tf::transformKDLToTF( x_ref_right_elbow_, tf_elbow_ref_right_);
	    br_elbow_pose_right_.sendTransform(tf::StampedTransform(tf_elbow_ref_right_,ros::Time::now(), "right_arm_base_link", "ref_elbow_right"));
	  }
	}
	if(left_arm_alive)
	{
		tf::transformKDLToTF( x_left_, tf_ee_pose_left_);
    	br_ee_pose_left_.sendTransform(tf::StampedTransform(tf_ee_pose_left_, ros::Time::now(), "left_arm_base_link", "ciao_sinistra"));
	
	// TODO for left arm: change x_ref_virtual_ to x_ref_right_virtual_
// 	tf::transformKDLToTF( x_ref_virtual_, tf_ee_ref_pose_right_);
// 	br_ee_ref_pose_right_.sendTransform(tf::StampedTransform(tf_ee_ref_pose_right_, ros::Time::now(), "right_arm_base_link", "ref_right"));
	}

}

void DualArms::joint_state_callback_left(const sensor_msgs::JointState& msg)
{
	const std::vector<int> joint_order = {0, 1, 6, 2, 3, 4, 5}; // when reading joints from joint_state publisher reorder them as the in KDL chain
	for(size_t i=0; i<number_arm_joints;i++)
	{
		q_left_meas_(i) = msg.position.at(joint_order.at(i));
		qdot_left_meas_(i) = msg.velocity.at(joint_order.at(i));
		tau_left_meas_(i) = msg.effort.at(joint_order.at(i));
	}

	// update kinematics with last readings
    fk_pos_solver_left_->JntToCart(q_left_meas_,x_left_);
  	// compute Jacobian
    jnt_to_jac_solver_left_->JntToJac(q_left_meas_,J_left_);

    if(!left_arm_alive)
    {
        q_left_ref_ = q_left_meas_;
        left_arm_alive = true;
    }

}

void DualArms::joint_state_callback_right(const sensor_msgs::JointState& msg)
{
	const std::vector<int> joint_order = {0, 1, 6, 2, 3, 4, 5}; // when reading joints from joint_state publisher reorder them as the in KDL chain
	for(size_t i=0; i<number_arm_joints;i++)
	{
		q_right_meas_(i) = msg.position.at(joint_order.at(i));
		qdot_right_meas_(i) = msg.velocity.at(joint_order.at(i));
		tau_right_meas_(i) = msg.effort.at(joint_order.at(i));
	}

	// update cartesian position with last readings
	x_right_old_ = x_right_;
	period = ros::Time::now() - time_prec_;
	time_prec_ = ros::Time::now();
    fk_pos_solver_right_->JntToCart(q_right_meas_,x_right_);
    
    if(control_type == Control::CARTESIAN_IMPEDANCE_ELBOW)
      fk_pos_solver_right_->JntToCart(q_right_meas_,x_right_elbow_,4);//"right_arm_a3_joint");
    
    
	if(!virtual_ref_available)
	{
		if(fabs(x_right_.p(2)) >= 0.1f) // HARDCODED with physical insight
		{
			x_ref_virtual_ = x_right_;
			// printKDLFrame(x_right_);
			// printKDLFrame(x_ref_virtual_);
			virtual_ref_available = true;
		}
	}
    // compute Jacobian
    jnt_to_jac_solver_right_->JntToJac(q_right_meas_,J_right_);

    if(!right_arm_alive)
    {
        q_right_ref_ = q_right_meas_;
        right_arm_alive = true;
    }
}

bool DualArms::load_robot(const Arm& arm)
//template <typename JI>
    //bool KinematicChainControllerBase<JI>::init(JI *robot, ros::NodeHandle &n)
{
	ros::NodeHandle nh_params;
	switch(arm)
	{
		case Arm::RIGHT:
			nh_params = nh_params_right;
			break;
		case Arm::LEFT:
			nh_params = nh_params_left;
			break;
		default:
			ROS_ERROR_STREAM("Error in load_robot: called with argument " << arm << ". Valid arguments are " << Arm::RIGHT << " " << Arm::LEFT);
			return false;			
	}
	
    // get URDF and name of root and tip from the parameter server
    std::string robot_description, root_name, tip_name;

    ROS_INFO_STREAM("nh_params.getNamespace() " << nh_params.getNamespace());
    if (!ros::param::search(nh_params.getNamespace(),"robot_description", robot_description))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No robot description (URDF) found on parameter server ("<<nh_params.getNamespace()<<"/robot_description)");
        return false;
    }

    if (!nh_params.getParam("root", root_name))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No root name found on parameter server ("<<nh_params.getNamespace()<<"/root_name)");
        return false;
    }

    if (!nh_params.getParam("tip", tip_name))
    {
        ROS_ERROR_STREAM("KinematicChainControllerBase: No tip name found on parameter server ("<<nh_params.getNamespace()<<"/tip_name)");
        return false;
    }
 
    // Get the gravity vector (direction and magnitude)
    gravity_ = KDL::Vector::Zero();
    gravity_(2) = -9.81;

    // Construct an URDF model from the xml string
    std::string xml_string;

    if (nh_params.hasParam(robot_description))
        nh_params.getParam(robot_description.c_str(), xml_string);
    else
    {
        ROS_ERROR("Parameter %s not set, shutting down node...", robot_description.c_str());
        nh_params.shutdown();
        return false;
    }

    if (xml_string.size() == 0)
    {
        ROS_ERROR("Unable to load robot model from parameter %s",robot_description.c_str());
        nh_params.shutdown();
        return false;
    }

    ROS_DEBUG("%s content\n%s", robot_description.c_str(), xml_string.c_str());
    // ROS_INFO("%s content\n%s", robot_description.c_str(), xml_string.c_str());
    
    // Get urdf model out of robot_description
    urdf::Model model;
    if (!model.initString(xml_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        nh_params.shutdown();
        return false;
    }
    ROS_INFO("Successfully parsed urdf file");
    
    KDL::Tree kdl_tree_;
    if (!kdl_parser::treeFromUrdfModel(model, kdl_tree_))
    {
        ROS_ERROR("Failed to construct kdl tree");
        nh_params.shutdown();
        return false;
    }

    // Populate the KDL chain
    if(!kdl_tree_.getChain(root_name, tip_name, kdl_chain_))
    {
        ROS_ERROR_STREAM("Failed to get KDL chain from tree: ");
        ROS_ERROR_STREAM("  "<<root_name<<" --> "<<tip_name);
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfJoints()<<" joints");
        ROS_ERROR_STREAM("  Tree has "<<kdl_tree_.getNrOfSegments()<<" segments");
        ROS_ERROR_STREAM("  The segments are:");

        KDL::SegmentMap segment_map = kdl_tree_.getSegments();
        KDL::SegmentMap::iterator it;

        for( it=segment_map.begin(); it != segment_map.end(); it++ )
          ROS_ERROR_STREAM( "    "<<(*it).first);

        return false;
    }

    {
    	for(std::vector<KDL::Segment>::const_iterator it = kdl_chain_.segments.begin(); it != kdl_chain_.segments.end(); ++it)
        {
            if ( it->getJoint().getType() != KDL::Joint::None )
            {
                ROS_INFO("%s", it->getJoint().getName().c_str() );
            }
        }  
    }

    ROS_DEBUG("Number of segments: %d", kdl_chain_.getNrOfSegments());
    ROS_DEBUG("Number of joints in chain: %d", kdl_chain_.getNrOfJoints());
    
    // Parsing joint limits from urdf model along kdl chain
    boost::shared_ptr<const urdf::Link> link_ = model.getLink(tip_name);
    boost::shared_ptr<const urdf::Joint> joint_;
    joint_limits_.min.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.max.resize(kdl_chain_.getNrOfJoints());
    joint_limits_.center.resize(kdl_chain_.getNrOfJoints());
    int index;
    
    for (int i = 0; i < kdl_chain_.getNrOfJoints() && link_; i++)
    {
        joint_ = model.getJoint(link_->parent_joint->name);  
        ROS_INFO("Getting limits for joint: %s", joint_->name.c_str());
        index = kdl_chain_.getNrOfJoints() - i - 1;

        joint_limits_.min(index) = joint_->limits->lower;
        joint_limits_.max(index) = joint_->limits->upper;
        joint_limits_.center(index) = (joint_limits_.min(index) + joint_limits_.max(index))/2;

        link_ = model.getLink(link_->getParent()->name);
    }

    // Get joint handles for all of the joints in the chain
    // getHandles(robot);
    
    // ROS_DEBUG("Number of joints in handle = %lu", joint_handles_.size() );

	number_arm_joints = kdl_chain_.getNrOfJoints();

	world_to_base_right_arm = KDL::Rotation::RPY(3.1415926535897932384626, -3.1415926535897932384626/4.0, 0.0);
	world_to_base_left_arm = KDL::Rotation::RPY(3.1415926535897932384626, -3.1415926535897932384626/4.0, 0.0); // TODO what's the number here
		    
	switch(arm)
	{
		case Arm::RIGHT:

			kdl_chain_right_ = kdl_chain_;
			joint_limits_right_ = joint_limits_;
			// kinematics and dynamics stuff
		    jnt_to_jac_solver_right_.reset(new KDL::ChainJntToJacSolver(kdl_chain_right_));
		    id_solver_right_.reset(new KDL::ChainDynParam(kdl_chain_right_,world_to_base_right_arm.Inverse()*gravity_));
		    fk_pos_solver_right_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_right_));
			break;
		case Arm::LEFT:
			kdl_chain_left_ = kdl_chain_;
			joint_limits_left_ = joint_limits_;
			jnt_to_jac_solver_left_.reset(new KDL::ChainJntToJacSolver(kdl_chain_left_));
		    id_solver_left_.reset(new KDL::ChainDynParam(kdl_chain_left_,world_to_base_left_arm.Inverse()*gravity_));
		    fk_pos_solver_left_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_left_));
			break;
		default:
			ROS_ERROR_STREAM("Error in load_robot: called with argument " << arm << ". Valid arguments are " << Arm::RIGHT << " " << Arm::LEFT);
			return false;			
	}
    return true;
}

void DualArms::commandCart_right_(const std_msgs::Float64MultiArray::ConstPtr &msg) {
  if ((int)msg->data.size() != 6 && (int)msg->data.size() != 12) {
    ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
    return;
  }
  
  fk_pos_solver_right_->JntToCart(q_right_meas_,x_meas_right_);
  
  frame_des_right_ = KDL::Frame(
    KDL::Rotation::EulerZYX(msg->data[5],
			    msg->data[4],
			    msg->data[3]),
			    KDL::Vector(msg->data[0],
					msg->data[1],
					msg->data[2]));
  
  xDES_right_(0) = msg->data[0];
  xDES_right_(1) = msg->data[1];
  xDES_right_(2) = msg->data[2];
  xDES_right_(3) = msg->data[3];
  xDES_right_(4) = msg->data[4];
  xDES_right_(5) = msg->data[5];

  
  for(int i=0; i<3; i++)
      x_ref_virtual_.p(i) = xDES_right_(i);
  
  x_ref_virtual_.M = KDL::Rotation::RPY((double)xDES_right_(3),(double)xDES_right_(4),(double)xDES_right_(5));
  
  if((int)msg->data.size() == 12)
  {
    for(int i=0; i<3; i++)
      x_ref_right_elbow_.p(i) = msg->data[6+i];
  
  x_ref_right_elbow_.M = KDL::Rotation::RPY((double)msg->data[9],(double)msg->data[10],(double)msg->data[11]);
  }
//     
  
  x_meas_right_.M.GetEulerZYX(rpy_right_(2),rpy_right_(1),rpy_right_(0));
  jnt_to_jac_solver_right_->JntToJac(q_right_meas_,J_right_last_);  
  
  x_des_right_ = frame_des_right_;
  
  if (primo_right_) {
    primo_right_ = false;
    N = 1000;       // da cambiare in base alla norma dell'errore
    x_meas_right_.M.GetQuaternion(quat_vec_right_(0),quat_vec_right_(1),quat_vec_right_(2),quat_scal_right_);          // quaternione terna attuale
    quat_0_right_ = tf::Quaternion(quat_vec_right_(0),quat_vec_right_(1),quat_vec_right_(2),quat_scal_right_);   
    x_des_right_.M.GetQuaternion(quat_vec_right_(0),quat_vec_right_(1),quat_vec_right_(2),quat_scal_right_);      // quaternione terna desiderata
    quat_f_right_ = tf::Quaternion(quat_vec_right_(0),quat_vec_right_(1),quat_vec_right_(2),quat_scal_right_);
    x0_right_ << x_meas_right_.p(0), x_meas_right_.p(1), x_meas_right_.p(2), 0, 0, 0;           // posizione attuale
    xDES_step_right_ = x0_right_;
    counter = 0;
  }
}


// id_solver_right_->JntToMass(q_right_meas_, M_right_);
// 	id_solver_right_->JntToGravity(q_right_meas_, G_right_);
// 	id_solver_right_->JntToCoriolis(q_right_meas_, qdot_right_meas_, C_right_);
// 	jnt_to_jac_solver_right_->JntToJac(q_right_meas_, J_right_);
// 	fk_pos_solver_right_->JntToCart(q_right_meas_, x_meas_right_);

void DualArms::printKDLTwist(KDL::Twist &twist, const std::string & name)
{
	ROS_INFO_STREAM("KDL Twist (" << name << "): ----------------------------");
	ROS_INFO_STREAM(twist.vel(0) << " " << twist.vel(1) << " " << twist.vel(2));
	ROS_INFO_STREAM(twist.rot(0) << " " << twist.rot(1) << " " << twist.rot(2));
	// ROS_INFO_STREAM(f.p(0) << " " << f.p(1) << " " << f.p(2));
	// double r,p,y;
	// f.M.GetRPY(r,p,y);
	// ROS_INFO_STREAM("roll: " << r << " pitch: " << p << " yaw: " << y);
	ROS_INFO_STREAM("-------------------------------------------------------");
}


void DualArms::printKDLFrame(KDL::Frame &f)
{
	ROS_INFO_STREAM("KDL Frame: --------------------------------------------");
	ROS_INFO_STREAM(f.p(0) << " " << f.p(1) << " " << f.p(2));
	double r,p,y;
	f.M.GetRPY(r,p,y);
	ROS_INFO_STREAM("roll: " << r << " pitch: " << p << " yaw: " << y);
	ROS_INFO_STREAM("-------------------------------------------------------");
}

void DualArms::printKDLJacobian(KDL::Jacobian &J)
{
	ROS_INFO_STREAM("KDL Jacobian: -----------------------------------------");
	ROS_INFO_STREAM("\n" << J.data);
	ROS_INFO_STREAM("-------------------------------------------------------");
}

void DualArms::printJacobianTranspose(KDL::Jacobian &J)
{
	ROS_INFO_STREAM("Eigen Jacobian: -----------------------------------------");
	ROS_INFO_STREAM("\n" << J.data.transpose());
	ROS_INFO_STREAM("-------------------------------------------------------");
}

void DualArms::printKDLInertia(KDL::JntSpaceInertiaMatrix &M)
{
	ROS_INFO_STREAM("KDL Inertia: ------------------------------------------");
	ROS_INFO_STREAM("\n" << M.data);
	ROS_INFO_STREAM("-------------------------------------------------------");
}

void DualArms::printKDLJntArray(KDL::JntArray &A, const std::string & name)
{
	ROS_INFO_STREAM("KDL Array (" << name << "): ------------------------------------------");
	ROS_INFO_STREAM("\n" << A.data);
	ROS_INFO_STREAM("-------------------------------------------------------");
}
