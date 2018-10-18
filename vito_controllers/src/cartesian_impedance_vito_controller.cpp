#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>
#include <utils/pseudo_inversion.h>
 
#include <vito_controllers/cartesian_impedance_vito_controller.h>

namespace vito_controllers { 

CartesianImpedanceVitoController::CartesianImpedanceVitoController() {
    ROS_WARN("VITOVITOVITO Constructor end");
    log_tau_meas.open("tau_meas.csv");
    log_tau_des.open("tau_des.csv");
    log_qerr.open("qerr.csv");
    log_qdot.open("qdot.csv");
    timer = 0;
    ROS_WARN("VITOVITOVITO Constructor end");
}

CartesianImpedanceVitoController::~CartesianImpedanceVitoController() {
    log_tau_meas.close();
    log_tau_des.close();
    log_qerr.close();
    log_qdot.close();
}

Eigen::Matrix<double,3,3> CartesianImpedanceVitoController::skew(Eigen::Matrix<double,3,1> v) {
    Eigen::Matrix<double,3,3> temp;
    temp <<     0, -v(2),  v(1),
             v(2),     0, -v(0),
            -v(1),  v(0),     0;
    return temp;
}

bool CartesianImpedanceVitoController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
	ROS_WARN("VITO init");
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());   
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());
    q_start_.resize(kdl_chain_.getNrOfJoints());

    tau_.resize(kdl_chain_.getNrOfJoints());

//     K_joint.resize(kdl_chain_.getNrOfJoints());
//     D_joint.resize(kdl_chain_.getNrOfJoints());
// 
//     K_joint(0) = 50.0;
//     K_joint(1) = 50.0;
//     K_joint(2) = 10.0;
//     K_joint(3) = 5.0;
//     K_joint(4) = 5.0;
//     K_joint(5) = 5.0;
//     K_joint(6) = 5.0;
// 
//     D_joint(0) = 10;
//     D_joint(1) = 10;
//     D_joint(2) = 7;
//     D_joint(3) = 7;
//     D_joint(4) = 1;
//     D_joint(5) = 1;
//     D_joint(6) = 1;

	K_cart.setZero();
    K_cart(0,0) = 100.0;
    K_cart(1,1) = 1000.0;
    K_cart(2,2) = 1000.0;
    K_cart(3,3) = 10.0;
    K_cart(4,4) = 10.0;
    K_cart(5,5) = 10.0;

    D_cart.setZero();
    D_cart(0,0) = 10.0;
    D_cart(1,1) = 100.0;
    D_cart(2,2) = 100.0;
    D_cart(3,3) = 10.0;
    D_cart(4,4) = 10.0;
    D_cart(5,5) = 10.0;
	
	M_cart.setZero();
    M_cart(0,0) = 1.0;
    M_cart(1,1) = 1.0;
    M_cart(2,2) = 1.0;
    M_cart(3,3) = 1.0;
    M_cart(4,4) = 1.0;
    M_cart(5,5) = 1.0;

    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_des_(i) = joint_handles_[i].getPosition();
        K_(i) = joint_stiffness_handles_[i].getPosition();
        D_(i) = joint_damping_handles_[i].getPosition();
        q_des_(i) = joint_set_point_handles_[i].getPosition();

        tau_(i) = joint_handles_[i].getEffort();
        q_start_(i) = joint_handles_[i].getPosition();
    }

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("stiffness_gains", K_(i) ) ) {
            ROS_WARN("Stiffness gain not set in yaml file, Using %f", K_(i));
        }
    }
    for (int i = 0; i < joint_handles_.size(); ++i) {
        if ( !nh_.getParam("damping_gains", D_(i)) ) {
            ROS_WARN("Damping gain not set in yaml file, Using %f", D_(i));
        }
    }

    typedef  const std_msgs::Float64MultiArray::ConstPtr& msg_type;
    sub_stiffness_ = nh_.subscribe<CartesianImpedanceVitoController, msg_type>("stiffness", 1, boost::bind(&CartesianImpedanceVitoController::setParam, this, _1, &K_, "K"));
    sub_damping_ = nh_.subscribe<CartesianImpedanceVitoController, msg_type>("damping", 1, boost::bind(&CartesianImpedanceVitoController::setParam, this, _1, &D_, "D"));
    sub_add_torque_ = nh_.subscribe<CartesianImpedanceVitoController, msg_type>("additional_torque", 1, boost::bind(&CartesianImpedanceVitoController::setParam, this, _1, &tau_des_, "AddTorque"));
    sub_posture_ = nh_.subscribe("command", 1, &CartesianImpedanceVitoController::command, this);
	sub_posture_cart_ = nh_.subscribe("commandCart", 1, &CartesianImpedanceVitoController::commandCart, this);
	sub_D_ = nh_.subscribe("setD", 1, &CartesianImpedanceVitoController::setD, this);
	sub_K_ = nh_.subscribe("setK", 1, &CartesianImpedanceVitoController::setK, this);
	sub_pose_ = nh_.subscribe("getPose", 1, &CartesianImpedanceVitoController::getPose, this);

    // kinematics and dynamics stuff
    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    KDL::Rotation world_to_base_right_arm = KDL::Rotation::RPY(3.1415926535897932384626, -3.1415926535897932384626/4.0, 0.0);
    id_solver_.reset(new KDL::ChainDynParam(kdl_chain_,world_to_base_right_arm.Inverse()*gravity_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));

    J_ge_.resize(kdl_chain_.getNrOfJoints());
	J_last_.resize(kdl_chain_.getNrOfJoints());
    M_.resize(kdl_chain_.getNrOfJoints());
    C_.resize(kdl_chain_.getNrOfJoints());
    G_.resize(kdl_chain_.getNrOfJoints());

    CartesianForces.resize(6);

    // TODO read config from yaml file

    ROS_WARN("VITO init end");

    return true;
}

void CartesianImpedanceVitoController::starting(const ros::Time& time)
{
    ROS_WARN("VITO starting");
    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        tau_des_(i) = 0.0;
        tau_(i) = joint_handles_[i].getEffort();
        ROS_INFO_STREAM("starting Effort [" << i << "] " << joint_handles_[i].getEffort());
        q_des_(i) = joint_handles_[i].getPosition();
    }

    for (size_t i = 0; i < joint_handles_.size(); i++) {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        ROS_INFO_STREAM("Effort [" << i << "] " << joint_handles_[i].getEffort());
    }
    
    // posizione dei giunti del robot all'inizio della simulazione
    q0_ = q_start_.data;

    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_des_);
    id_solver_->JntToGravity(joint_msr_states_.q, G_);

    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_);
    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_old_);

	cmd_flag_ = 0;
	
	q_des_ct_ << 3.14/4, 3.14/4, 3.14/4, 3.14/4, 3.14/4, 3.14/4, 3.14/4;
	
	int n = 1000;
	Kp_ct_ << n, 0, 0, 0, 0, 0, 0,
			  0, n, 0, 0, 0, 0, 0,
			  0, 0, n, 0, 0, 0, 0,
			  0, 0, 0, n, 0, 0, 0,
			  0, 0, 0, 0, n, 0, 0,
			  0, 0, 0, 0, 0, n, 0,
			  0, 0, 0, 0, 0, 0, n;

	n = 10;
	Kv_ct_ << n, 0, 0, 0, 0, 0, 0,
			  0, n, 0, 0, 0, 0, 0,
			  0, 0, n, 0, 0, 0, 0,
			  0, 0, 0, n, 0, 0, 0,
			  0, 0, 0, 0, n, 0, 0,
			  0, 0, 0, 0, 0, n, 0,
			  0, 0, 0, 0, 0, 0, n;
	counter = 0;
	N = 10000;
	
    ROS_WARN("VITO end starting");

}

void CartesianImpedanceVitoController::update(const ros::Time& time, const ros::Duration& period)
{
    // get joints positions
    for (int i = 0; i < joint_handles_.size(); i++) {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }
    // compute gravity term
    id_solver_->JntToGravity(joint_msr_states_.q, G_);
    id_solver_->JntToCoriolis(joint_msr_states_.q, joint_msr_states_.qdot, C_);
	
	for (size_t i = 0; i < joint_handles_.size(); i++)
        tau_(i) = G_(i) + C_(i);
	
	if (com_torque) {
        if (counter < N) {
            // interpolazione per le posizioni
            q_des_ct_step_ = q_des_ct_step_ + (q_des_ct_- q0_)/N;
            counter++;
        } else {
            q_des_ct_step_ = q_des_ct_;
        }
        //tau_(i) = 0;
		for (size_t i = 0; i < joint_handles_.size(); i++)
			tau_(i) += Kp_ct_(i,i)*(q_des_ct_step_(i) - joint_msr_states_.q(i)) - Kv_ct_(i,i)*joint_msr_states_.qdot(i);
	}

    if (cmd_flag_) {
		id_solver_->JntToMass(joint_msr_states_.q,M_);
		jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_ge_);
		fk_pos_solver_->JntToCart(joint_msr_states_.q, x_meas_);
		xVEC_(0) = x_meas_.p(0);
		xVEC_(1) = x_meas_.p(1);
		xVEC_(2) = x_meas_.p(2);
		x_meas_.M.GetEulerZYX(xVEC_(5),xVEC_(4),xVEC_(3));
		x_meas_.M.GetEulerZYX(rpy(2),rpy(1),rpy(0));
		
		//pseudo_inverse(J_ge_.data,J_ge_pinv_,false);
		
		// velocity error
		e_ref_dot_ = J_ge_.data*joint_msr_states_.qdot.data;
		e_ref_dot_(3) = 0;
		e_ref_dot_(4) = 0;
		e_ref_dot_(5) = 0;
		
		if (counter < N) {
			// interpolazione per le posizioni
			xDES_step_ = xDES_step_ + (xDES_ - x0)/N;
			//interpolazione per l'orientamento
			quat_t = quat_0.slerp(quat_f,counter/N);
			counter++;
		} else {
			if (!primo)
				ROS_INFO("End positioning.");
			xDES_step_ = xDES_;
			quat_t = quat_f;
			primo = true;
		}    
		//position error
		e_ref_ = xDES_step_ - xVEC_;
		
		// Quaternion
		quat_des_vec_(0) = quat_t.x();
		quat_des_vec_(1) = quat_t.y();
		quat_des_vec_(2) = quat_t.z();
		quat_des_scal_ = quat_t.w();
		x_meas_.M.GetQuaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);
		quat_temp_ = quat_scal_*quat_des_vec_ - quat_des_scal_*quat_vec_ - skew(quat_des_vec_)*quat_vec_;
		e_ref_(3) = quat_temp_(0);
		e_ref_(4) = quat_temp_(1);
		e_ref_(5) = quat_temp_(2);
		e_ref_dot_(3) = 0;//(quat_temp_(0) - quat_old_(0))/period.toSec();
		e_ref_dot_(4) = 0;//(quat_temp_(1) - quat_old_(1))/period.toSec();
		e_ref_dot_(5) = 0;//(quat_temp_(2) - quat_old_(2))/period.toSec();
		quat_old_ = quat_temp_;
		
		// jacobian derivative
		J_dot_.data = (J_ge_.data - J_last_.data)/period.toSec(); 
		J_last_ = J_ge_;

		L_ = J_ge_.data*M_.data.inverse()*J_ge_.data.transpose();
		L_ = L_.inverse();

		tau_.data = G_.data + C_.data;

		//for(size_t i = 0; i < joint_handles_.size(); i++)
		//	  tau_(i) = 0;
		
		tau_.data = tau_.data + J_ge_.data.transpose()*(L_*M_cart.inverse()*
							(K_cart*e_ref_ - D_cart*e_ref_dot_ - L_*J_dot_.data*joint_msr_states_.qdot.data));
	}

	for (size_t i = 0; i < joint_handles_.size(); i++)
		joint_handles_[i].setCommand(tau_(i));

	//ROS_INFO_STREAM(xVEC_(0) << "  " << xVEC_(1) << "  " << xVEC_(2));
	
	ros::spinOnce();
    tf::transformKDLToTF( x_des_, tf_ee_pose_);
    br_ee_pose_.sendTransform(tf::StampedTransform(tf_ee_pose_, ros::Time::now(), "right_arm_base_link", "ciao"));
}

void CartesianImpedanceVitoController::command(const std_msgs::Float64MultiArray::ConstPtr &msg) {
    if (msg->data.size() == 0) {
        ROS_INFO("Desired configuration must be: %lu dimension", joint_handles_.size());
    }
    else if ((int)msg->data.size() != joint_handles_.size()) {
        ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
    else
    {
        for (unsigned int j = 0; j < joint_handles_.size(); ++j)
            q_des_(j) = msg->data[j];
    }
}

void CartesianImpedanceVitoController::setK(const std_msgs::Float64MultiArray::ConstPtr &msg) {
	if ((int)msg->data.size() == 6) {
		for (unsigned int j = 0; j < 6; j++)
            K_cart(j,j) = msg->data[j];
		ROS_INFO("New K_cart param: %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf", K_cart(0,0), K_cart(1,1), K_cart(2,2), K_cart(3,3), K_cart(4,4), K_cart(5,5));
	} else if ((int)msg->data.size() == 7) {
		for (unsigned int j = 0; j < 7; j++)
            K_joint(j) = msg->data[j];	
	} else {
        ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
}

void CartesianImpedanceVitoController::setD(const std_msgs::Float64MultiArray::ConstPtr &msg) {
	if ((int)msg->data.size() == 6) {
		for (unsigned int j = 0; j < 6; j++)
            D_cart(j,j) = msg->data[j];
		ROS_INFO("New D_cart param: %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf", D_cart(0,0), D_cart(1,1), D_cart(2,2), D_cart(3,3), D_cart(4,4), D_cart(5,5));
	} else if ((int)msg->data.size() == 7) {
		for (unsigned int j = 0; j < 7; j++)
            D_joint(j) = msg->data[j];	
	} else {
        ROS_ERROR("Posture message had the wrong size: %d", (int)msg->data.size());
        return;
    }
}

void CartesianImpedanceVitoController::getPose(const std_msgs::Float64MultiArray::ConstPtr &msg) {
	ROS_INFO_STREAM(xVEC_(0) << "  " << xVEC_(1) << "  " << xVEC_(2));
}

void CartesianImpedanceVitoController::commandCart(const std_msgs::Float64MultiArray::ConstPtr &msg) {
	if ((int)msg->data.size() != 6) {
        ROS_ERROR("Posture message had the wrong size: %d", 6);
        return;
    }
    
    for (int i = 0; i < joint_handles_.size(); i++) {
    	joint_msr_states_.q(i) = joint_handles_[i].getPosition();
    	joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }
    com_torque = false;
    
    fk_pos_solver_->JntToCart(joint_msr_states_.q,x_meas_);
    
	frame_des_ = KDL::Frame(
					KDL::Rotation::EulerZYX(msg->data[5],
						 			        msg->data[4],
								 	        msg->data[3]),
					KDL::Vector(msg->data[0],
								msg->data[1],
								msg->data[2]));
	xDES_(0) = msg->data[0];
    xDES_(1) = msg->data[1];
    xDES_(2) = msg->data[2];
    xDES_(3) = msg->data[3];
    xDES_(4) = msg->data[4];
    xDES_(5) = msg->data[5];
	
	x_meas_.M.GetEulerZYX(rpy(2),rpy(1),rpy(0));
	jnt_to_jac_solver_->JntToJac(joint_msr_states_.q,J_last_);  
	
	x_des_ = frame_des_;
	cmd_flag_ = 1;
	
	if (primo) {
        primo = false;
        N = 10000;       // da cambiare in base alla norma dell'errore
        x_meas_.M.GetQuaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);          // quaternione terna attuale
        quat_0 = tf::Quaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);   
        x_des_.M.GetQuaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);      // quaternione terna desiderata
        quat_f = tf::Quaternion(quat_vec_(0),quat_vec_(1),quat_vec_(2),quat_scal_);
        x0 << x_meas_.p(0), x_meas_.p(1), x_meas_.p(2), 0, 0, 0;           // posizione attuale
        xDES_step_ = x0;
        counter = 0;
    }
}

void CartesianImpedanceVitoController::setParam(const std_msgs::Float64MultiArray_< std::allocator< void > >::ConstPtr& msg, KDL::JntArray* array, std::string s)
{
    if (msg->data.size() == joint_handles_.size())
    {
        for (unsigned int i = 0; i < joint_handles_.size(); ++i)
        {
            (*array)(i) = msg->data[i];
        }
    }
    else
    {
        ROS_INFO("Num of Joint handles = %lu", joint_handles_.size());
    }

    ROS_INFO("Num of Joint handles = %lu, dimension of message = %lu", joint_handles_.size(), msg->data.size());

    ROS_INFO("New param %s: %.2lf, %.2lf, %.2lf %.2lf, %.2lf, %.2lf, %.2lf", s.c_str(),
             (*array)(0), (*array)(1), (*array)(2), (*array)(3), (*array)(4), (*array)(5), (*array)(6));
}

} // namespace

PLUGINLIB_EXPORT_CLASS( vito_controllers::CartesianImpedanceVitoController, controller_interface::ControllerBase)
