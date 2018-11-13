#ifndef VITO_CONTROLLERS_DUAL_ARMS_H__
#define VITO_CONTROLLERS_DUAL_ARMS_H__

#include <iostream>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <ros/topic.h>

#include <urdf/model.h>

#include <kdl/tree.hpp>
#include <kdl/kdl.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/chaindynparam.hpp> //this to compute the gravity vector
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>
#include <Eigen/Dense>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

class DualArms {
public:
	DualArms();
	~DualArms();
	void init();
	void run();
	
	Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1> v);
private:
	// Put your control types in the enum Control and define a separate method for each controller
	enum Control {JOINT_IMPEDANCE, CARTESIAN_IMPEDANCE};
	Control control_type;
	void joint_impedance();
	void cartesian_impedance();
	void cartesian_impedance_lorale();

	
	//
	enum Arm {RIGHT, LEFT};

	// robot model loading stuff
	bool load_robot(const Arm& arm);
	KDL::Vector gravity_; 
	KDL::Chain kdl_chain_, kdl_chain_right_, kdl_chain_left_;
	KDL::Rotation world_to_base_right_arm;
	KDL::Rotation world_to_base_left_arm;
	struct limits_
	{
		KDL::JntArray min;
		KDL::JntArray max;
		KDL::JntArray center;
	} joint_limits_, joint_limits_right_, joint_limits_left_;

	size_t number_arm_joints; // for each arm

	ros::NodeHandle nh, nh_params_right, nh_params_left;

    void joint_state_callback_left(const sensor_msgs::JointState& msg);
    void joint_state_callback_right(const sensor_msgs::JointState& msg);
    
    void commandCart_right_(const std_msgs::Float64MultiArray::ConstPtr &msg);

	ros::Subscriber sub_joint_state_right_arm_;
	ros::Subscriber sub_joint_state_left_arm_;
	ros::Subscriber sub_command_right_;
	
	KDL::JntArray q_left_meas_,	qdot_left_meas_,	tau_left_meas_;
	KDL::JntArray q_right_meas_, qdot_right_meas_, tau_right_meas_;

    boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_right_, jnt_to_jac_solver_left_;
    boost::scoped_ptr<KDL::ChainDynParam> id_solver_right_, id_solver_left_;
    boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_right_, fk_pos_solver_left_;

    KDL::Frame x_right_, x_left_;  // end effector pose

    // Rviz interface
    tf::TransformBroadcaster br_ee_pose_right_, br_ee_pose_left_;
    tf::Transform tf_ee_pose_right_, tf_ee_pose_left_;

    void pub_ee_pose();

    // control publisher
    ros::Publisher pub_torques_right_arm_;
	ros::Publisher pub_torques_left_arm_;
	ros::Publisher pub_error_right_arm_;

	void pub_torques();
	void pub_error();

	sensor_msgs::JointState msg_right_arm_, msg_left_arm_;

	void compute_torques();

    KDL::Jacobian J_right_, J_left_;
	KDL::Jacobian J_right_last_, J_left_last_;
	KDL::Jacobian J_right_dot_, J_left_dot_;
	
    KDL::JntArray CartesianForces_right_, CartesianForces_left_;

    KDL::JntArray K_joint, D_joint;
    KDL::JntArray K_cart, D_cart;
	Eigen::Matrix<double,6,6> K_cart_imp_right, D_cart_imp_right, M_cart_imp_right;
	Eigen::Matrix<double,6,6> K_cart_imp_left, D_cart_imp_left, M_cart_imp_left;
	
    KDL::JntArray tau_right_, tau_left_;

    KDL::JntArray q_right_ref_, q_left_ref_;

	bool right_arm_alive = false;
    bool left_arm_alive = false;
	
	Eigen::Matrix<double,6,1> xVEC_right_;
	Eigen::Matrix<double,6,6> L_right_;
	
	Eigen::Matrix<double,3,1> quat_des_vec_right_;
    Eigen::Matrix<double,3,1> quat_vec_right_;
    Eigen::Matrix<double,3,1> quat_temp_right_;
    Eigen::Matrix<double,3,1> quat_old_right_;
    double quat_des_scal_right_, quat_scal_right_;
	tf::Quaternion quat_0_right_;
    tf::Quaternion quat_f_right_;
    tf::Quaternion quat_t_right_ = tf::Quaternion(0, 0, 0, 1); 

	Eigen::Matrix<double,6,1> e_ref_right_;
    Eigen::Matrix<double,6,1> e_ref_dot_right_;
	
	int counter;
	int N;
	Eigen::Matrix<double,6,1> x0_right_;
    Eigen::Matrix<double,6,1> xDES_step_right_;
	Eigen::Matrix<double,6,1> xDES_right_;
	bool primo_right_ = true;
	bool primo_left_ = true;
	
	KDL::JntSpaceInertiaMatrix M_right_;
	KDL::JntSpaceInertiaMatrix M_left_;
	
	KDL::Frame x_des_right_;  //desired pose
    KDL::Frame x_meas_right_;  //measured pose (from Direct Kinematics)
    KDL::Frame x_meas_old_right_;  //measured pose (from Direct Kinematics)
    
    KDL::Frame x_des_left_;  //desired pose
    KDL::Frame x_meas_left_;  //measured pose (from Direct Kinematics)
    KDL::Frame x_meas_old_left_;  //measured pose (from Direct Kinematics)
    
    Eigen::Matrix<double,3,1> rpy_right_;
	Eigen::Matrix<double,3,1> rpy_left_;
	
	ros::Duration period;
	ros::Time time_prec_;
	KDL::Frame frame_des_right_;
	KDL::Frame frame_des_left_;
	bool starting = true;
	
	KDL::JntArray C_right_;   // coriolis
    KDL::JntArray G_right_;   // gravity

	// utils functions
	void printKDLTwist(KDL::Twist &twist, const std::string & name = "");
	void printKDLFrame(KDL::Frame &f);
	void printKDLJacobian(KDL::Jacobian &J);
	void printJacobianTranspose(KDL::Jacobian &J);
	void printKDLInertia(KDL::JntSpaceInertiaMatrix &M);
	void printKDLJntArray(KDL::JntArray &A, const std::string & name);

	// cartesian impedance control stuff (Dan)
	KDL::Frame x_ref_virtual_;
	bool virtual_ref_available = false;

};

#endif