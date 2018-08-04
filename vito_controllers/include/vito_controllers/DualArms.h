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

#include <boost/scoped_ptr.hpp>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>

class DualArms {
public:
	DualArms();
	~DualArms();
	void init();
	void run();
private:
	// Put your control types in the enum Control and define a separate method for each controller
	enum Control {JOINT_IMPEDANCE, CARTESIAN_IMPEDANCE};
	Control control_type;
	void joint_impedance();
	void cartesian_impedance();

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

	ros::Subscriber sub_joint_state_right_arm_;
	ros::Subscriber sub_joint_state_left_arm_;
	
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

	void pub_torques();

	sensor_msgs::JointState msg_right_arm_, msg_left_arm_;

	void compute_torques();

    KDL::Jacobian J_right_, J_left_;

    KDL::JntArray CartesianForces_right_, CartesianForces_left_;

    KDL::JntArray K_joint, D_joint;
    KDL::JntArray K_cart, D_cart;
	
    KDL::JntArray tau_right_, tau_left_;

    KDL::JntArray q_right_ref_, q_left_ref_;

	bool right_arm_alive = false;
    bool left_arm_alive = false;
};

#endif