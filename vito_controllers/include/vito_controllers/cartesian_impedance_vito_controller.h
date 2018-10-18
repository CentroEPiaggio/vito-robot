
#ifndef VITO_CONTROLLERS__CARTESIAN_INPEDANCE_CONTROLLER_H
#define VITO_CONTROLLERS__CARTESIAN_INPEDANCE_CONTROLLER_H

#include "vito_controllers/KinematicChainControllerBase.h"

#include <visualization_msgs/Marker.h>
#include <std_msgs/Float64MultiArray.h>

#include <boost/scoped_ptr.hpp>

#include <fstream>

#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_kdl.h>


/*
    tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

namespace vito_controllers
{

    class CartesianImpedanceVitoController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
    {
    public:

        CartesianImpedanceVitoController();
        ~CartesianImpedanceVitoController();
		
		Eigen::Matrix<double,3,3> skew(Eigen::Matrix<double,3,1> v);

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);
        void command(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void commandCart(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void setK(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void setD(const std_msgs::Float64MultiArray::ConstPtr &msg);
		void getPose(const std_msgs::Float64MultiArray::ConstPtr &msg);
        void setParam(const std_msgs::Float64MultiArray::ConstPtr &msg, KDL::JntArray* array, std::string s);
        
    private:

        ros::Subscriber sub_stiffness_, sub_damping_, sub_add_torque_;
        ros::Subscriber sub_posture_, sub_posture_cart_;
		ros::Subscriber sub_D_, sub_K_, sub_pose_;

        KDL::JntArray q_des_;
        KDL::JntArray tau_des_;
        KDL::JntArray K_, D_;
        KDL::JntArray tau_;
        KDL::JntArray q_start_;
		
		int counter;
		Eigen::Matrix<double,6,1> x0;
        Eigen::Matrix<double,6,1> xDES_step_;
		Eigen::Matrix<double,6,1> xDES_;

        boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;
        boost::scoped_ptr<KDL::ChainDynParam> id_solver_;
        boost::scoped_ptr<KDL::ChainFkSolverPos_recursive> fk_pos_solver_;

        KDL::Jacobian J_ge_;    //Jacobian J(q)
        KDL::JntSpaceInertiaMatrix M_;  // intertia matrix
        KDL::JntArray C_;   // coriolis
        KDL::JntArray G_;   // gravity
        
        int cmd_flag_;
        double N;
		bool primo = true;
		Eigen::Matrix<double,7,7> Kp_ct_, Kv_ct_;
		Eigen::Matrix<double,7,1> q_des_ct_, q_des_ct_step_, q0_;
        
        Eigen::Matrix<double,6,1> xVEC_;
		Eigen::Matrix<double,6,6> L_;
		
		Eigen::Matrix<double,3,1> quat_des_vec_;
        Eigen::Matrix<double,3,1> quat_vec_;
        Eigen::Matrix<double,3,1> quat_temp_;
        Eigen::Matrix<double,3,1> quat_old_;
        double quat_des_scal_, quat_scal_;
		tf::Quaternion quat_0;
        tf::Quaternion quat_f;
        tf::Quaternion quat_t = tf::Quaternion(0, 0, 0, 1); 
        
        Eigen::MatrixXd J_ge_pinv_;
		Eigen::Matrix<double,6,1> e_ref_;
        Eigen::Matrix<double,6,1> e_ref_dot_;
		KDL::Jacobian J_last_;	//Jacobian of the last step
		KDL::Jacobian J_dot_;

        KDL::JntArray CartesianForces;
		
		bool com_torque = true;

        KDL::Frame x_des_;  //desired pose
        KDL::Frame x_meas_;  //measured pose (from Direct Kinematics)
        KDL::Frame x_meas_old_;  //measured pose (from Direct Kinematics)
        
        KDL::Frame x_;
		Eigen::Matrix<double,3,1> rpy;

        std::ofstream log_tau_meas, log_tau_des;
        std::ofstream log_qerr, log_qdot;
        long int timer;

        KDL::JntArray K_joint, D_joint;
        //KDL::JntArray K_cart, D_cart, M_cart;
		Eigen::Matrix<double,6,6> K_cart, D_cart, M_cart;

        // Rviz interface stuff
        tf::TransformBroadcaster br_ee_pose_;
        tf::Transform tf_ee_pose_;
        KDL::Frame frame_des_;


    };

} // namespace

#endif
