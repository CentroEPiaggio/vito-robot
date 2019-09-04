
#include <vito_controllers/teleoperation_controller.h>
#include <utils/pseudo_inversion.h>
#include <utils/skew_symmetric.h>
#include "utils/kuka_lwr_utilities.h"

#include <pluginlib/class_list_macros.h>
#include <kdl_parser/kdl_parser.hpp>
#include <Eigen/LU>

#include <math.h>

#include <std_msgs/Float64MultiArray.h>
#include <tf_conversions/tf_kdl.h>
#include <kdl/frames_io.hpp>

namespace vito_controllers
{
TeleoperationController::TeleoperationController() {}
TeleoperationController::~TeleoperationController() {}

bool TeleoperationController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    if ( !(KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n)) )
    {
        ROS_ERROR("Couldn't initialize OneTaskInverseKinematics controller.");
        return false;
    }

    jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
    fk_pos_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
    ik_vel_solver_.reset(new KDL::ChainIkSolverVel_pinv(kdl_chain_));
    ik_pos_solver_.reset(new KDL::ChainIkSolverPos_NR_JL(kdl_chain_, joint_limits_.min, joint_limits_.max, *fk_pos_solver_, *ik_vel_solver_));

    q_cmd_.resize(kdl_chain_.getNrOfJoints());
    J_.resize(kdl_chain_.getNrOfJoints());


    K_.resize(kdl_chain_.getNrOfJoints());
    D_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());

    std::string listen_topic;
    nh_.param<std::string>("listen_topic", listen_topic, "command_twist");
    sub_command_ = nh_.subscribe("command", 1, &TeleoperationController::cb_command, this);
    sub_command_twist = nh_.subscribe(listen_topic, 1, &TeleoperationController::cb_twist, this);
    sub_command2 = nh_.subscribe("command2", 1, &TeleoperationController::cb_command2, this);
    sub_start_controller = nh_.subscribe("start_controller", 1, &TeleoperationController::cb_startController, this);
    stiffness_topic = nh_.subscribe("stiffness_scale", 1, &TeleoperationController::cb_setStiffness, this);
    sub_home_ = nh_.subscribe("home", 1, &TeleoperationController::cb_home, this);

    pub_error = nh_.advertise<std_msgs::Float64MultiArray>("error", 1000);
    pub_error2 = nh_.advertise<std_msgs::Float64MultiArray>("error2", 1000);
    pub_twist_error = nh_.advertise<geometry_msgs::Twist>("twist_error", 1000);

    nh_.param<double>("alpha1", alpha1, 1);
    nh_.param<double>("alpha2", alpha2, 1);
    nh_.param<double>("stiffness_max", stiffness_max_, 1000);

    return true;
}

void TeleoperationController::starting(const ros::Time& time)
{
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
        joint_des_states_.q(i) = joint_msr_states_.q(i);
        joint_des_states_.qdot(i) = 0.0;
        tau_des_(i) = 0.0;
        K_(i) = stiffness_max_;
        D_(i) = joint_damping_handles_[i].getPosition();
    }

    // computing forward kinematics
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);

    position_desired_flag_ =  false;
    twist_desired_flag_ =  false;
    second_task_ = false;
    activate_controller_ = false;

    twist_desired.Zero();
    twist_measured_old.Zero();
    twist_error.Zero();
    twist_i_error.Zero();
}

void TeleoperationController::update(const ros::Time& time, const ros::Duration& period)
{

    nh_.param<double>("alpha1", alpha1, 8.0);
    nh_.param<double>("alpha2", alpha2, 1.0);
    
    std_msgs::Float64MultiArray error_msg;
    std_msgs::Float64MultiArray error_msg2;
    KDL::Twist x_err_2;
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_msr_states_.q(i) = joint_handles_[i].getPosition();
        joint_msr_states_.qdot(i) = joint_handles_[i].getVelocity();
    }

    jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_);
    fk_pos_solver_->JntToCart(joint_msr_states_.q, x_);

    // computing J_pinv_
    pseudo_inverse(J_.data, J_pinv_);

    if (activate_controller_)
    {
        switch (control_mode_)
        {
        case 1:
        {
            // end-effector position error

            x_err_.vel = x_des_.p - x_.p;
            x_.M.GetQuaternion(quat_curr_.v(0), quat_curr_.v(1), quat_curr_.v(2), quat_curr_.a);

            x_des_.M.GetQuaternion(quat_des_.v(0), quat_des_.v(1), quat_des_.v(2), quat_des_.a);

            skew_symmetric(quat_des_.v, skew_);

            for (int i = 0; i < skew_.rows(); i++)
            {
                v_temp_(i) = 0.0;
                for (int k = 0; k < skew_.cols(); k++)
                    v_temp_(i) += skew_(i, k) * (quat_curr_.v(k));
            }

            x_err_.rot = quat_curr_.a * quat_des_.v - quat_des_.a * quat_curr_.v - v_temp_;

            Eigen::MatrixXd epsilon_hat = skew_symmetric(quat_des_.v);
            Eigen::MatrixXd orientation_error = Eigen::MatrixXd::Zero(3, 1);
            double eta_d = quat_des_.a;
            double eta = quat_curr_.a;
            Eigen::MatrixXd actual_quaternion = Eigen::MatrixXd::Zero(3, 1);
            Eigen::MatrixXd desired_quaternion = Eigen::MatrixXd::Zero(3, 1);

            actual_quaternion << quat_curr_.v(0), quat_curr_.v(1), quat_curr_.v(2);
            desired_quaternion << quat_des_.v(0), quat_des_.v(1), quat_des_.v(2);

            orientation_error = eta_d * actual_quaternion - eta * desired_quaternion + epsilon_hat * actual_quaternion;

            KDL::Twist xerr;
            xerr.vel = x_des_.p - x_.p;
            xerr.rot = 0.5 * (  x_.M.UnitX() * x_des_.M.UnitX() +
                                x_.M.UnitY() * x_des_.M.UnitY() +
                                x_.M.UnitZ() * x_des_.M.UnitZ());

            Eigen::MatrixXd qp_des_attractive = Eigen::MatrixXd::Zero(7, 1);
            Eigen::MatrixXd x_err_temp_eigen = Eigen::MatrixXd::Zero(6, 1);
            x_err_temp_eigen << x_err_.vel.data[0],  x_err_.vel.data[1],  x_err_.vel.data[2],
                             orientation_error(0), orientation_error(1), orientation_error(2);
            qp_des_attractive = alpha1 * J_pinv_ * x_err_temp_eigen;

            KDL::Twist x_err_temp;
            x_err_temp =  diff(x_, x_des_);

            x_err_.rot = x_err_temp.rot;
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                    joint_des_states_.qdot(i) += alpha1 * J_pinv_(i, k) * x_err_(k); //removed scaling factor of .7
            }

            KDL::Frame x_2;
            fk_pos_solver_->JntToCart(joint_msr_states_.q, x_2, 6);

            KDL::Jacobian J_2;
            J_2.resize(kdl_chain_.getNrOfJoints());
            jnt_to_jac_solver_->JntToJac(joint_msr_states_.q, J_2, 6);

            if (second_task_)
            {
                Eigen::Matrix<double, 7, 7> P;
                P =  Eigen::Matrix<double, 7, 7>::Identity() - J_pinv_ * J_.data;

                Eigen::Matrix<double, 7, 1> q_null;
                Eigen::MatrixXd J_pinv_2;

                Eigen::Matrix<double, 3, 7> J_2_short = Eigen::Matrix<double, 3, 7>::Zero();
                J_2_short = J_2.data.block<3, 7>(0, 0);
                pseudo_inverse(J_2_short, J_pinv_2);
                Eigen::Matrix<double, 7, 3> NullSpace = Eigen::Matrix<double, 7, 3>::Zero();
                NullSpace = P * J_pinv_2;

                x_err_2.vel = x_des_2.p - x_2.p;

                Eigen::MatrixXd x_err_2_eigen = Eigen::MatrixXd::Zero(3, 1);
                x_err_2_eigen << x_err_2.vel(0), x_err_2.vel(1), x_err_2.vel(2);

                q_null = alpha2 * NullSpace * x_err_2_eigen; //removed scaling factor of .7

                for (int i = 0; i < J_pinv_2.rows(); i++)
                {
                    joint_des_states_.qdot(i) += alpha2 * q_null[i]; //removed scaling factor of .7
                }
            }
            for (unsigned int i = 0; i < 6; ++i)
            {
                error_msg.data.push_back(x_err_(i));
                error_msg2.data.push_back(x_err_2(i));
            }

            pub_error.publish(error_msg);
            pub_error2.publish(error_msg2);
        }
        break;
        case 2:
        {

            for (int i = 0; i < J_.rows(); i++)
            {
                twist_measured(i) = 0.0;
                for (int k = 0; k < J_.columns(); k++)
                {
                    twist_measured(i) += J_(i, k) * joint_msr_states_.qdot(k); //removed scaling factor of .7
                }
            }
            twist_error = twist_desired - twist_measured;
            twist_i_error += twist_error * period.toSec();
            for (int i = 0; i < J_pinv_.rows(); i++)
            {
                joint_des_states_.qdot(i) = 0.0;
                for (int k = 0; k < J_pinv_.cols(); k++)
                {
                    // joint_des_states_.qdot(i) += J_pinv_(i, k) * twist_desired(k); //removed scaling factor of .7
                    joint_des_states_.qdot(i) += J_pinv_(i, k) * (alpha1 * twist_error(k) + alpha2 * twist_i_error(k)); //removed scaling factor of .7
                }
            }

            geometry_msgs::Twist msg_twist_error;
            msg_twist_error.linear.x = twist_error(0);
            msg_twist_error.linear.y = twist_error(1);
            msg_twist_error.linear.z = twist_error(2);
            msg_twist_error.angular.x = twist_error(3);
            msg_twist_error.angular.y = twist_error(4);
            msg_twist_error.angular.z = twist_error(5);
            pub_twist_error.publish(msg_twist_error);
        }
        break;
        case 3:
        {
            for (int k = 0; k < joint_handles_.size(); k++)
            {
                    joint_des_states_.qdot(k) = 0.1 * alpha1 * ( q_home[k] - joint_msr_states_.q(k) )/period.toSec() ;
            }
        }
        break;
        default:
            break;
        }

        saturateJointVelocities(joint_des_states_.qdot);

        for (unsigned  int i = 0; i < joint_handles_.size(); i++)
        {
            joint_des_states_.q(i) += period.toSec() * joint_des_states_.qdot(i);
        }

        saturateJointPositions(joint_des_states_.q);

    }
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        joint_handles_[i].setCommand(0.0);
        joint_set_point_handles_[i].setCommand(joint_des_states_.q(i));
        joint_stiffness_handles_[i].setCommand(K_(i));
        joint_damping_handles_[i].setCommand(D_(i));

    }

}

void TeleoperationController::cb_setStiffness(const std_msgs::Float64_< std::allocator< void > >::ConstPtr& msg)
{
    for (unsigned int i = 0; i < joint_handles_.size(); i++)
    {
        K_(i) = msg->data * stiffness_max_;
    }

}

void TeleoperationController::cb_command(const geometry_msgs::Pose::ConstPtr &msg)
{
    KDL::Frame frame_des_;


    frame_des_ = KDL::Frame(
                     KDL::Rotation::Quaternion(msg->orientation.x,
                             msg->orientation.y,
                             msg->orientation.z,
                             msg->orientation.w),
                     KDL::Vector(msg->position.x,
                                 msg->position.y,
                                 msg->position.z));

    x_des_ = frame_des_;

    tf::Transform CollisionTransform;
    tf::transformKDLToTF( frame_des_, CollisionTransform);
    tf_desired_hand_pose.sendTransform( tf::StampedTransform( CollisionTransform, ros::Time::now(), "world", "reference") );
    control_mode_ = 1;
}

void TeleoperationController::cb_home(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    if(msg->data.size()!=joint_handles_.size())
    {
        ROS_ERROR("WRONG SIZE");
    }

    for (int i=0;i<7;i++)
    {
        q_home[i] = msg->data[i];
    }

    control_mode_ = 3;
}

void TeleoperationController::cb_twist(const geometry_msgs::Twist::ConstPtr &msg)
{
    twist_desired.vel(0) = msg->linear.x;
    twist_desired.vel(1) = msg->linear.y;
    twist_desired.vel(2) = msg->linear.z;
    twist_desired.rot(0) = msg->angular.x;
    twist_desired.rot(1) = msg->angular.y;
    twist_desired.rot(2) = msg->angular.z;
    twist_i_error.Zero();
    double sum_t(0.0);
    for (int i=0;i<5;++i)
    {
        sum_t += twist_desired(i);
    }
    if (sum_t == 0.0)
    {
        fk_pos_solver_->JntToCart(joint_msr_states_.q, x_des_);
        control_mode_ = 1;
        return;
    }
    control_mode_ = 2;

}

void TeleoperationController::cb_command2(const geometry_msgs::Pose::ConstPtr &msg)
{
    KDL::Frame frame_des_;

    frame_des_ = KDL::Frame(
                     KDL::Rotation::Quaternion(msg->orientation.x,
                             msg->orientation.y,
                             msg->orientation.z,
                             msg->orientation.w),
                     KDL::Vector(msg->position.x,
                                 msg->position.y,
                                 msg->position.z));

    x_des_2 = frame_des_;
    second_task_ =  true;

    tf::Transform CollisionTransform2;
    tf::transformKDLToTF( frame_des_, CollisionTransform2);
    elbow_reference.sendTransform( tf::StampedTransform( CollisionTransform2, ros::Time::now(), "world", "elbow_reference") );
}


void TeleoperationController::cb_startController(const std_msgs::Bool::ConstPtr& msg)
{
    activate_controller_ = msg->data;
    return;
}


}

PLUGINLIB_EXPORT_CLASS(vito_controllers::TeleoperationController, controller_interface::ControllerBase)
