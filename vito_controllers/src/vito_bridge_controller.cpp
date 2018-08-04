#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <kdl/tree.hpp>
#include <kdl/chainfksolvervel_recursive.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include <vito_controllers/vito_bridge_controller.h>

namespace vito_controllers {

VitoBridgeController::VitoBridgeController() {}

VitoBridgeController::~VitoBridgeController() {}

bool VitoBridgeController::init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n)
{
    KinematicChainControllerBase<hardware_interface::EffortJointInterface>::init(robot, n);
    q_des_.resize(kdl_chain_.getNrOfJoints());
    tau_des_.resize(kdl_chain_.getNrOfJoints());
 
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        tau_des_(i) = 0.0;;
        q_des_(i) = joint_set_point_handles_[i].getPosition();
    }

    ROS_DEBUG(" Number of joints in handle = %lu", joint_handles_.size() );

    sub_command_  = n.subscribe("command_torques",1,&VitoBridgeController::receive_torques,this);

    return true;


}

void VitoBridgeController::starting(const ros::Time& time)
{
    // Initializing stiffness, damping, ext_torque and set point values
    for (size_t i = 0; i < joint_handles_.size(); i++) {
        tau_des_(i) = 0.0;
        q_des_(i) = joint_handles_[i].getPosition();
    }


}

void VitoBridgeController::update(const ros::Time& time, const ros::Duration& period)
{

    //Compute control law. This controller sets all variables for the JointImpedance Interface from kuka
    for (size_t i = 0; i < joint_handles_.size(); i++)
    {
        // we use tau for the control and set the other references to zero
        joint_handles_[i].setCommand(tau_des_(i));
        joint_stiffness_handles_[i].setCommand(0.0);
        joint_damping_handles_[i].setCommand(0.0);
        joint_set_point_handles_[i].setCommand(joint_handles_[i].getPosition());
    }

}

void VitoBridgeController::receive_torques(const sensor_msgs::JointState &msg)
{
    for(size_t i=0; i<joint_handles_.size(); i++)
        tau_des_(i) = msg.effort.at(i);
}

} // namespace

PLUGINLIB_EXPORT_CLASS( vito_controllers::VitoBridgeController, controller_interface::ControllerBase)
