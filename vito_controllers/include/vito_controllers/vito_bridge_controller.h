#ifndef VITO_CONTROLLERS__VITO_BRIDGE_CONTROLLER_H
#define VITO_CONTROLLERS__VITO_BRIDGE_CONTROLLER_H

#include "KinematicChainControllerBase.h"

#include <visualization_msgs/Marker.h>
#include <sensor_msgs/JointState.h>

#include <boost/scoped_ptr.hpp>

/*
    tau_cmd_ = K_*(q_des_ - q_msr_) + D_*dotq_msr_ + G(q_msr_)

*/

namespace vito_controllers
{

    class VitoBridgeController: public controller_interface::KinematicChainControllerBase<hardware_interface::EffortJointInterface>
    {
    public:

        VitoBridgeController();
        ~VitoBridgeController();

        bool init(hardware_interface::EffortJointInterface *robot, ros::NodeHandle &n);

        void starting(const ros::Time& time);

        void update(const ros::Time& time, const ros::Duration& period);
        
        void receive_torques(const sensor_msgs::JointState &msg);
    private:

        ros::Subscriber sub_command_;

        KDL::JntArray q_des_;
        KDL::JntArray tau_des_;


    };

} // namespace

#endif
