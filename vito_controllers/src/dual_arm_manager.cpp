#include "vito_controllers/DualArms.h"

int main(int argc, char **argv)
{
	ros::init(argc,argv,"dual_arm_manager");
	ROS_INFO("Dual Arm Manager startup");

	// dual_arm::DualArmKinematics right_arm;
	// dual_arm::DualArmKinematics left_arm;
	DualArms da;

	da.init();

	sleep(1);
	da.run();

	ROS_INFO("Dual Arm Manager: clean exit");
	return 0;
}

void joint_state_callback_right_arm()
{
	ROS_INFO("Right arm status received!");
}

void joint_state_callback_left_arm()
{
	ROS_INFO("Left arm status received!");
}