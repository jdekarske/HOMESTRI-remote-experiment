#include "RPur.h"

/* The implementation of RPTutorial.h */
namespace KCL_rosplan
{

	/* constructor */
	RPur::RPur(ros::NodeHandle &nh)
	{
		// perform setup
		manipulator_group = new moveit::planning_interface::MoveGroupInterface("manipulator");
		gripper_group = new moveit::planning_interface::MoveGroupInterface("gripper");

		// 	// move to neutral
		manipulator_group->setNamedTarget("vertical");
		manipulator_group->move();
	}

	/* action dispatch callback */
	bool RPur::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
	{
		// complete the action (test)
		ROS_INFO("KCL: (%s) TUTORIAL Action completing.", msg->name.c_str());
		return move(0.25, -0.29, 0.20);
	}

	bool RPur::move(float x_des, float y_des, float z_des)
	{
		geometry_msgs::Pose target_pose;
		target_pose.orientation.x = 0.707;
		target_pose.orientation.y = 0.0;
		target_pose.orientation.z = -.707;
		target_pose.orientation.w = 0.0;
		target_pose.position.x = x_des;
		target_pose.position.y = y_des;
		target_pose.position.z = z_des;
		manipulator_group->setPoseTarget(target_pose, "flange");

		bool success = (manipulator_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
		return success;
	}
} // namespace KCL_rosplan

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{

	ros::init(argc, argv, "rosplan_tutorial_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");

	// definitely necessary...TODO
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// create PDDL action subscriber
	KCL_rosplan::RPur rpti(nh);
	rpti.runActionInterface();

	return 0;
}