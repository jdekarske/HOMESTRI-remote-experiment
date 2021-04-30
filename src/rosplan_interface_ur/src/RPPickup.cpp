#include <RPPickup.h>

namespace KCL_rosplan
{
	RPPickup::RPPickup(ros::NodeHandle &nh)
	{
		tp = new target_pose_node();
	}

	/* action dispatch callback */
	bool RPPickup::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
	{
		// complete the action
		ROS_INFO("KCL: (%s) Action starting.", msg->name.c_str());

		return tp->moveGripper(target_pose_node::Gripper::CLOSE);
	}
} // namespace KCL_rosplan

int main(int argc, char **argv)
{

	ros::init(argc, argv, "rosplan_ur_pickup_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");

	// definitely necessary...TODO
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// create PDDL action subscriber
	KCL_rosplan::RPPickup rpp(nh);
	rpp.runActionInterface();

	ros::waitForShutdown();
	return 0;
}