#include <RPDrop.h>

namespace KCL_rosplan
{
	RPDrop::RPDrop(ros::NodeHandle &nh)
	{
		tp = new target_pose_node();
	}

	/* action dispatch callback */
	bool RPDrop::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
	{
		// complete the action
		ROS_INFO("KCL: (%s) Action starting.", msg->name.c_str());

		return tp->moveGripper(target_pose_node::Gripper::OPEN); // z axis adjustment from here /catkin_ws/src/experiment_world/launch/spawn_robot.launch I don't think `setPoseRefenceFrame()` is working
	}
} // namespace KCL_rosplan

int main(int argc, char **argv)
{

	ros::init(argc, argv, "rosplan_ur_drop_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");

	// definitely necessary...TODO
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// create PDDL action subscriber
	KCL_rosplan::RPDrop rpd(nh);
	rpd.runActionInterface();

	ros::waitForShutdown();
	return 0;
}