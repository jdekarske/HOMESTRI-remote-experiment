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
		ROS_INFO("KCL: (%s) TUTORIAL Action starting.", msg->name.c_str());

		std::map<std::string, double> coords = getWaypointCoordinates(msg);

		return move(coords["x"], coords["y"], coords["z"]);
	}

	std::map<std::string, double> RPur::getWaypointCoordinates(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
	{
		// msg->parameters is probably:
		// v-arm
		// from - home
		// to - cin1
		std::map<std::string, double> coords; // for coordinates
		std::string param_root = "/blockpositions";
		std::string stupidinputname = "/cube"; // I'm frustrated with the lack of templating in getparam api

		for (const auto &arg : msg->parameters)
		{
			if (arg.key == "to")
			{
				if (arg.value.find("in") != std::string::npos)
				{
					// input waypoint
					ROS_DEBUG("Rosplan interface moving to input");
					param_root.append("/inputs");
				}
				else if (arg.value.find("out") != std::string::npos)
				{
					// output waypoint
					ROS_DEBUG("Rosplan interface moving to output");
					param_root.append("/outputs");
				}
				else
				{
					break;
				}
				ros::param::get(param_root + stupidinputname + arg.value.back(), coords);
			}
		}

		return coords;
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

	ros::waitForShutdown();
	return 0;
}