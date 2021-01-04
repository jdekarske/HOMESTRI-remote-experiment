#include <RPGoto.h>

/* The implementation of RPTutorial.h */
namespace KCL_rosplan
{
	RPGoto::RPGoto(ros::NodeHandle &nh)
	{
		tp = new target_pose_node();
		tp->initPosition();
	}

	/* action dispatch callback */
	bool RPGoto::concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
	{
		// complete the action (test)
		ROS_INFO("KCL: (%s) TUTORIAL Action starting.", msg->name.c_str());

		// std::map<std::string, double> coords = getWaypointCoordinates(msg);

		return tp->move(coords["x"], coords["y"], coords["z"]);
	}

	std::map<std::string, double> RPGoto::getWaypointCoordinates(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg)
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
} // namespace KCL_rosplan

/*-------------*/
/* Main method */
/*-------------*/

int main(int argc, char **argv)
{

	ros::init(argc, argv, "rosplan_ur_goto_action", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");

	// definitely necessary...TODO
	ros::AsyncSpinner spinner(2);
	spinner.start();

	// create PDDL action subscriber
	KCL_rosplan::RPGoto rpg(nh);
	rpg.runActionInterface();

	ros::waitForShutdown();
	return 0;
}