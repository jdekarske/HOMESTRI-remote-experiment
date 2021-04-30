#pragma once

#include <ros/ros.h>
#include <ros/console.h>
#include <rosplan_action_interface/RPActionInterface.h>

// Position
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>

// to actually move
#include <target_pose/target_pose_node.h>

namespace KCL_rosplan
{

	class RPPickup : public RPActionInterface
	{

	private:
		target_pose_node *tp;

	public:
		/* constructor */
		RPPickup(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);
	};
} // namespace KCL_rosplan
