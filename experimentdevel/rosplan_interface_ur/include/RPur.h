#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <math.h>
#include "rosplan_knowledge_msgs/KnowledgeItem.h"
#include "rosplan_knowledge_msgs/KnowledgeUpdateService.h"
#include "rosplan_dispatch_msgs/ActionDispatch.h"
#include "rosplan_dispatch_msgs/ActionFeedback.h"
#include "rosplan_action_interface/RPActionInterface.h"

// ROS
#include <ros/console.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Position
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

// Msg
#include <gazebo_msgs/GetModelState.h>

#ifndef KCL_RPUR
#define KCL_RPUR

namespace KCL_rosplan
{

	class RPur : public RPActionInterface
	{

	private:
		const float pick_height = 0.15;
		const float place_height = 0.23;
		const float approach_height = 0.3;
		const float open_position = 0.0;
		const float closed_position = 0.25;

		moveit::planning_interface::MoveGroupInterface *manipulator_group;
		moveit::planning_interface::MoveGroupInterface *gripper_group;

		bool move(float x_des, float y_des, float z_des);

	public:
		/* constructor */
		RPur(ros::NodeHandle &nh);

		/* listen to and process action_dispatch topic */
		bool concreteCallback(const rosplan_dispatch_msgs::ActionDispatch::ConstPtr &msg);
	};
} // namespace KCL_rosplan

#endif