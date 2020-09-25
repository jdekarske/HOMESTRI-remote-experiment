// ROS
#include <ros/ros.h>
#include <ros/console.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// Position
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"

// Msg
#include <targetpose/pickplace.h>
#include <gazebo_msgs/GetModelState.h>

const float pick_height = 0.15;
const float place_height = 0.23;
const float approach_height = 0.3;
const float open_position = 0.0;
const float closed_position = 0.25;

moveit::planning_interface::MoveGroupInterface *manipulator_group;
moveit::planning_interface::MoveGroupInterface *gripper_group;

bool moveGripper(double pos)
{
  gripper_group->setJointValueTarget("gripper_finger1_joint", pos);
  bool success = (gripper_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ros::Duration(0.7).sleep(); //wait for the object to attach
  return success;
}

bool move(float x_des, float y_des, float z_des)
{
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.707;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = -.707;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = x_des;
  target_pose1.position.y = y_des;
  target_pose1.position.z = z_des;
  manipulator_group->setPoseTarget(target_pose1, "flange");

  bool success = (manipulator_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;
}

bool pick(float objx, float objy, float objz)
{

  bool success = true && move(objx, objy, objz + approach_height) && moveGripper(open_position) && move(objx, objy, objz + pick_height) && moveGripper(closed_position) && move(objx, objy, objz + approach_height);
  return success;
}

bool pick(std::string object_name)
{
  gazebo_msgs::GetModelState model;
  model.request.model_name = object_name;
  model.request.relative_entity_name = "robot";

  if (ros::service::call("/gazebo/get_model_state", model))
  {
    geometry_msgs::Point model_position = model.response.pose.position;
    return pick(model_position.x, model_position.y, model_position.z);
  }
  else
  {
    ROS_ERROR("Failed to get model state");
    return false;
  }
}

bool place(float objx, float objy, float objz)
{
  bool success = true && move(objx, objy, objz + approach_height) && move(objx, objy, objz + place_height) && moveGripper(open_position) && move(objx, objy, objz + approach_height);
  return true;
}

bool place(std::string object_name)
{
  gazebo_msgs::GetModelState model;
  model.request.model_name = object_name;
  model.request.relative_entity_name = "robot";

  if (ros::service::call("/gazebo/get_model_state", model))
  {
    geometry_msgs::Point model_position = model.response.pose.position;
    return place(model_position.x, model_position.y, model_position.z);
  }
  else
  {
    ROS_ERROR("Failed to get model state");
    return false;
  }
}

bool pickplaceCallback(targetpose::pickplace::Request &req, targetpose::pickplace::Response &res)
{
  res.status = pick(req.pick_object) && place(req.place_object); // this is poetic
  return res.status;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "UR5e_pick_place");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("pick_place", pickplaceCallback);

  ros::AsyncSpinner spinner(2);
  spinner.start();

  manipulator_group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  gripper_group = new moveit::planning_interface::MoveGroupInterface("gripper");
  
  ros::Duration(2).sleep();
  manipulator_group->setNamedTarget("vertical");
  manipulator_group->move();

  // manipulator_group->setPlanningTime(45.0); // Not sure why this was here

  // (tests)
  // pick(0.7, 0);
  // place(0, 0.7);

  ros::waitForShutdown();
  return 0;
}
