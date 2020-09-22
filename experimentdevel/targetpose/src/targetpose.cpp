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

const float pick_height = 0.28;
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

bool pick(float objx, float objy)
{
  bool success = true && move(objx, objy, pick_height) && moveGripper(open_position) && move(objx, objy, pick_height - 0.1) && moveGripper(closed_position) && move(objx, objy, pick_height);
  return success;
}

bool place(float objx, float objy)
{
  bool success = true && move(objx, objy, pick_height) && move(objx, objy, pick_height - 0.1) && moveGripper(open_position) && move(objx, objy, pick_height);
  return true;
}

bool pickplaceCallback(targetpose::pickplace::Request &req, targetpose::pickplace::Response &res)
{
  // find out where the cube is
  gazebo_msgs::GetModelState model;
  model.request.model_name = req.pick_object;
  model.request.relative_entity_name = "robot";
  if (ros::service::call("/gazebo/get_model_state", model))
  {
    geometry_msgs::Point model_position = model.response.pose.position;
    res.status = pick(model_position.x, model_position.y) && place(req.place_position.x, req.place_position.y); // this is poetic
    return res.status;
  }
  else
  {
    ROS_ERROR("Failed to get model state");
    res.status = false;
    return res.status;
  }
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

  // manipulator_group->setPlanningTime(45.0); // Not sure why this was here

  // (tests)
  // pick(0.7, 0);
  // place(0, 0.7);

  ros::waitForShutdown();
  return 0;
}
