// ROS
#include <ros/ros.h>
#include <ros/console.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

const float pick_height = 0.6;
const float open_position = 0.0;
const float closed_position = 0.25;

void moveGripper(moveit::planning_interface::MoveGroupInterface &group, double pos)
{
  group.setJointValueTarget("gripper_finger1_joint", pos);
  bool success = (group.move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
}

void move(moveit::planning_interface::MoveGroupInterface &group, float x_des, float y_des, float z_des)
{
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.x = 0.707;
  target_pose1.orientation.y = 0.0;
  target_pose1.orientation.z = -.707;
  target_pose1.orientation.w = 0.0;
  target_pose1.position.x = x_des;
  target_pose1.position.y = y_des;
  target_pose1.position.z = z_des;
  group.setPoseTarget(target_pose1, "flange");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  group.move();
}

void pick(moveit::planning_interface::MoveGroupInterface &arm_group, moveit::planning_interface::MoveGroupInterface &grab_group, float objx, float objy)
{
  move(arm_group, objx, objy, pick_height);
  moveGripper(grab_group, open_position);
  move(arm_group, objx, objy, pick_height - 0.1);
  moveGripper(grab_group, closed_position);
  move(arm_group, objx, objy, pick_height);
}

void place(moveit::planning_interface::MoveGroupInterface &arm_group, moveit::planning_interface::MoveGroupInterface &grab_group, float objx, float objy)
{
  move(arm_group, objx, objy, pick_height);
  move(arm_group, objx, objy, pick_height - 0.1);
  moveGripper(grab_group, open_position);
  move(arm_group, objx, objy, pick_height);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "UR5e_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // connect to moveit
  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::MoveGroupInterface manipulator_group("manipulator");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

  manipulator_group.setPlanningTime(45.0);

  // (tests)
  // pick(manipulator_group, gripper_group, 0.7, 0);
  // place(manipulator_group, gripper_group, 0, 0.7);

  ros::waitForShutdown();
  return 0;
}
