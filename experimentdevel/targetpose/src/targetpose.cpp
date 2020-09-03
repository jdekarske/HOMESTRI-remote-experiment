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
const float closed_position = 0.5;

void moveGripper(moveit::planning_interface::MoveGroupInterface &group, float pos)
{
    geometry_msgs::Pose target_pose1;
    target_pose1.position = pos;
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    group.move();
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
    group.setPoseTarget(target_pose1, "ee_link");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    group.move();

}

void pick(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& grab_group, float objx, float objy)
{
  move(arm_group, objx, objy, pick_height);
  moveGripper(grab_group, open_position);
  move(arm_group, objx, objy, pick_height-0.1);
  moveGripper(grab_group, closed_position);
  arm_group.attachObject("object");
  move(arm_group, objx, objy, pick_height);
}

void place(moveit::planning_interface::MoveGroupInterface& arm_group, moveit::planning_interface::MoveGroupInterface& grab_group, float objx, float objy)
{
  move(arm_group, objx, objy, pick_height);
  move(arm_group, objx, objy, pick_height-0.1);
  moveGripper(grab_group, closed_position);
  arm_group.detachObject("object");
  move(arm_group, objx, objy, pick_height);

}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(3);
    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.7;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.0;
    collision_objects[0].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.4;
    collision_objects[1].primitives[0].dimensions[1] = 0.2;
    collision_objects[1].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.7;
    collision_objects[1].primitive_poses[0].position.z = 0.0;
    collision_objects[1].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[2].header.frame_id = "world";
    collision_objects[2].id = "object";

    /* Define the primitive and its dimensions. */
    collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.7;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.3;
    collision_objects[2].primitive_poses[0].orientation.w = 1.0;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface manipulator_group("arm");
  moveit::planning_interface::MoveGroupInterface gripper_group("gripper");

  manipulator_group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  pick(manipulator_group, gripper_group, 0.7, 0);

  place(manipulator_group, gripper_group, 0, 0.7);

  ros::waitForShutdown();
  return 0;
}
