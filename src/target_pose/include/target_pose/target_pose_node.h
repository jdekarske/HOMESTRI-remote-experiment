#pragma once

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
#include <target_pose/pickplace.h>
#include <gazebo_msgs/GetModelState.h>

// Attach
#include <gazebo_ros_link_attacher/Attach.h>

class target_pose_node
{
private:
    const float pick_height = 0.15;
    const float place_height = 0.23;
    const float approach_height = 0.3;
    const float open_position = 0.0;
    const float closed_position = 0.20;
    const float orientation_top[4] = {0.0, 0.707, 0, -0.707}; // facing negative Z
    const float orientation_front[4] = {0.0, 0.707, 0.707, 0.0};  // facing positive y

    gazebo_ros_link_attacher::Attach attach; //currently attached block

    ros::NodeHandle nh;
    ros::ServiceServer service;

    moveit::planning_interface::MoveGroupInterface *manipulator_group;
    moveit::planning_interface::MoveGroupInterface *gripper_group;

public:
    target_pose_node();
    target_pose_node(ros::NodeHandle &nodehandle);
    ~target_pose_node();

    // move to the assigned position which is above the table
    void initPosition();

    enum Gripper
    {
        OPEN,
        CLOSE
    };

    bool moveGripper(float pos);

    // use target_pose_node::Gripper::OPEN or target_pose_node::Gripper::CLOSE
    bool moveGripper(Gripper pos);

    bool move(float x_des, float y_des, float z_des, float ow_des, float ox_des, float oy_des, float oz_des);

    bool pick(float objx, float objy, float objz);

    bool pick(std::string object_name);

    bool place(float objx, float objy, float objz);

    bool place(std::string object_name);

    bool pickplaceCallback(target_pose::pickplace::Request &req, target_pose::pickplace::Response &res);
};
