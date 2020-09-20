#include "ros/ros.h"
#include "spawn_objects/spawn_objects.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/SpawnModel.h"

#include <sstream>

const float cube_size = 0.064;
const float cube_mass = 1.0;
const float offset_height = 0.05; // don't spawn *in* the table

// A lot of help from https://github.com/JenniferBuehler/gazebo-pkgs/blob/master/gazebo_test_tools/src/cube_spawner.cpp!

bool spawncube(float x, float y, float z, uint id, float red=-1, float green=-1, float blue=-1)
{
    //This does not check if there is already a model present with the same name
    geometry_msgs::Pose initial_pose;
    initial_pose.position.x = x;
    initial_pose.position.y = y;
    initial_pose.position.z = z;
    // initial_pose.orientation = ...;

    gazebo_msgs::SpawnModel model;
    model.request.model_name = "block_" + std::to_string(id);
    model.request.robot_namespace = "/blocks";
    model.request.initial_pose = initial_pose;
    model.request.reference_frame = "";

    float size[] = {cube_size, cube_size, cube_size}; //m robotiq stroke is 0.085
    float mass = cube_mass; //kg
    float inertia = mass*size[0]*size[0] / 0.6;
    std::string model_name = "my_cube";
    if(red == -1){
        red = (float) rand()/RAND_MAX;
    }
    if(green == -1){
        green = (float) rand()/RAND_MAX;
    }
    if(blue == -1){
        blue = (float) rand()/RAND_MAX;
    }

    std::stringstream _s;

    // http://gazebosim.org/tutorials?tut=build_model
    _s<<"<?xml version='1.0'?>\
    <sdf version='1.4'>\
    <model name='"<<model_name<<"'>\
        <pose>0 0 "<<(size[1]/2)<<" 0 0 0</pose>\
        <static>false</static>\
        <link name='link'>\
        <inertial>\
            <mass>"<<mass<<"</mass>\
            <inertia>\
            <ixx>"<<inertia<<"</ixx>\
            <ixy>0.0</ixy>\
            <ixz>0.0</ixz>\
            <iyy>"<<inertia<<"</iyy>\
            <iyz>0.0</iyz>\
            <izz>"<<inertia<<"</izz>\
            </inertia>\
        </inertial>\
        <collision name='collision'>\
            <geometry>\
            <box>\
                <size>"<<size[0]<<" "<<size[1]<<" "<<size[2]<<"</size>\
            </box>\
            </geometry>\
            <surface>\
                <contact>\
                    <ode>\
                        <kp>1000000</kp>\
                        <kd>1</kd>\
                    </ode>\
                </contact>\
            </surface>\
        </collision>\
        <visual name='visual'>\
            <geometry>\
                <box>\
                    <size>"<<size[0]<<" "<<size[1]<<" "<<size[2]<<"</size>\
                </box>\
            </geometry>\
            <material>\
                <ambient>"<<red<<" "<<green<<" "<<blue<<" 1</ambient>\
            </material>\
        </visual>\
        </link>\
    </model>\
    </sdf>";

    model.request.model_xml = _s.str();

    if (ros::service::call("/gazebo/spawn_sdf_model", model))
    {
        ROS_INFO("Success!");
    }
    else
    {
        ROS_ERROR("Failed to call service");
        return false;
    }

    return true;
}

bool callback(spawn_objects::spawn_objects::Request &request, spawn_objects::spawn_objects::Response &response)
{
    // Spawn a clump of objects centered at the middle of the object (table for now)
    // TODO: generalize this to any model in the scene, now it is hardcoded for the table
    uint8_t width_objs = request.width; // number of objects along the width (x)
    uint8_t length_objs = request.length; // number of objects along the length (y)

    float table_center[] = {0.2, -0.6}; // coords for the center of the table (x,y)
    float table_height = 1.0;
    float cube_separation = cube_size + 0.1; // defined from the middle of the cube

    float objectset_dim[] = {cube_separation * (width_objs-1), cube_separation * (length_objs-1)}; //distance of the sides of the object set
    float start_coord[] = {table_center[0]-objectset_dim[0]/2,table_center[1]-objectset_dim[1]/2}; //the coordinates of the first object

    int all_id = 0; // increment the ID. TODO, get the current max cube ID
    bool status = true; // make sure they ALL spawn successfully
    for (float x = start_coord[0]; x <= start_coord[0] + objectset_dim[0]; x=x+cube_separation)
    {
        for (float y = start_coord[1]; y <= start_coord[1] + objectset_dim[1]; y = y+cube_separation)
        {
            status = spawncube(x,y,table_height+offset_height,all_id) && status;
            all_id++;
        }
    }
    
    return status;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawn_objects_server");
    ros::NodeHandle n;

    // Start client for gazebo model spawner
    // ros::ServiceClient client = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");

    // Start our own server
    ros::ServiceServer service = n.advertiseService("spawn_objects_service", callback);
    ROS_INFO("spawn_objects server started.");
    ros::spin();
    return 0;
}