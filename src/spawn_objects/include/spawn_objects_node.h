#include "ros/ros.h"
#include "spawn_objects/spawn_objects.h"
#include "geometry_msgs/Pose.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/GetWorldProperties.h"

#include <sstream>

#ifndef SPAWN_OBJECTS_NODEH
#define SPAWN_OBJECTS_NODEH

namespace ns_spawn_objects
{
    class spawn_objects_node
    {
    private:
        const float cube_size = 0.064;
        const float cube_mass = 0.25;
        const float offset_height = 0.05; // don't spawn *in* the table
        const char *model_prefix = "cube_";

        ros::NodeHandle nh;

        // A lot of help from https://github.com/JenniferBuehler/gazebo-pkgs/blob/master/gazebo_test_tools/src/cube_spawner.cpp!
        bool spawncube(float x, float y, float z, uint id, float red = -1, float green = -1, float blue = -1);

        // These come from gazebo in the from "cube_i", probably starting with i=0
        // The argument is a substring of the model name
        std::vector<std::string> findModelNames(const char *str);

    public:
        spawn_objects_node(ros::NodeHandle &nh);
        ros::ServiceServer service;
        bool callback(spawn_objects::spawn_objects::Request &request, spawn_objects::spawn_objects::Response &response);
    };

} // namespace ns_spawn_objects

#endif