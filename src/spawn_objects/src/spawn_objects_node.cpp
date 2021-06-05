#include "spawn_objects_node.h"

namespace ns_spawn_objects
{
    spawn_objects_node::spawn_objects_node(ros::NodeHandle &nodehandle):nh(nodehandle)
    {
        service = nh.advertiseService("spawn_objects_service", &ns_spawn_objects::spawn_objects_node::callback, this);
        ROS_INFO("spawn_objects server started.");
    }

    bool spawn_objects_node::spawncube(float x, float y, float z, uint id, float red, float green, float blue)
    {
        geometry_msgs::Pose initial_pose;
        initial_pose.position.x = x;
        initial_pose.position.y = y;
        initial_pose.position.z = z;
        // initial_pose.orientation = ...;

        gazebo_msgs::SpawnModel model;
        model.request.model_name = model_prefix + std::to_string(id); // TODO This does not check if there is already a model present with the same name
        model.request.robot_namespace = "/cubes";
        model.request.initial_pose = initial_pose;
        model.request.reference_frame = "";

        float size[] = {cube_size, cube_size, cube_size}; //m robotiq stroke is 0.085
        float mass = cube_mass;                           //kg
        float inertia = mass * size[0] * size[0] / 0.6;
        std::string model_name = "my_cube";
        if (red == -1)
        {
            red = (float)rand() / RAND_MAX;
        }
        if (green == -1)
        {
            green = (float)rand() / RAND_MAX;
        }
        if (blue == -1)
        {
            blue = (float)rand() / RAND_MAX;
        }

        std::stringstream _s;

        // http://gazebosim.org/tutorials?tut=build_model
        _s << "<?xml version='1.0'?>\
            <sdf version='1.4'>\
            <model name='" << model_name << "'>\
                <pose>0 0 " << (size[1] / 2) << " 0 0 0</pose>\
                <static>false</static>\
                <link name='link'>\
                <inertial>\
                    <mass>" << mass << "</mass>\
                    <inertia>\
                    <ixx>" << inertia << "</ixx>\
                    <ixy>0.0</ixy>\
                    <ixz>0.0</ixz>\
                    <iyy>" << inertia << "</iyy>\
                    <iyz>0.0</iyz>\
                    <izz>" << inertia << "</izz>\
                    </inertia>\
                </inertial>\
                <collision name='collision'>\
                    <geometry>\
                    <box>\
                        <size>" << size[0] << " " << size[1] << " " << size[2] << "</size>\
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
                            <size>" << size[0] << " " << size[1] << " " << size[2] << "</size>\
                        </box>\
                    </geometry>\
                    <material>\
                        <ambient>" << red << " " << green << " " << blue << " 1</ambient>\
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

    // These come from gazebo in the from "cube_i", probably starting with i=0
    // The argument is a substring of the model name
    std::vector<std::string> spawn_objects_node::findModelNames(const char *str)
    {
        std::vector<std::string> modelNames;
        gazebo_msgs::GetWorldProperties world;
        if (ros::service::call("/gazebo/get_world_properties", world))
        {
            std::vector<std::string> models = world.response.model_names;
            for (size_t i = 0; i < models.size(); i++)
            {
                if (models[i].find(str) != std::string::npos)
                {
                    modelNames.push_back(models[i]);
                }
            }
        }
        else
        {
            ROS_ERROR("Failed to get world properties");
        }
        return modelNames;
    }

    bool spawn_objects_node::callback(spawn_objects::spawn_objects::Request &request, spawn_objects::spawn_objects::Response &response)
    {
        std::vector<std::string> cubes = findModelNames(model_prefix);
        int all_id = 0;

        bool status = true; // make sure ALL cubes spawn successfully

        for (size_t i = 0; i < cubes.size(); i++)
        {
            if (request.overwrite)
            {
                //delete cubes
                gazebo_msgs::DeleteModel dmodel;
                dmodel.request.model_name = cubes[i];
                if (ros::service::call("/gazebo/delete_model", dmodel))
                {
                    ROS_INFO("Deleted model: %s", cubes[i].c_str());
                }
                else
                {
                    ROS_ERROR("Failed to delete model: %s", cubes[i].c_str());
                }
            }
            else
            {
                // find the max id to iterate from
                all_id = std::max(all_id, (int)(cubes[i].back() - '0')) + 1;
            }
        }

        // option A will load cube positions from a param file in the provided format
        if (!request.param_name.empty())
        {
            // retrieve the positions
            std::string param_root = request.param_name;
            std::map<std::string, double> input;    // for coordinates
            std::string stupidinputname = "/cube_"; // I'm frustrated with the lack of templating in getparam api
            int numcubes;

            if (nh.getParam(param_root + "/num", numcubes)) //param_name likely /cube_positions/inputs
            {
                // if we say position = -1, then spawn all the cubes
                if (request.position == 0)
                {
                    // spawn cubes in the positions
                    for (size_t i = 1; i <= numcubes; i++) //index from one
                    {
                        nh.getParam(param_root + stupidinputname + std::to_string(i), input);
                        status = spawncube(input["x"], input["y"], input["z"], i) && status;
                    }
                }
                else
                {
                    // spawn a single cube in a position
                    nh.getParam(param_root + stupidinputname + std::to_string(request.position), input);
                    // if any of the colors are negative, make it a random color
                    if (request.color[0] < 0 || request.color[1] < 0 || request.color[2] < 0)
                    {
                        status = spawncube(input["x"], input["y"], input["z"], request.position) && status;
                    }
                    else
                    {
                                                status = spawncube(input["x"], input["y"], input["z"], request.position, request.color[0], request.color[1], request.color[2]) && status;
                    }
                }
            }
            else
            {
                ROS_WARN("spawn position param not found.");
                status = false;
            }
        }
        else // Option B will generate the number of cubes that you want in a grid
        {
            // Spawn a clump of objects centered at the middle of the object (table for now)
            uint8_t width_objs = request.width;   // number of objects along the width (x)
            uint8_t length_objs = request.length; // number of objects along the length (y)

            float table_center[] = {0.2, -0.4}; // coords for the center of the table (x,y)
            float table_height = 1.0;
            float cube_separation = cube_size + 0.07; // defined from the middle of the cube

            float objectset_dim[] = {cube_separation * (width_objs - 1), cube_separation * (length_objs - 1)};      //distance of the sides of the object set
            float start_coord[] = {table_center[0] - objectset_dim[0] / 2, table_center[1] - objectset_dim[1] / 2}; //the coordinates of the first object

            for (float x = start_coord[0]; x <= start_coord[0] + objectset_dim[0]; x = x + cube_separation)
            {
                for (float y = start_coord[1]; y <= start_coord[1] + objectset_dim[1]; y = y + cube_separation)
                {
                    status = spawncube(x, y, table_height + offset_height, all_id) && status;
                    all_id++;
                }
            }
        }

        response.status = status;
        return status;
    }

} // namespace ns_spawn_objects

int main(int argc, char **argv)
{
    ros::init(argc, argv, "spawn_objects_server");

    ros::NodeHandle nh;
    ns_spawn_objects::spawn_objects_node son(nh);

    ros::spin();
    return 0;
}