#include <target_pose/target_pose_node.h>

target_pose_node::target_pose_node()
{
  manipulator_group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  gripper_group = new moveit::planning_interface::MoveGroupInterface("gripper");
  manipulator_group->setPoseReferenceFrame("world"); // this needs investigating... this is the default
}

target_pose_node::target_pose_node(ros::NodeHandle &nodehandle) : nh(nodehandle)
{
  manipulator_group = new moveit::planning_interface::MoveGroupInterface("manipulator");
  gripper_group = new moveit::planning_interface::MoveGroupInterface("gripper");
  // manipulator_group->setGoalOrientationTolerance(0.1);
  // manipulator_group->setPoseReferenceFrame("world");

  attach.request.model_name_1 = "robot";
  attach.request.link_name_1 = "wrist_3_link";
  attach.request.model_name_2 = "";
  attach.request.link_name_2 = "link";

  pickplace_service = nh.advertiseService("pick_place", &target_pose_node::pickplaceCallback, this);
  reset_service = nh.advertiseService("pick_place/reset", &target_pose_node::resetCallback, this);
  ROS_INFO("pick_place server started.");
}

target_pose_node::~target_pose_node()
{
  delete manipulator_group;
  delete gripper_group;
}

bool target_pose_node::initPosition()
{
  if (attach.request.model_name_2 != "")
  {
    ros::service::call("/link_attacher_node/detach", attach);
    attach.request.model_name_2 = "";
  }
  
  manipulator_group->setStartStateToCurrentState();
  gripper_group->setStartStateToCurrentState();
  manipulator_group->setNamedTarget("home-box");
  manipulator_group->move();

  // logic: RRTConnect does it in like 0.05s so this is probably a good margin 
  // TODO this works pretty well but it seems convergence parameters are imminent (see PRMconfig.yaml)
  // TODO test PRMstar
  manipulator_group->setPlanningTime(0.5); //seconds
  ROS_INFO("planning time: %f", manipulator_group->getPlanningTime());

  moveGripper(open_position);
  ROS_INFO("Arm initial postion");
  return true; //TODO success everytime I guess?
}

bool target_pose_node::moveGripper(float pos)
{
  gripper_group->setJointValueTarget("gripper_finger1_joint", pos);
  bool success = (gripper_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;
}

bool target_pose_node::moveGripper(Gripper pos)
{
  if (pos == OPEN)
  {
    moveGripper(open_position);
  }
  else if (pos == CLOSE)
  {
    moveGripper(closed_position);
  }
}

bool target_pose_node::move(float x_des, float y_des, float z_des, float ow_des, float ox_des, float oy_des, float oz_des)
{
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = ow_des;
  target_pose1.orientation.x = ox_des;
  target_pose1.orientation.y = oy_des;
  target_pose1.orientation.z = oz_des;
  target_pose1.position.x = x_des;
  target_pose1.position.y = y_des;
  target_pose1.position.z = z_des;
  manipulator_group->setPoseTarget(target_pose1, "flange");

  bool success = (manipulator_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  return success;
}

bool target_pose_node::moveConstantOrientation(float x_des, float y_des, float z_des)
{
  auto current_pose = manipulator_group->getCurrentPose("flange");
  moveit_msgs::OrientationConstraint orientation_constraint;
  orientation_constraint.header.frame_id = manipulator_group->getPoseReferenceFrame();
  orientation_constraint.orientation = current_pose.pose.orientation;
  orientation_constraint.link_name = "flange";
  orientation_constraint.absolute_x_axis_tolerance = 1.0; // could be less
  orientation_constraint.absolute_y_axis_tolerance = 1.0;
  orientation_constraint.absolute_z_axis_tolerance = 1.0;
  orientation_constraint.weight = 1.0;
  moveit_msgs::Constraints constraint;
  constraint.orientation_constraints.emplace_back(orientation_constraint);

  manipulator_group->setPathConstraints(constraint);
  bool success = move(x_des, y_des, z_des,
          current_pose.pose.orientation.w,
          current_pose.pose.orientation.x,
          current_pose.pose.orientation.y,
          current_pose.pose.orientation.z);
  manipulator_group->clearPathConstraints();

  // fallback without constraints just in case
  if (!success) {
  success = move(x_des, y_des, z_des,
          current_pose.pose.orientation.w,
          current_pose.pose.orientation.x,
          current_pose.pose.orientation.y,
          current_pose.pose.orientation.z);
  }

  return success;
}

// this assumes cubes are coming out of a box, will generalize later
bool target_pose_node::pick(float objx, float objy, float objz)
{
  // do everything complain if any one thing doesn't work
  bool success = move(objx, objy, objz + approach_height, orientation_top[0], orientation_top[1], orientation_top[2], orientation_top[3]) &&
      moveGripper(open_position) &&
      moveConstantOrientation(objx, objy, objz + pick_height) &&
      moveGripper(closed_position) &&
      ros::service::call("/link_attacher_node/attach", attach) && //TODO abstract this
      moveConstantOrientation(objx, objy, objz + approach_height);
  return success;
}

bool target_pose_node::pick(std::string object_name)
{
    ROS_INFO("picking %s", object_name.c_str());
  if (attach.request.model_name_2 != "")
  {
    ROS_ERROR("Object %s already attached", attach.request.model_name_2.c_str());
    return false;
  }

  const char *model_prefix = "cube_";

  // get the object position (1,2,3,4 changed from object name)
  int position = object_name.back()  - '0';

  // get current cubes in sim
  std::vector<std::string> modelNames;
  gazebo_msgs::GetWorldProperties world;
  if (ros::service::call("/gazebo/get_world_properties", world))
  {
    std::vector<std::string> models = world.response.model_names;
    for (size_t i = 0; i < models.size(); i++)
    {
      if (models[i].find(model_prefix) != std::string::npos)
      {
        modelNames.push_back(models[i]);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to get world properties");
    return false;
  }

  std::string gazebo_object_name = modelNames[position-1]; // assume they are in order, index from 1

  gazebo_msgs::GetModelState model;
  model.request.model_name = gazebo_object_name;
  model.request.relative_entity_name = "robot";

  if (ros::service::call("/gazebo/get_model_state", model))
  {
    attach.request.model_name_2 = gazebo_object_name;
    geometry_msgs::Point model_position = model.response.pose.position;
    if (pick(model_position.x, model_position.y, model_position.z))
    {
      return true;
    }
    else
    {
      attach.request.model_name_2 = "";
      ROS_ERROR("Failed to pick");
      return false;
    }
  }
  else
  {
    ROS_ERROR("Failed to get model state");
    return false;
  }
}

bool target_pose_node::place(float objx, float objy, float objz)
{
    // do all the actions but complain if one of them doesnt work
  bool success = move(objx, objy - approach_height, objz, orientation_front[0], orientation_front[1], orientation_front[2], orientation_front[3]) &&
      moveConstantOrientation(objx, objy - pick_height, objz) &&
      moveGripper(open_position) &&
      ros::service::call("/link_attacher_node/detach", attach) && //TODO abstract this
      moveConstantOrientation(objx, objy - approach_height, objz);
  return success;
}

bool target_pose_node::place(std::string object_name)
{
  ROS_INFO("placing %s", object_name.c_str());
  if (attach.request.model_name_2 == "")
  {
    ROS_ERROR("Nothing to detach!");
    return false;
  }

  gazebo_msgs::GetModelState model;
  model.request.model_name = object_name;
  model.request.relative_entity_name = "robot";

  std::map<std::string, double> coords; // for coordinates
  std::string param_root = "/cube_positions/outputs/";
  std::string object_position = param_root + object_name;

  if (!ros::param::get(object_position, coords))
  {
    ROS_WARN("incorrect cube output: got %s", object_position.c_str());
  }

  if (place(coords["x"], coords["y"], coords["z"]))
  {
    attach.request.model_name_2 = "";
    return true;
  }
  else
  {
    return false;
  }
}

bool target_pose_node::pickplaceCallback(target_pose::pickplace::Request &req, target_pose::pickplace::Response &res)
{
  res.status = pick(req.pick_object);
  res.status = res.status && place(req.place_object);
  if (!res.status) {
    ROS_WARN("Something went wrong, resetting arm");
    initPosition();
  }
  return res.status;
}

bool target_pose_node::resetCallback(target_pose::reset::Request &req, target_pose::reset::Response &res)
{
  res.status = initPosition();
  return res.status;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "UR5e_pick_place");
  ros::NodeHandle nh;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  target_pose_node tp(nh);
  tp.initPosition();

  ros::waitForShutdown();
  return 0;
}
