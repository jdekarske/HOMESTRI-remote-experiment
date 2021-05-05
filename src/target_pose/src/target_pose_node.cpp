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
  // manipulator_group->setPoseReferenceFrame("world");

  attach.request.model_name_1 = "robot";
  attach.request.link_name_1 = "gripper_finger1_finger_tip_link";
  attach.request.model_name_2 = "";
  attach.request.link_name_2 = "link";

  service = nh.advertiseService("pick_place", &target_pose_node::pickplaceCallback, this);
  ROS_INFO("pick_place server started.");
}

target_pose_node::~target_pose_node()
{
  delete manipulator_group;
  delete gripper_group;
}

void target_pose_node::initPosition()
{
  manipulator_group->setNamedTarget("vertical");
  manipulator_group->move();

  moveGripper(open_position);
  ROS_INFO("Arm initial postion");
}

bool target_pose_node::moveGripper(float pos)
{
  gripper_group->setJointValueTarget("gripper_finger1_joint", pos);
  bool success = (gripper_group->move() == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ros::Duration(0.7).sleep(); //wait for the object to attach
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

// this assumes cubes are coming out of a box, will generalize later
bool target_pose_node::pick(float objx, float objy, float objz)
{

  bool success = true && move(objx, objy, objz + approach_height, orientation_top[0], orientation_top[1], orientation_top[2], orientation_top[3]) &&
                 moveGripper(open_position) &&
                 move(objx, objy, objz + pick_height, orientation_top[0], orientation_top[1], orientation_top[2], orientation_top[3]) &&
                 moveGripper(closed_position);
  return success;
}

bool target_pose_node::pick(std::string object_name)
{
  if (attach.request.model_name_2 != "")
  {
    ROS_ERROR("Object %s already attached", attach.request.model_name_2.c_str());
    return false;
  }
  attach.request.model_name_2 = object_name;

  gazebo_msgs::GetModelState model;
  model.request.model_name = object_name;
  model.request.relative_entity_name = "robot";

  if (ros::service::call("/gazebo/get_model_state", model))
  {
    geometry_msgs::Point model_position = model.response.pose.position;
    return pick(model_position.x, model_position.y, model_position.z) &&
           ros::service::call("/link_attacher_node/attach", attach);
  }
  else
  {
    ROS_ERROR("Failed to get model state");
    return false;
  }
}

bool target_pose_node::place(float objx, float objy, float objz)
{
  bool success = true && move(objx, objy - approach_height, objz, orientation_front[0], orientation_front[1], orientation_front[2], orientation_front[3]) &&
                 move(objx, objy - pick_height, objz, orientation_front[0], orientation_front[1], orientation_front[2], orientation_front[3]) &&
                 moveGripper(open_position);
  return true;
}

bool target_pose_node::place(std::string object_name)
{
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

  if (place(coords["x"], coords["y"], coords["z"]) &&
      ros::service::call("/link_attacher_node/detach", attach))
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
  res.status = pick(req.pick_object) && place(req.place_object); // this is poetic
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
