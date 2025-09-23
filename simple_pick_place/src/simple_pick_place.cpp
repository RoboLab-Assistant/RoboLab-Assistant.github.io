#include <cstdio>
#include <memory>
#include <thread>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char ** argv)
{
  // most code is adapted from hello_moveit.cpp and my_moveit_program.cpp (move_program.cpp)

  // boilerplate up until creating logger
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Declare Node wuth same name as package name
  auto const node = std::make_shared<rclcpp::Node>(
    "simple_pick_place",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger for node - print messages in terminal
  auto const logger = rclcpp::get_logger("simple_pick_place");
  
   // We spin up a SingleThreadedExecutor so MoveItVisualTools interact with ROS
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  auto spinner = std::thread([&executor]() { executor.spin(); });

  // Create the MoveIt MoveGroup Interface
  // We need to specify our node and the Planning Group "ur_manipulator"
  // This planning group is the same Planning Group in RViz, under the MotionPlanning -> Planning Request

  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "ur_manipulator");

  // // Construct and initialize MoveItVisualTools
  // auto moveit_visual_tools = moveit_visual_tools::MoveItVisualTools{
  //     node, "base_link", rviz_visual_tools::RVIZ_MARKER_TOPIC,
  //     move_group_interface.getRobotModel()}; // passing base link of robot - to use base link's frame?
  // moveit_visual_tools.deleteAllMarkers();
  // moveit_visual_tools.loadRemoteControl();
  
  // // Create a closures for visualization
  // auto const draw_title = [&moveit_visual_tools](auto text) {
  //   auto const text_pose = [] {
  //     auto msg = Eigen::Isometry3d::Identity();
  //     msg.translation().z() = 1.0;
  //     return msg;
  //   }();
  //   moveit_visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE,
  //                                   rviz_visual_tools::XLARGE);
  // };

  // auto const prompt = [&moveit_visual_tools](auto text) {
  //   moveit_visual_tools.prompt(text);
  // };
  // auto const draw_trajectory_tool_path =
  //   [&moveit_visual_tools,
  //   jmg = move_group_interface.getRobotModel()->getJointModelGroup(
  //       "ur_manipulator")](auto const trajectory) {
  //     moveit_visual_tools.publishTrajectoryLine(trajectory, jmg);
  //   };
  
  //   // Set a target Pose
  // auto const target_pose = []{
  //   geometry_msgs::msg::Pose msg;
  //   msg.orientation.w = 1.0;
  //   msg.position.x = 0.28;
  //   msg.position.y = 0.4;
  //   msg.position.z = 0.5;
  //   return msg;
  // }();

    // Define the orientation --> how the coordinate system is rotated with respect to other fixed coordinate system and define the centre of the coordinate system --> how end effector is positioned in space if one point is fixed
  tf2::Quaternion tf2_quat;
  // Create a quaternion object by specifying roll, pitch, and yaw angles
  // These angles should be specified in radians
  //tf2_quat.setRPY(0, 0, -3.14/2);
  tf2_quat.setRPY(5*3.14/4, 0, 0);
  // Convert tf2::Quaternion to geometry msgs::msg::Quaternion --> so that the coordinate system can be communicated through the ROS system
  geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);

  double const pick_cuvette_position[3] = {-0.3245, 0.40875, -0.15}; // see noteboook for calculations
  
  // Set a goal pose, however, try to avoid self-collision of joints
  auto const target_pose = [msg_quat, pick_cuvette_position]{
    geometry_msgs::msg::Pose msg;
    // location of TCP will be 30cm away in the YZ plane, 135 degrees from pos Y direction
    msg.orientation = msg_quat;
    msg.position.x = pick_cuvette_position[0];
    msg.position.y = pick_cuvette_position[1]-(0.3/sqrt(2));
    msg.position.z = pick_cuvette_position[2]+(0.3/sqrt(2));
    return msg;
  }();
  // geometry_msgs::msg::Pose GoalPose;
  // GoalPose.orientation = msg_quat;
  // GoalPose.position.x = 0.3;
  // GoalPose.position.y = -0.3;
  // GoalPose.position.z = 0.4;

  move_group_interface.setPoseTarget(target_pose);

// int const NUM_BOXES = 7;

// double const box_dims[NUM_BOXES][3] = {
//   {1.5, 0.8, 0.92},
//   {0.12, 0.12, 0.27},
//   {0.08, 0.9, 1.2},
//   {0.045, 0.9, 0.85},
//   {0.13, 0.9, 0.85},
//   {1.5, 0.8, 0.15},
//   {0.18, 0.8, 0.14}
// };

// // original positions:
// double const box_positions[NUM_BOXES][3] = {
//   {0.2, 0, -0.3},
//   {1.07, -0.185, 0},
//   {0.05, 0, 0},
//   {1.34, 0, 0},
//   {0.9, 0, 0},
//   {0.2, 0, 0.62},
//   {0.75, 0, 0.48}
// };

int const NEW_NUM_BOXES = 9;//10;

double const new_box_dims[NEW_NUM_BOXES][3] = {
  {1.5, 0.75, 0.92},
  {0.12, 0.12, 0.27},
  {0.08, 0.9, 1.2},
  {0.045, 0.9, 0.85},
  {0.13, 0.9, 0.85},
  {1.5, 0.75, 0.15},
  {0.18, 0.8, 0.14},
  //{0.36, 0.09, 0.0625},//{0.36, 0.8, 0.0625}, // modified so that top panel of box containing DLS is open
  {0.36, 0.1, 0.18},
  {0.36, 0.15, 0.18}
};

double const new_box_positions[NEW_NUM_BOXES][3] = {
  {0.2, -0.05, -0.3},
  {1.07, -0.185, 0},
  {0.05, 0, 0},
  {1.34, 0, 0},
  {0.9, 0, 0},
  {0.2, -0.05, 0.62},
  {0.75, 0, 0.48},
  //{1.14, -0.39, 0.4},//{1.14,-0.025, 0.4}, // modified as according to above
  {1.14, -0.08, 0.08},
  {1.14, -0.35, 0.08}
};



// define displacements
// for (int i = 0; i < NUM_BOXES; i++) {

// }


// double const box_positions[NUM_BOXES][3] = {
//   {-0.65, 0.7, -0.5},
//   {0.22, 0.515, -0.2},
//   {-0.8, 0.7, -0.2},
//   {0.49, 0.7, -0.2},
//   {0.05, 0.7, -0.2},
//   {-0.65, 0.7, 0.42},
//   {-0.1, 0.7, 0.28}
// };

// Create collision object for the robot to avoid
auto const collision_object = [frame_id =
                                move_group_interface.getPlanningFrame()] 
                                (auto box_dim[3], auto box_pos[3], auto name){
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = name;
  shape_msgs::msg::SolidPrimitive primitive;

  // Define the size of the box in meters
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = box_dim[0];
  primitive.dimensions[primitive.BOX_Y] = box_dim[1];
  primitive.dimensions[primitive.BOX_Z] = box_dim[2];

  // Define the pose of the box (relative to the frame_id)
  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = box_pos[0];
  box_pose.position.y = box_pos[1];
  box_pose.position.z = box_pos[2];

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
};

double box_pos[3];
double box_dim[3];
std::string box_name;

moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

double const X_DISP = -1.0;
double const Y_DISP = 0.65;
double const Z_DISP = -0.3;

for (int i = 0; i < NEW_NUM_BOXES; i++) {
  box_dim[0] = new_box_dims[i][0];
  box_dim[1] = new_box_dims[i][1];
  box_dim[2] = new_box_dims[i][2];

  box_pos[0] = new_box_positions[i][0] + X_DISP;
  box_pos[1] = new_box_positions[i][1] + Y_DISP;
  box_pos[2] = new_box_positions[i][2] + Z_DISP;
  box_name = "box" + std::to_string(i+1);
  // Add the collision object to the scene
  planning_scene_interface.applyCollisionObject(collision_object(box_dim, box_pos, box_name));
}
rclcpp::sleep_for(std::chrono::seconds(1));


  // // Create collision object for the robot to avoid
  // auto const collision_object = [frame_id =
  //                                 move_group_interface.getPlanningFrame()] {
  //   moveit_msgs::msg::CollisionObject collision_object;
  //   collision_object.header.frame_id = frame_id;
  //   collision_object.id = "box1";
  //   shape_msgs::msg::SolidPrimitive primitive;

  //   // Define the size of the box in meters
  //   primitive.type = primitive.BOX;
  //   primitive.dimensions.resize(3);
  //   primitive.dimensions[primitive.BOX_X] = 0.5;
  //   primitive.dimensions[primitive.BOX_Y] = 0.1;
  //   primitive.dimensions[primitive.BOX_Z] = 0.5;

  //   // Define the pose of the box (relative to the frame_id)
  //   geometry_msgs::msg::Pose box_pose;
  //   box_pose.orientation.w = 1.0;
  //   box_pose.position.x = 0.2;
  //   box_pose.position.y = 0.3;
  //   box_pose.position.z = 0.25;

  //   collision_object.primitives.push_back(primitive);
  //   collision_object.primitive_poses.push_back(box_pose);
  //   collision_object.operation = collision_object.ADD;

  //   return collision_object;
  // }();

  // // Add the collision object to the scene
  // moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  // planning_scene_interface.applyCollisionObject(collision_object);

  // Create a plan to that target pose
  // prompt("Press 'Next' in the RvizVisualToolsGui window to plan");
  // draw_title("Planning");
  // moveit_visual_tools.trigger();
  auto const [success, plan] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    // draw_trajectory_tool_path(plan.trajectory_);
    // moveit_visual_tools.trigger();
    // prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    // draw_title("Executing");
    // moveit_visual_tools.trigger();
    move_group_interface.execute(plan);
  } else {
    // draw_title("Planning Failed!");
    // moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Planning failed!");
  }


  // part 2:
 // Define the orientation --> how the coordinate system is rotated with respect to other fixed coordinate system and define the centre of the coordinate system --> how end effector is positioned in space if one point is fixed
  // tf2::Quaternion tf2_quat;
  // Create a quaternion object by specifying roll, pitch, and yaw angles
  // These angles should be specified in radians
  //tf2_quat.setRPY(0, 0, -3.14/2);
  // tf2_quat.setRPY(0, 0, 0);
  // // Convert tf2::Quaternion to geometry msgs::msg::Quaternion --> so that the coordinate system can be communicated through the ROS system
  // msg_quat = tf2::toMsg(tf2_quat);


  // suggested edit:
  tf2::Quaternion tf2_quat2;
tf2_quat2.setRPY(0, 0, 0);
tf2_quat2.normalize();
geometry_msgs::msg::Quaternion msg_quat2 = tf2::toMsg(tf2_quat2);


  double const place_cuvette_position[3] = {new_box_positions[1][0]+X_DISP, new_box_positions[1][1]+Y_DISP,
     new_box_positions[1][2]+0.5*new_box_dims[1][2]+Z_DISP}; // see noteboook for calculations
  


  // Set a goal pose, however, try to avoid self-collision of joints
  //auto const target_pose = [msg_quat, pick_cuvette_position]{
  auto const target_pose2 = [msg_quat2, place_cuvette_position]{
    geometry_msgs::msg::Pose msg;
    // location of TCP will be 30cm away in the YZ plane, 135 degrees from pos Y direction
    msg.orientation = msg_quat2;
    msg.position.x = place_cuvette_position[0];
    msg.position.y = place_cuvette_position[1];
    msg.position.z = place_cuvette_position[2]+0.3;
    return msg;
  }();
  // geometry_msgs::msg::Pose GoalPose;
  // GoalPose.orientation = msg_quat;
  // GoalPose.position.x = 0.3;
  // GoalPose.position.y = -0.3;
  // GoalPose.position.z = 0.4;

  move_group_interface.clearPoseTargets();

  move_group_interface.setPoseTarget(target_pose2);

  //auto const [success, plan] = [&move_group_interface]{
  auto const [success2, plan2] = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  RCLCPP_INFO(logger, "Place pose: x=%.3f, y=%.3f, z=%.3f", 
    target_pose2.position.x,
    target_pose2.position.y,
    target_pose2.position.z);


  // Execute the plan
  if(success2) {
    // draw_trajectory_tool_path(plan.trajectory_);
    // moveit_visual_tools.trigger();
    // prompt("Press 'Next' in the RvizVisualToolsGui window to execute");
    // draw_title("Executing");
    // moveit_visual_tools.trigger();
    move_group_interface.execute(plan2);
  } else {
    // draw_title("Planning Failed!");
    // moveit_visual_tools.trigger();
    RCLCPP_ERROR(logger, "Second Planning failed!");
  }

  // Shutdown ROS
rclcpp::shutdown();
spinner.join();  // <- Join the thread to cleanly exit



  return 0;
}
