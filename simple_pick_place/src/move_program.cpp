// Motion planning in C++ and MoveIt2

// necessary memory include
#include <memory>
// rclcpp - ROS Client Library for C++ package
#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Quaternion.h>


int main(int argc, char* argv[]) {
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Declare Node
    auto const node = std::make_shared<rclcpp::Node>
    (
    "move_program",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    
    // Create a ROS logger - print messages in terminal
    auto const logger = rclcpp::get_logger("move_program");


    // Create the MoveIt MoveGroup Interface
    // We need to specify our node and the Planning Group "ur_manipulator"
    // This planning group is the same Planning Group in RViz, under the MotionPlanning -> Planning Request
    moveit::planning_interface::MoveGroupInterface move_group_interface(node, "ur_manipulator");
    
    // Define the goal pose
    
    // Define the orientation --> how the coordinate system is rotated with respect to other fixed coordinate system and define the centre of the coordinate system --> how end effector is positioned in space if one point is fixed
    tf2::Quaternion tf2_quat;
    // Create a quaternion object by specifying roll, pitch, and yaw angles
    // These angles should be specified in radians
    tf2_quat.setRPY(0, 0, -3.14/2);
    // Convert tf2::Quaternion to geometry msgs::msg::Quaternion --> so that the coordinate system can be communicated through the ROS system
    geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);
    
    // Set a goal pose, however, try to avoid self-collision of joints
    geometry_msgs::msg::Pose GoalPose;
    GoalPose.orientation = msg_quat;
    GoalPose.position.x = 0.3;
    GoalPose.position.y = -0.3;
    GoalPose.position.z = 0.4;
    
    // After defining the goal pose, we need to tell the MoveGroupInterface to accept and set that pose
    move_group_interface.setPoseTarget(GoalPose);
    
    // Create a plan that will move the robot to the goal pose
    moveit::planning_interface::MoveGroupInterface::Plan plan1;
    auto const outcome = static_cast<bool>(move_group_interface.plan(plan1));
    
    // Execute the plan
    if(outcome)
    {
        move_group_interface.execute(plan1);
    }
    else
    {
        RCLCPP_ERROR(logger, "We were not able to plan and execute!");
    }
    
    // Shutdown ROS
    rclcpp::shutdown();
    return 0;
}
