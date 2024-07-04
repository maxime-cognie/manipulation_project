#include "geometry_msgs/msg/detail/pose__struct.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <vector>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("pick_and_place_real");

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto move_group_node =
      rclcpp::Node::make_shared("pick_and_place_real_node", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(move_group_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
  static const std::string PLANNING_GROUP_GRIPPER = "gripper";

  moveit::planning_interface::MoveGroupInterface move_group_arm(
      move_group_node, PLANNING_GROUP_ARM);
  moveit::planning_interface::MoveGroupInterface move_group_gripper(
      move_group_node, PLANNING_GROUP_GRIPPER);

  const moveit::core::JointModelGroup *joint_model_group_arm =
      move_group_arm.getCurrentState()->getJointModelGroup(PLANNING_GROUP_ARM);
  const moveit::core::JointModelGroup *joint_model_group_gripper =
      move_group_gripper.getCurrentState()->getJointModelGroup(
          PLANNING_GROUP_GRIPPER);

  // Get Current State
  moveit::core::RobotStatePtr current_state_arm =
      move_group_arm.getCurrentState(10);
  moveit::core::RobotStatePtr current_state_gripper =
      move_group_gripper.getCurrentState(10);

  std::vector<double> joint_group_positions_arm;
  std::vector<double> joint_group_positions_gripper;

  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);
  current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);
              
  move_group_arm.setStartStateToCurrentState();
  move_group_gripper.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan my_plan_arm;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan_gripper;

  bool success_arm;
  bool success_gripper;

  move_group_arm.setNamedTarget("initial_position");
  success_arm = (move_group_arm.plan(my_plan_arm) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_arm);

  // Pregrasp
  RCLCPP_INFO(LOGGER, "Pregrasp Position");

  move_group_arm.setNamedTarget("pre_grasp_position"); 

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
  move_group_arm.execute(my_plan_arm);

  // Open Gripper
  RCLCPP_INFO(LOGGER, "Open Gripper!");

  move_group_gripper.setNamedTarget("open");
  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                          moveit::core::MoveItErrorCode::SUCCESS);
  move_group_gripper.execute(my_plan_gripper);

  // Approach
  RCLCPP_INFO(LOGGER, "Approach to object!");

  geometry_msgs::msg::Pose target_pose;
  std::vector<geometry_msgs::msg::Pose> approach_waypoints;
  target_pose.position.x = 0.343;
  target_pose.position.y = 0.132;
  target_pose.position.z = 0.25;
  target_pose.orientation.x = -1.0;
  target_pose.orientation.y = 0.0;
  target_pose.orientation.z = 0.0;
  target_pose.orientation.w = 0.0;
  approach_waypoints.push_back(target_pose);

  target_pose.position.z -= 0.025;
  approach_waypoints.push_back(target_pose);

  target_pose.position.z -= 0.025;
  approach_waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory_approach;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;

  double fraction = move_group_arm.computeCartesianPath(
      approach_waypoints, eef_step, jump_threshold, trajectory_approach);

  move_group_arm.execute(trajectory_approach);

  // Close Gripper
  RCLCPP_INFO(LOGGER, "Close Gripper!");

  move_group_gripper.setNamedTarget("close");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Retreat
  RCLCPP_INFO(LOGGER, "Retreat from object!");
  std::vector<geometry_msgs::msg::Pose> retreat_waypoints;

  for (int i = 0; i < 4; i++){
    target_pose.position.z += 0.025;
    retreat_waypoints.push_back(target_pose);
  }

  moveit_msgs::msg::RobotTrajectory trajectory_reatreat;

  fraction = move_group_arm.computeCartesianPath(
      retreat_waypoints, eef_step, jump_threshold, trajectory_reatreat);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  move_group_arm.execute(trajectory_reatreat);

  // Place
  RCLCPP_INFO(LOGGER, "Rotating Arm");

  current_state_arm = move_group_arm.getCurrentState(10);
  current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                             joint_group_positions_arm);

  joint_group_positions_arm[0] += 3.0; // Shoulder Pan

  move_group_arm.setJointValueTarget(joint_group_positions_arm);

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                 moveit::core::MoveItErrorCode::SUCCESS);

  move_group_arm.execute(my_plan_arm);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//   // Place
//   RCLCPP_INFO(LOGGER, "Rotating Arm");

//   current_state_arm = move_group_arm.getCurrentState(10);
//   current_state_arm->copyJointGroupPositions(joint_model_group_arm,
//                                              joint_group_positions_arm);

//   joint_group_positions_arm[0] -= 3.1415; // Shoulder Pan

//   move_group_arm.setJointValueTarget(joint_group_positions_arm);

//   success_arm = (move_group_arm.plan(my_plan_arm) ==
//                  moveit::core::MoveItErrorCode::SUCCESS);

//   move_group_arm.execute(my_plan_arm);
//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//   RCLCPP_INFO(LOGGER, "Releasing the block");
//   std::vector<geometry_msgs::msg::Pose> release_waypoints;

//   for (int i = 0; i < 4; i++){
//     target_pose.position.z -= 0.025;
//     release_waypoints.push_back(target_pose);
//   }

//   moveit_msgs::msg::RobotTrajectory trajectory_release;

//   fraction = move_group_arm.computeCartesianPath(
//       release_waypoints, eef_step, jump_threshold, trajectory_release);

//   std::this_thread::sleep_for(std::chrono::milliseconds(1000));

//   move_group_arm.execute(trajectory_release);

  // Open Gripper
  RCLCPP_INFO(LOGGER, "Release Object!");

  move_group_gripper.setNamedTarget("open");

  success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_gripper);

  // Go back to arm initial position
  RCLCPP_INFO(LOGGER, "Move back to initial position!");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  move_group_arm.setNamedTarget("initial_position");

  success_arm = (move_group_arm.plan(my_plan_arm) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

  move_group_gripper.execute(my_plan_arm);

  RCLCPP_INFO(LOGGER, "Pick and place mission done!");

  rclcpp::shutdown();
  return 0;
}
