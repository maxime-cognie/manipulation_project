#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <vector>

#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/executors.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

static const rclcpp::Logger LOGGER =
  rclcpp::get_logger("pick_and_place_perception");

static const std::string PLANNING_GROUP_ARM = "ur_manipulator";
static const std::string PLANNING_GROUP_GRIPPER = "gripper";

class PickAndPlacePerception : public rclcpp::Node
{
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit PickAndPlacePerception(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("pick_and_place_perception", node_options),
    goal_done_(false)
  {
    this->client_ptr_ = rclcpp_action::create_client<Find>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "find_objects");     

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&PickAndPlacePerception::send_goal, this));
  }

  bool is_goal_done() const
  {
    return this->goal_done_;
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_) {
      RCLCPP_ERROR(LOGGER, "Action client not initialized");
    }

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(LOGGER, "Action server not available after waiting");
      return;
    }

    auto goal_msg = Find::Goal();
    goal_msg.plan_grasps = false;

    RCLCPP_INFO(LOGGER, "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Find>::SendGoalOptions();
                
    send_goal_options.goal_response_callback =
      std::bind(&PickAndPlacePerception::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&PickAndPlacePerception::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&PickAndPlacePerception::result_callback, this, _1);
      
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool goal_done_;
  float x_;
  float y_;

  void goal_response_callback(const GoalHandleFind::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(LOGGER, "Goal was rejected by server");
    } else {
      RCLCPP_INFO(LOGGER, "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleFind::SharedPtr,
    const std::shared_ptr<const Find::Feedback>)
  {
    RCLCPP_INFO(LOGGER, "Feedback received");
  }

  void result_callback(const GoalHandleFind::WrappedResult & result)
  {
    RCLCPP_INFO(LOGGER, "Result received");

    for(auto obj : result.result->objects){
      x_ = obj.object.primitive_poses[0].position.x + 0.012;
      y_ = obj.object.primitive_poses[0].position.y - 0.01;
      RCLCPP_INFO(LOGGER, "\nx: %f\ny: %f", x_, y_);
    }

    pick_and_place();

    goal_done_ = true;
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(LOGGER, "Success");
        return;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(LOGGER, "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(LOGGER, "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(LOGGER, "Unknown result code");
        return;
    }
  }

  void pick_and_place() {
      rclcpp::NodeOptions node_options;
      node_options.automatically_declare_parameters_from_overrides(true);
      auto move_group_node =
          rclcpp::Node::make_shared("pick_and_place_node", node_options);

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

      
      // Create collision object for the robot to avoid
      auto const collision_object = [frame_id =
                                      move_group_arm.getPlanningFrame()] {
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = frame_id;
        collision_object.id = "box1";
        shape_msgs::msg::SolidPrimitive primitive;

        // Define the size of the box in meters
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 2.0;
        primitive.dimensions[primitive.BOX_Y] = 2.0;
        primitive.dimensions[primitive.BOX_Z] = 0.001;

        // Define the pose of the box (relative to the frame_id)
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;  // We can leave out the x, y, and z components of the quaternion since they are initialized to 0
        box_pose.position.x = 0.0;
        box_pose.position.y = 0.1;
        box_pose.position.z = -0.1;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        return collision_object;
      }();

      // Add the collision object to the scene
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.applyCollisionObject(collision_object);

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

      geometry_msgs::msg::Pose target_pose;

      move_group_arm.setNamedTarget("initial_pose");
      success_arm = (move_group_arm.plan(my_plan_arm) ==
                         moveit::core::MoveItErrorCode::SUCCESS);

      move_group_gripper.execute(my_plan_arm);

      // Pregrasp
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Pregrasp Position");

      target_pose.position.x = x_,
      target_pose.position.y = y_;
      target_pose.position.z = 0.25;
      target_pose.orientation.x = -1.0;
      target_pose.orientation.y = 0.0;
      target_pose.orientation.z = 0.0;
      target_pose.orientation.w = 0.0;

      move_group_arm.setPoseTarget(target_pose);
      success_arm = (move_group_arm.plan(my_plan_arm) ==
                         moveit::core::MoveItErrorCode::SUCCESS);
      move_group_arm.execute(my_plan_arm);

      // Open Gripper
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Open Gripper!");

      move_group_gripper.setNamedTarget("open");

      success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                              moveit::core::MoveItErrorCode::SUCCESS);

      move_group_gripper.execute(my_plan_gripper);

      // Approach
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Approach to object!");
      
      std::vector<geometry_msgs::msg::Pose> approach_waypoints;

      for (int i = 0; i < 3; i++){
        target_pose.position.z -= 0.025;
        approach_waypoints.push_back(target_pose);
      }

      moveit_msgs::msg::RobotTrajectory trajectory_approach;
      const double jump_threshold = 0.0;
      const double eef_step = 0.01;

      double fraction = move_group_arm.computeCartesianPath(
          approach_waypoints, eef_step, jump_threshold, trajectory_approach);

      move_group_arm.execute(trajectory_approach);

      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // Close Gripper
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Close Gripper!");

      joint_group_positions_gripper[2] = 0.6; // Shoulder Pan
      move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
      success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                    moveit::core::MoveItErrorCode::SUCCESS);
      move_group_gripper.execute(my_plan_gripper);

      current_state_gripper = move_group_gripper.getCurrentState(10);
      current_state_gripper->copyJointGroupPositions(joint_model_group_gripper,
                                                 joint_group_positions_gripper);

      while (joint_group_positions_gripper[2] <= 0.64) {
        joint_group_positions_gripper[2] += 0.005; // Shoulder Pan
        move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
        success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                     moveit::core::MoveItErrorCode::SUCCESS);
        move_group_gripper.execute(my_plan_gripper);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      joint_group_positions_gripper[2] += 0.0015; // Shoulder Pan
      move_group_gripper.setJointValueTarget(joint_group_positions_gripper);
      success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                   moveit::core::MoveItErrorCode::SUCCESS);
      move_group_gripper.execute(my_plan_gripper);  

      // Retreat
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Retreat from object!");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      std::vector<geometry_msgs::msg::Pose> retreat_waypoints;

      for (int i = 0; i < 3; i++){
        target_pose.position.z += 0.02;
        retreat_waypoints.push_back(target_pose);
      }

      moveit_msgs::msg::RobotTrajectory trajectory_retreat;

      fraction = move_group_arm.computeCartesianPath(
          retreat_waypoints, eef_step, jump_threshold, trajectory_retreat);

      move_group_arm.execute(trajectory_retreat);
      
      // Place
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Rotating Arm");

      current_state_arm = move_group_arm.getCurrentState(10);
      current_state_arm->copyJointGroupPositions(joint_model_group_arm,
                                                 joint_group_positions_arm);

      joint_group_positions_arm[0] += 3.1415; // Shoulder Pan

      move_group_arm.setJointValueTarget(joint_group_positions_arm);

      success_arm = (move_group_arm.plan(my_plan_arm) ==
                     moveit::core::MoveItErrorCode::SUCCESS);

      move_group_arm.execute(my_plan_arm);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));

      // Open Gripper
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Release Object!");

      move_group_gripper.setNamedTarget("open");

      success_gripper = (move_group_gripper.plan(my_plan_gripper) ==
                         moveit::core::MoveItErrorCode::SUCCESS);

      move_group_gripper.execute(my_plan_gripper);

      // Go back to arm initial position
      RCLCPP_INFO(rclcpp::get_logger("pick_and_place_node"), "Move back to initial position!");
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      move_group_arm.setNamedTarget("initial_pose");

      success_arm = (move_group_arm.plan(my_plan_arm) ==
                         moveit::core::MoveItErrorCode::SUCCESS);

      move_group_gripper.execute(my_plan_arm);

  }
};  // class PickAndPlacePerception

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<PickAndPlacePerception>();
    
  while (!action_client->is_goal_done() && rclcpp::ok()) {
    rclcpp::spin_some(action_client);
  }

  rclcpp::shutdown();
  return 0;
}