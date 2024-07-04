#include <inttypes.h>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "grasping_msgs/action/find_graspable_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

static const rclcpp::Logger LOGGER =
    rclcpp::get_logger("get_cube_pose");

class GetCubePose : public rclcpp::Node
{
public:
  using Find = grasping_msgs::action::FindGraspableObjects;
  using GoalHandleFind = rclcpp_action::ClientGoalHandle<Find>;

  explicit GetCubePose(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("get_cube_pose", node_options),
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
      std::bind(&GetCubePose::send_goal, this));
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
      std::bind(&GetCubePose::goal_response_callback, this, _1);

    send_goal_options.feedback_callback =
      std::bind(&GetCubePose::feedback_callback, this, _1, _2);

    send_goal_options.result_callback =
      std::bind(&GetCubePose::result_callback, this, _1);
      
    auto goal_handle_future = this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Find>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool goal_done_;

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
    const std::shared_ptr<const Find::Feedback> feedback)
  {
    RCLCPP_INFO(LOGGER, "Feedback received");
  }

  void result_callback(const GoalHandleFind::WrappedResult & result)
  {
    float x;
    float y;
    RCLCPP_INFO(LOGGER, "Result received");

    for(auto obj : result.result->objects){
      x = obj.object.primitive_poses[0].position.x;
      y = obj.object.primitive_poses[0].position.y;
      RCLCPP_INFO(LOGGER, "\nx: %f\ny: %f", x, y);
    }    
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
};  // class GetCubePose

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto action_client = std::make_shared<GetCubePose>();
    
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_client);

  while (!action_client->is_goal_done() && rclcpp::ok()) {
    executor.spin();
  }

  rclcpp::shutdown();
  return 0;
}