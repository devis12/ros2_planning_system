#include <string>
#include <map>
#include <memory>
 
#include "plansys2_executor/behavior_tree/prompting_raider.hpp"
 
using namespace std::placeholders;
 
namespace plansys2
{
 
PromptingRaider::PromptingRaider(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: ActionNodeBase(xml_tag_name, conf)
{
  action_map_ =
    config().blackboard->get<std::shared_ptr<std::map<std::string, ActionExecutionInfo>>>(
    "action_map");
 
  problem_client_ =
    config().blackboard->get<std::shared_ptr<plansys2::ProblemExpertClient>>(
    "problem_client");
  
 
  node_ = config().blackboard->get<rclcpp_lifecycle::LifecycleNode::SharedPtr>("node");
  
  reset_client_status();
}
 
void PromptingRaider::reset_client_status()
{
  goal_handle_.reset();
 
  goal_sent_ = false;
  response_received_ = false;
  result_received_ = false;
  raider_client_.reset();
 
  raider_result_.reset();
 
  raider_client_ =
            rclcpp_action::create_client<Raider>(node_, "raider_issue_detect");
}
 
 
Raider::Goal PromptingRaider::buildGoal(const std::string& full_action_name)
{
    auto goal = Raider::Goal();
 
    // CALL RAIDER
    return goal;
}
 
void PromptingRaider::send_goal(const Raider::Goal& goal)
{
    raider_client_->wait_for_action_server();
 
    auto options = rclcpp_action::Client<Raider>::SendGoalOptions();
    options.goal_response_callback = std::bind(&PromptingRaider::goal_response_callback, this, _1);
    options.result_callback = std::bind(&PromptingRaider::goal_result_callback, this, _1);
    options.feedback_callback = std::bind(&PromptingRaider::goal_feedback_callback, this, _1, _2);
 
    RCLCPP_INFO(node_->get_logger(), "Sent raider goal for %s",
      goal.action_name.c_str());
 
    raider_client_->async_send_goal(goal, options);
 
    goal_sent_ = true;
}
 
void PromptingRaider::goal_response_callback(const RaiderGoalHandle::SharedPtr &goal_handle)
{
    if (!goal_handle) {
        RCLCPP_INFO(node_->get_logger(), "Goal got rejected");
    }
    else {
        this->goal_handle_ = goal_handle;
        RCLCPP_INFO(node_->get_logger(), "Goal got accepted");
    }
    response_received_ = true;
}
 
void PromptingRaider::goal_result_callback(const RaiderGoalHandle::WrappedResult &result)
{
    auto status = result.code;        
    if (status == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "Raider: Succeeded");
    }
    else if (status == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(node_->get_logger(), "Raider: Aborted");
    }
    else if (status == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "Raider: Canceled");
    }
 
    RCLCPP_INFO(node_->get_logger(), "Raider issue detected: %d [%s]", result.result->success,
      result.result->issue_detected);
 
    raider_result_ = result.result;
    result_received_ = true;
}
 
void PromptingRaider::goal_feedback_callback(const RaiderGoalHandle::SharedPtr &goal_handle,
    const std::shared_ptr<const Raider::Feedback> feedback)
{
    (void)goal_handle;
    (void)feedback;
}
 
void PromptingRaider::cancel_goal()
{
    if (this->goal_handle_)
    {
        this->raider_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    reset_client_status();
}
 
BT::NodeStatus
PromptingRaider::tick()
{
    std::string action;
    getInput("action", action);
    std::cout << "Running Raider for " << action << "\n" << std::flush;
    return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2