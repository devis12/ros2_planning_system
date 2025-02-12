#include <string>
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/action_resolve_unfeasibilities.hpp"

using namespace std::placeholders;

namespace plansys2
{

ActionResolveUnfeasibilities::ActionResolveUnfeasibilities(
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

void ActionResolveUnfeasibilities::reset_client_status()
{
  goal_handle_.reset();

  goal_sent_ = false;
  response_received_ = false;
  result_received_ = false;
  resolve_unfeasibilities_client_.reset();

  resolve_unfeasibilities_result_.reset();

  resolve_unfeasibilities_client_ = 
            rclcpp_action::create_client<ResolveUnfeasibilities>(node_, "resolve_unfeasibilities");
}


ResolveUnfeasibilities::Goal ActionResolveUnfeasibilities::buildGoal(const std::string& full_action_name, const std::string& explanation)
{
    // std::cout << "building goal " << "\n" << std::flush;
    auto goal = ResolveUnfeasibilities::Goal();

    size_t start = full_action_name.find('(');
    size_t end = full_action_name.find(')', start);

    if (start != std::string::npos && end != std::string::npos && end > start) 
    {
        // Return the content inside the parentheses
        goal.action_name = full_action_name.substr(start + 1, end - start - 1);
    }
    else
        goal.action_name = full_action_name;

    goal.explanation = explanation;

    // std::cout << "goal built" << "\n" << std::flush;
    return goal;
}

void ActionResolveUnfeasibilities::send_goal(const ResolveUnfeasibilities::Goal& goal)
{
    std::cout << "waiting for action server" << "\n" << std::flush;
    resolve_unfeasibilities_client_->wait_for_action_server();
    std::cout << "action server ready" << "\n" << std::flush;

    auto options = rclcpp_action::Client<ResolveUnfeasibilities>::SendGoalOptions();
    options.goal_response_callback = std::bind(&ActionResolveUnfeasibilities::goal_response_callback, this, _1);
    options.result_callback = std::bind(&ActionResolveUnfeasibilities::goal_result_callback, this, _1);
    options.feedback_callback = std::bind(&ActionResolveUnfeasibilities::goal_feedback_callback, this, _1, _2);

    RCLCPP_INFO(node_->get_logger(), "Sending goal: %s", goal.action_name.c_str());
    resolve_unfeasibilities_client_->async_send_goal(goal, options);
    std::cout << "Sent resolve unfeasibilities goal " << "\n" << std::flush;
    goal_sent_ = true;
}

void ActionResolveUnfeasibilities::goal_response_callback(const ResolveUnfeasibilitiesGoalHandle::SharedPtr &goal_handle)
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

void ActionResolveUnfeasibilities::goal_result_callback(const ResolveUnfeasibilitiesGoalHandle::WrappedResult &result)
{
    auto status = result.code;        
    if (status == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "Resolve Unfeasibilities: Succeeded");
    }
    else if (status == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(node_->get_logger(), "Resolve Unfeasibilities: Aborted");
    }
    else if (status == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "Resolve Unfeasibilities: Canceled");
    }

    resolve_unfeasibilities_result_ = result.result;
    result_received_ = true;
}

void ActionResolveUnfeasibilities::goal_feedback_callback(const ResolveUnfeasibilitiesGoalHandle::SharedPtr &goal_handle,
    const std::shared_ptr<const ResolveUnfeasibilities::Feedback> feedback)
{
    (void)goal_handle;
    (void)feedback;
}

void ActionResolveUnfeasibilities::cancel_goal()
{
    if (this->goal_handle_) 
    {
        this->resolve_unfeasibilities_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    reset_client_status();
}

BT::NodeStatus
ActionResolveUnfeasibilities::tick()
{
  std::string action;
  getInput("action", action);
  // std::cout << "Running ActionResolveUnfeasibilities for " << action << "\n" << std::flush;

  std::string issue_detected, explanation;
  getInput("issue_detected", issue_detected);
  getInput("explanation", explanation);
  
  int raider_loop_counter;
  getInput("raider_loop_counter", raider_loop_counter);

  bool no_issue_detected = issue_detected.find("unfeasibility") == std::string::npos;

  // std::cout << "Running ActionResolveUnfeasibilities no_issue_detected " << no_issue_detected << "\n" << std::flush;
  // std::cout << "THE ISSUE DETECTED ISSSSSSSS: " << issue_detected << "\n" << std::flush;
  auto goal = buildGoal(action, explanation);
  if(no_issue_detected)
    {
      std::cout << "No unfeasibility" << "\n" << std::flush;
      return BT::NodeStatus::SUCCESS; // no unfeasibility
    }  

  if(!goal_sent_)
  {
    send_goal(goal);
    return BT::NodeStatus::RUNNING;
  }
  else
  {
    if(response_received_ && !goal_handle_)
    {
      // goal rejected or canceled
      return BT::NodeStatus::FAILURE;
    }
    else if(result_received_)
    {
      // post process result to affect action execution
      if(resolve_unfeasibilities_result_)
      {
        if(resolve_unfeasibilities_result_->success)
        {
          std::cout << "Unfeasibility resolved successfully" << "\n" << std::flush;
          reset_client_status();
          return BT::NodeStatus::SUCCESS;
        }
        else
          return BT::NodeStatus::FAILURE;
      }
    }
    return BT::NodeStatus::RUNNING;
  }

  
}

}  // namespace plansys2
