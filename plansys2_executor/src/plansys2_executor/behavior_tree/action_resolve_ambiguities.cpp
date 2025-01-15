#include <string>
#include <map>
#include <memory>

#include "plansys2_executor/behavior_tree/action_resolve_ambiguities.hpp"

using namespace std::placeholders;

namespace plansys2
{

ActionResolveAmbiguities::ActionResolveAmbiguities(
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

void ActionResolveAmbiguities::reset_client_status()
{
  goal_handle_.reset();

  goal_sent_ = false;
  response_received_ = false;
  result_received_ = false;
  resolve_ambiguities_client_.reset();

  resolve_ambiguities_result_.reset();

  resolve_ambiguities_client_ = 
            rclcpp_action::create_client<ResolveAmbiguities>(node_, "resolve_ambiguities");
}

void ActionResolveAmbiguities::send_goal(const std::string& full_action_name)
{
    resolve_ambiguities_client_->wait_for_action_server();

    
    auto goal = ResolveAmbiguities::Goal();
    goal.action_name = full_action_name;
    
    // Split the string into words
    std::stringstream ss(full_action_name);
    std::string word;
    
    std::vector<std::string> known_args;
    std::vector<std::string> ambiguous_args;

    // Skip the first word (action name), which is not an argument
    ss >> word; // Read and discard the action name
    
    // Process the remaining words (arguments)
    while (ss >> word) {
        // Check if the word contains two consecutive underscores
        if (word.find("__") != std::string::npos) {
            goal.ambiguous_arguments.push_back(word);
        } else {
            goal.known_arguments.push_back(word);
        }
    }

    auto options = rclcpp_action::Client<ResolveAmbiguities>::SendGoalOptions();
    options.goal_response_callback = std::bind(&ActionResolveAmbiguities::goal_response_callback, this, _1);
    options.result_callback = std::bind(&ActionResolveAmbiguities::goal_result_callback, this, _1);
    options.feedback_callback = std::bind(&ActionResolveAmbiguities::goal_feedback_callback, this, _1, _2);

    RCLCPP_INFO(node_->get_logger(), "Sent resolve ambiguities goal for %s, with ambiguous arguments %s", 
      goal.action_name.c_str(),
      std::accumulate(std::next(goal.ambiguous_arguments.begin()), goal.ambiguous_arguments.end(), goal.ambiguous_arguments[0], 
                                         [](const std::string& a, const std::string& b) {
                                             return a + "," + b;
                                         }).c_str());

    resolve_ambiguities_client_->async_send_goal(goal, options);

    goal_sent_ = true;
}

void ActionResolveAmbiguities::goal_response_callback(const ResolveAmbiguitiesGoalHandle::SharedPtr &goal_handle)
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

void ActionResolveAmbiguities::goal_result_callback(const ResolveAmbiguitiesGoalHandle::WrappedResult &result)
{
    auto status = result.code;        
    if (status == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "Resolve Ambiguities: Succeeded");
    }
    else if (status == rclcpp_action::ResultCode::ABORTED) {
        RCLCPP_ERROR(node_->get_logger(), "Resolve Ambiguities: Aborted");
    }
    else if (status == rclcpp_action::ResultCode::CANCELED) {
        RCLCPP_WARN(node_->get_logger(), "Resolve Ambiguities: Canceled");
    }

    RCLCPP_INFO(node_->get_logger(), "Ambiguities resolved: %d [%s]", result.result->success, 
      std::accumulate(std::next(result.result->resolved_arguments.begin()), result.result->resolved_arguments.end(), result.result->resolved_arguments[0], 
                                         [](const std::string& a, const std::string& b) {
                                             return a + "," + b;
                                         }).c_str());

    resolve_ambiguities_result_ = result.result;
    result_received_ = true;
}

void ActionResolveAmbiguities::goal_feedback_callback(const ResolveAmbiguitiesGoalHandle::SharedPtr &goal_handle,
    const std::shared_ptr<const ResolveAmbiguities::Feedback> feedback)
{
    (void)goal_handle;
    (void)feedback;
}

void ActionResolveAmbiguities::cancel_goal()
{
    if (this->goal_handle_) 
    {
        this->resolve_ambiguities_client_->async_cancel_goal(goal_handle_);
        goal_handle_.reset();
    }
    reset_client_status();
}

BT::NodeStatus
ActionResolveAmbiguities::tick()
{
  std::string action;
  getInput("action", action);

  std::cout << "Running ActionResolveAmbiguities for " << action << "\n" << std::flush;

  // if(!goal_sent_)
  // {
  //   send_goal(action);
  // }
  // else
  // {
  //   if(response_received_ && !goal_handle_)
  //   {
  //     // goal rejected or canceled
  //     return BT::NodeStatus::FAILURE;
  //   }
  //   else if(result_received_)
  //   {
  //     // post process result to affect action execution
  //     if(resolve_ambiguities_result_)
  //     {
  //       if(resolve_ambiguities_result_->success)
  //       {
  //         // here you have the disambiguated arguments
  //         resolve_ambiguities_result_->resolved_arguments;
  //       }
  //       else
  //         return BT::NodeStatus::FAILURE;
  //     }
  //   }
  // }

  return BT::NodeStatus::SUCCESS;
}

}  // namespace plansys2
