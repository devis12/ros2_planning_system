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


ResolveAmbiguities::Goal ActionResolveAmbiguities::buildGoal(const std::string& full_action_name, const std::string& explanation)
{
    std::cout << "building goal " << "\n" << std::flush;
    auto goal = ResolveAmbiguities::Goal();

    size_t start = full_action_name.find('(');
    size_t end = full_action_name.find(')', start);

    if (start != std::string::npos && end != std::string::npos && end > start) 
    {
        // Return the content inside the parentheses
        goal.action_name = full_action_name.substr(start + 1, end - start - 1);
    }
    else
        goal.action_name = full_action_name;
    
    // Split the string into words
    std::stringstream ss(goal.action_name);
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

    goal.explanation = explanation;

    std::cout << "goal built" << "\n" << std::flush;
    return goal;
}

void ActionResolveAmbiguities::send_goal(const ResolveAmbiguities::Goal& goal)
{
    std::cout << "waiting for action server" << "\n" << std::flush;
    resolve_ambiguities_client_->wait_for_action_server();
    std::cout << "action server ready" << "\n" << std::flush;

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
    RCLCPP_INFO(node_->get_logger(), "Sending goal: %s", goal.action_name.c_str());
    resolve_ambiguities_client_->async_send_goal(goal, options);
    std::cout << "Sent resolve ambiguities goal " << "\n" << std::flush;
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

    std::string issue_detected, explanation;
    getInput("issue_detected", issue_detected);
    getInput("explanation", explanation);

    bool no_issue_detected = issue_detected.find("ambiguity") == std::string::npos && issue_detected.find("unfeasibility") == std::string::npos;

    std::cout << "Running ActionResolveAmbiguities no_issue_detected " << no_issue_detected << "\n" << std::flush;

    auto goal = buildGoal(action, explanation);
    if(no_issue_detected && goal.ambiguous_arguments.size() == 0)
        {
            std::cout << "No ambiguous arguments" << "\n" << std::flush;
            return BT::NodeStatus::SUCCESS; // no ambiguous
        }
    else if(issue_detected.find("ambiguity") != std::string::npos)   
    {
      
    }     

    auto instances = problem_client_->getInstances();

    // TODO REMOVE THIS TEST
    std::cout << "Found an Ambiguous ARGUMENT! Actually doing something in ActionResolveAmbiguities for " << action << "\n" << std::flush;

    int ambiguous_resolved_counter = 0;
    for (auto &p_ins : instances)
    {
        for(const auto& instance_name : goal.ambiguous_arguments)
        {
            if(p_ins.name == instance_name && !p_ins.metainfo.empty())
            {
              ambiguous_resolved_counter++;
              break;
            }
        }   
    }



    if(no_issue_detected && ambiguous_resolved_counter == goal.ambiguous_arguments.size())
      return BT::NodeStatus::SUCCESS;

    // END TEST

  

  // TODO UNCOMMENT BELOW
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
      if(resolve_ambiguities_result_)
      {
        if(resolve_ambiguities_result_->success)
        {
          // here you have the disambiguated arguments
        //   resolve_ambiguities_result_->resolved_arguments;
          //TODO IMPLEMENT MISSING LOGIC HERE
            for(auto& p_ins : instances)
            {
                for (int idx = 0; idx < goal.ambiguous_arguments.size(); idx++)
                {
                    const auto &instance_name = goal.ambiguous_arguments[idx];
                    if (p_ins.name == instance_name && !resolve_ambiguities_result_->resolved_arguments[idx].empty())
                    {
                        p_ins.metainfo = resolve_ambiguities_result_->resolved_arguments[idx];
                        std::cout << "Supplying metainfo " << p_ins.metainfo << " to param " << p_ins.name << "\n" << std::flush;
                        problem_client_->updateInstance(p_ins);
                        break;
                    }
                }   
            }
        }
        else
          return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::RUNNING;
  }

  
}

}  // namespace plansys2
