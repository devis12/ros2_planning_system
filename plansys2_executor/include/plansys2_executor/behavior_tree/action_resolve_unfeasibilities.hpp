#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__ACTION_RESOLVE_UNFEASIBILITIES_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__ACTION_RESOLVE_UNFEASIBILITIES_NODE_HPP_

#include <map>
#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_problem_expert/Utils.hpp"

#include "plansys2_executor/behavior_tree/execute_action_node.hpp"

#include "plansys2_msgs/action/resolve_unfeasibilities.hpp"

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/client.hpp"

namespace plansys2
{

using ResolveUnfeasibilities = plansys2_msgs::action::ResolveUnfeasibilities;
using ResolveUnfeasibilitiesResult = plansys2_msgs::action::ResolveUnfeasibilities_Result;
using ResolveUnfeasibilitiesGoalHandle = rclcpp_action::ClientGoalHandle<ResolveUnfeasibilities>;

class ActionResolveUnfeasibilities : public BT::ActionNodeBase
{
public:
  ActionResolveUnfeasibilities(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt() {cancel_goal();}
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("action", "Action whose at end reqs must stop"),
        BT::InputPort<std::string>("issue_detected", "Issue detected in Raider check"),
        BT::InputPort<std::string>("explanation", "", "Explanation for the issue detected in Raider check"),
        BT::InputPort<int>("raider_loop_counter", "Number of times we have tried to resolve an issue for the same action")
      });
  }

private:
  static ResolveUnfeasibilities::Goal buildGoal(const std::string& full_action_name, const std::string& explanation);
  void send_goal(const ResolveUnfeasibilities::Goal& goal);

  void goal_response_callback(const ResolveUnfeasibilitiesGoalHandle::SharedPtr &goal_handle);

  void goal_result_callback(const ResolveUnfeasibilitiesGoalHandle::WrappedResult &result);

  void goal_feedback_callback(const ResolveUnfeasibilitiesGoalHandle::SharedPtr &goal_handle,
      const std::shared_ptr<const ResolveUnfeasibilities::Feedback> feedback);

  void cancel_goal();

  void reset_client_status();

  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;

  rclcpp_action::Client<ResolveUnfeasibilities>::SharedPtr resolve_unfeasibilities_client_;
  std::shared_ptr<ResolveUnfeasibilitiesResult> resolve_unfeasibilities_result_;
  ResolveUnfeasibilitiesGoalHandle::SharedPtr goal_handle_;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

  bool goal_sent_;
  bool response_received_;
  bool result_received_;
};

}  // namespace plansys2

#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__ACTION_REFINEMENT_NODE_HPP_
