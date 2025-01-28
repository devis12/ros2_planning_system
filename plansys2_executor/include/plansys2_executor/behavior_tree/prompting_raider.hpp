#ifndef PLANSYS2_EXECUTOR__BEHAVIOR_TREE__RAIDER_NODE_HPP_
#define PLANSYS2_EXECUTOR__BEHAVIOR_TREE__RAIDER_NODE_HPP_
 
#include <map>
#include <string>
#include <memory>
 
#include "behaviortree_cpp_v3/action_node.h"
 
#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_executor/ActionExecutor.hpp"
#include "plansys2_problem_expert/Utils.hpp"
 
#include "plansys2_executor/behavior_tree/execute_action_node.hpp"
 
#include "plansys2_msgs/action/raider.hpp"
 
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_action/client.hpp"
 
namespace plansys2
{
 
using Raider = plansys2_msgs::action::Raider;
using RaiderResult = plansys2_msgs::action::Raider_Result;
using RaiderGoalHandle = rclcpp_action::ClientGoalHandle<Raider>;
 
class PromptingRaider : public BT::ActionNodeBase
{
public:
  PromptingRaider(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);
 
  void halt() {cancel_goal();}
  BT::NodeStatus tick() override;
 
  static BT::PortsList providedPorts()
  {
    return BT::PortsList(
      {
        BT::InputPort<std::string>("action", "Action whose at end reqs must stop"),
        BT::OutputPort<std::string>("issue_detected", "Issue detected in Raider check"),
        BT::OutputPort<std::string>("explanation", "Explanation for the issue detected in Raider check"),
      });
  }
 
private:
  static Raider::Goal buildGoal(const std::string& full_action_name);
  void send_goal(const Raider::Goal& goal);
 
  void goal_response_callback(const RaiderGoalHandle::SharedPtr &goal_handle);
 
  void goal_result_callback(const RaiderGoalHandle::WrappedResult &result);
 
  void goal_feedback_callback(const RaiderGoalHandle::SharedPtr &goal_handle,
      const std::shared_ptr<const Raider::Feedback> feedback);
 
  void cancel_goal();
 
  void reset_client_status();
 
  std::shared_ptr<std::map<std::string, ActionExecutionInfo>> action_map_;
  std::shared_ptr<plansys2::ProblemExpertClient> problem_client_;
 
  rclcpp_action::Client<Raider>::SharedPtr raider_client_;
  std::shared_ptr<RaiderResult> raider_result_;
  RaiderGoalHandle::SharedPtr goal_handle_;
 
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
 
  bool goal_sent_;
  bool response_received_;
  bool result_received_;
};
 
}  // namespace plansys2
 
#endif  // PLANSYS2_EXECUTOR__BEHAVIOR_TREE__ACTION_REFINEMENT_NODE_HPP_