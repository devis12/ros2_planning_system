// BASED ON POPF PLANNER - LICENSE? TODO


// Copyright 2019 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <sys/stat.h>
#include <sys/types.h>

#include <filesystem>
#include <string>
#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <nlohmann/json.hpp> // JSON library

using json = nlohmann::json;

#include "plansys2_msgs/msg/plan_item.hpp"
#include "plansys2_llm_plan_solver/llm_plan_solver.hpp"

namespace plansys2
{

LLMPlanSolver::LLMPlanSolver()
{
}

void LLMPlanSolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr & lc_node,
  const std::string & plugin_name)
{
  parameter_name_ = plugin_name + ".arguments";
  lc_node_ = lc_node;
  lc_node_->declare_parameter<std::string>(parameter_name_, "-gpt");
}

std::optional<plansys2_msgs::msg::Plan>
LLMPlanSolver::getPlan(
  const std::string & domain, const std::string & problem,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    std::filesystem::path tp = std::filesystem::temp_directory_path();
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        tp /= p;
      }
    }
    std::filesystem::create_directories(tp);
  }

  plansys2_msgs::msg::Plan ret;
  std::ofstream domain_out("/tmp/" + node_namespace + "/domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/problem.pddl");
  problem_out << problem;
  problem_out.close();

  // call LLM planner node to get the plan
  std::string command = "ros2 run llm_planner llm_planner " +
                      lc_node_->get_parameter(parameter_name_).value_to_string() +
                      " /tmp" + node_namespace + "/domain.pddl /tmp" + node_namespace +
                      "/problem.pddl /tmp" + node_namespace + "/plan";
  
  // std::cout << "Executing command: " << command << std::endl;
  
  system(command.c_str());

  std::string line;
  std::ifstream plan_file("/tmp/" + node_namespace + "/plan");
  bool solution = false;

  if (!plan_file.is_open()) {
    std::cerr << "Failed to open plan file: " << "/tmp/" + node_namespace + "/plan" << std::endl;
    return {};
  }

  // Parse LLM response
  json plan_json;
  try {
      plan_file >> plan_json;
      // std::cerr << "JSON parse successful" << std::endl;
      // std::cout << "Plan JSON:\n" << plan_json.dump(2) << std::endl;
  } catch (json::parse_error &e) {
      std::cerr << "JSON parsing error: " << e.what() << std::endl;
      return {};
  }
  plan_file.close();

  if (!plan_json["solution_found"].get<bool>()) {
      return {};
  }
  // std::cout << "Solution found, beginning plan parsing..." << std::endl;

  for (const auto &entry : plan_json["plan"]) {
      plansys2_msgs::msg::PlanItem item;
      item.time = entry["time"].get<float>();
      item.action = "(" + entry["action"].get<std::string>() + ")";
      item.duration = entry["duration"].get<float>();
      ret.items.push_back(item);
      // std::cout << "Pushed new action / PlanItem to plan: "
      //     << "time: " << item.time
      //     << ", action: " << item.action
      //     << ", duration: " << item.duration
      //     << std::endl;
  }

  if (ret.items.empty()) {
    std::cout << "Parse unsuccessful" << std::endl;
    return {};
  } else {
    std::cout << "Parse complete" << std::endl;
    return ret;
  }
}

bool
LLMPlanSolver::is_valid_domain( // KEEPING POPF AS DOMAIN VALIDATOR FOR NOW
  const std::string & domain,
  const std::string & node_namespace)
{
  if (node_namespace != "") {
    mkdir(("/tmp/" + node_namespace).c_str(), ACCESSPERMS);
  }

  std::ofstream domain_out("/tmp/" + node_namespace + "/check_domain.pddl");
  domain_out << domain;
  domain_out.close();

  std::ofstream problem_out("/tmp/" + node_namespace + "/check_problem.pddl");
  problem_out << "(define (problem void) (:domain plansys2))";
  problem_out.close();

  std::cout << "Validating domain with POPF... " << std::endl;

  system(
    ("ros2 run popf popf /tmp/" + node_namespace + "/check_domain.pddl /tmp/" +
    node_namespace + "/check_problem.pddl > /tmp/" + node_namespace + "/check.out").c_str());

  std::ifstream plan_file("/tmp/" + node_namespace + "/check.out");

  std::string result((std::istreambuf_iterator<char>(plan_file)),
    std::istreambuf_iterator<char>());

  return result.find("Solution Found") != result.npos;
}

}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::LLMPlanSolver, plansys2::PlanSolverBase);
