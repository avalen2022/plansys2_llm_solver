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

#ifndef PLANSYS2_LLAMA_SOLVER__LLAMA_SOLVER_HPP_
#define PLANSYS2_LLAMA_SOLVER__LLAMA_SOLVER_HPP_

#include <filesystem>
#include <optional>
#include <memory>
#include <string>

#include "plansys2_solver/SolverBase.hpp"

// using namespace std::chrono_literals;
using std::chrono_literals::operator""s;

namespace plansys2
{

/**
 * @class plansys2::POPFPlanSolver
 * @brief Plan solver implementation that uses the POPF planning system.
 *
 * This class implements the PlanSolverBase interface using the POPF (Partial Order Planning Forward)
 * planner. It handles writing domain and problem files to disk, executing the planner,
 * and parsing the resulting plan.
 */
class LLAMASolver : public SolverBase
{
public:
  /**
   * @brief Default constructor.
   */
  LLAMASolver();

  /**
   * @brief Configures the solver with runtime parameters.
   *
   * Sets up necessary parameters including command-line arguments for the planner
   * and the output directory for temporary files.
   *
   * @param[in] lc_node Pointer to the lifecycle node.
   * @param[in] plugin_name Name of the plugin to use for parameter namespacing.
   */

  std::optional<std::filesystem::path> create_folders(const std::string & node_namespace);

  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
    const std::string & plugin_name) override;

  void initialize(const std::string & node_name) override;

  virtual std::optional<plansys2_msgs::msg::Solver> solve(
    const std::string & domain, const std::string & problem,
    const std::string & question,
    const std::string & action_file,
    const std::string & node_namespace = "",
    const rclcpp::Duration resolution_timeout = 15s) override;

  // Summarize the raw action hub log into a compact format for the LLM.
  // limited=true:  only FINISH/CANCEL entries (compact, fits small context windows)
  // limited=false: all entries in compact one-line format (needs larger context)
  static std::string summarize_action_log(
    const std::string & raw_log, bool limited = true);

private:
  std::string arguments_parameter_name_;
  std::string output_dir_parameter_name_;
  std::string llm_debug_parameter_;
  std::string prompt_debug_parameter_;
  std::string summarize_mode_parameter_;
  std::string model_yaml_parameter_name_;
  bool cancel_requested_;

};

}  // namespace plansys2

#endif  // PLANSYS2_LLAMA_SOLVER__LLAMA_SOLVER_HPP_
