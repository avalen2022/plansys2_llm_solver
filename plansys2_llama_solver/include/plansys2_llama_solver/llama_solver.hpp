// Copyright 2026 Intelligent Robotics Lab
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

using std::chrono_literals::operator""s;

namespace plansys2
{

/**
 * @class plansys2::LLAMASolver
 * @brief Solver plugin that uses a local LLM (via llama_ros) as a reasoner over PDDL state.
 *
 * Implements the SolverBase interface. On each solve() call it writes the current
 * domain, problem, and observation to disk, spawns the llama_ros model server and a
 * prompt process (fork+exec on `ros2 llama`), captures the JSON reply and fills a
 * plansys2_solver_msgs::msg::Solver with the classification (CORRECT / MODIFY_PLAN /
 * MODIFY_DOMAIN / UNSOLVABLE) and the predicate add/remove lists. The fork+exec design
 * is deliberate so the same plugin shape can wrap other backends (local models, paid APIs)
 * without pulling their SDKs into the solver process.
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

  virtual std::optional<plansys2_solver_msgs::msg::Solver> solve(
    const std::string & domain, const std::string & problem,
    const std::string & observation,
    const std::string & action_file,
    const std::string & node_namespace = "",
    const rclcpp::Duration solve_timeout = 15s) override;

private:
  std::string arguments_parameter_name_;
  std::string output_dir_parameter_name_;
  std::string llm_debug_parameter_;
  std::string model_yaml_parameter_name_;

};

}  // namespace plansys2

#endif  // PLANSYS2_LLAMA_SOLVER__LLAMA_SOLVER_HPP_
