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

#include <sys/types.h>

#include <filesystem>
#include <optional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "plansys2_solver/SolverBase.hpp"

using std::chrono_literals::operator""s;

namespace plansys2
{

/**
 * @class plansys2::LLAMASolver
 * @brief Solver plugin that uses a local LLM (via llama_ros) as a reasoner over PDDL state.
 *
 * Implements the SolverBase interface. Forks `ros2 llama launch` to start the
 * llama_node server and `ros2 llama prompt` to send prompts, capturing the JSON
 * reply and filling a plansys2_solver_msgs::msg::Solver with the classification
 * (CORRECT / MODIFY_PLAN / MODIFY_DOMAIN / UNSOLVABLE) and the predicate
 * add/remove lists. The fork+exec design lets the same plugin shape wrap other
 * backends (local models, paid APIs) without pulling their SDKs into the solver
 * process. When pre_launch=true the server is started once at configure() and
 * kept alive for the plugin's lifetime.
 */
class LLAMASolver : public SolverBase
{
public:
  /**
   * @brief Default constructor.
   */
  LLAMASolver();

  /**
   * @brief Destructor. Terminates the llama_node child if one is alive
   *        (pre_launch mode) so no orphaned LLM server survives plugin unload.
   */
  ~LLAMASolver() override;

  /**
   * @brief Configures the solver plugin and snapshots its parameters.
   *
   * Declares/reads the plugin-namespaced params (launch_extra_args, model_yaml,
   * llm_debug, pre_launch, output_dir). When pre_launch=true, also forks the
   * llama_node server so the first solve() does not pay the model-load cost.
   *
   * @param[in] lc_node Pointer to the owning lifecycle node.
   * @param[in] plugin_name Name used for parameter namespacing.
   */
  void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
    const std::string & plugin_name) override;

  void initialize(const std::string & node_name) override;

  std::optional<plansys2_solver_msgs::msg::Solver> solve(
    const std::string & domain, const std::string & problem,
    const std::string & observation,
    const std::string & action_file,
    const std::string & node_namespace = "",
    const rclcpp::Duration solve_timeout = 15s) override;

private:
  std::optional<std::filesystem::path> create_folders(const std::string & node_namespace);

  /**
   * @brief Fork `ros2 llama launch` and, if llm_debug, spawn the drain thread.
   * Idempotent; re-entry with an alive server is a no-op.
   */
  void launch_llm_server();

  /**
   * @brief SIGINT the llama_node child, reap it and join the drain thread.
   * Idempotent; destructor-safe (does not touch lc_node_).
   */
  void shutdown_llm_server();

  std::string launch_extra_args_parameter_name_;
  std::string prompt_extra_args_parameter_name_;
  std::string output_dir_parameter_name_;
  std::string llm_debug_parameter_name_;
  std::string model_yaml_parameter_name_;
  std::string pre_launch_parameter_name_;

  // Snapshotted at configure() — rclcpp param APIs are not fork-safe.
  std::string yaml_path_;
  std::vector<std::string> launch_extras_;
  bool llm_debug_ = false;
  bool pre_launch_ = true;

  pid_t launch_pid_ = -1;
  std::thread launch_drain_;
};

}  // namespace plansys2

#endif  // PLANSYS2_LLAMA_SOLVER__LLAMA_SOLVER_HPP_
