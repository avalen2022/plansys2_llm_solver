#ifndef PLANSYS2_SOLVER__SOLVERNODE_HPP_
#define PLANSYS2_SOLVER__SOLVERNODE_HPP_

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>
#include <fstream>

#include "plansys2_solver/SolverBase.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_solver_msgs/msg/solver_array.hpp"
#include "plansys2_solver_msgs/srv/get_solve.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pluginlib/class_loader.hpp"

namespace plansys2
{
class SolverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for the SolverNode.
   */
  SolverNode();

  /**
   * @brief Destructor for the SolverNode. Unloads all pluginlib-loaded solvers.
   */
  ~SolverNode();

  using CallbackReturnT =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  using SolverMap = std::unordered_map<std::string, plansys2::SolverBase::Ptr>;
  /**
   * @brief Configures the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if configuration is successful, FAILURE otherwise.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if activation is successful, FAILURE otherwise.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if deactivation is successful, FAILURE otherwise.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if cleanup is successful, FAILURE otherwise.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if shutdown is successful, FAILURE otherwise.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the node.
   *
   * @param[in] state The current lifecycle state.
   * @return SUCCESS if error handling is successful, FAILURE otherwise.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Service callback that runs the configured solver plugins over the current PDDL
   *        domain, problem and observation, returning a Solver message with the requested
   *        state-delta (add/remove predicates) plus a classification.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request with domain, problem and observation strings.
   * @param[out] response Service response with the Solver message and status.
   */
  void get_solve_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_solver_msgs::srv::GetSolve::Request> request,
    const std::shared_ptr<plansys2_solver_msgs::srv::GetSolve::Response> response);

  /**
   * @brief Runs every loaded solver plugin in parallel (std::async) and collects their
   *        results, honouring the configured solve_timeout. Plugins that do not finish
   *        in time are asked to cancel() and their futures are drained before return.
   *
   * @param[in] domain PDDL domain text.
   * @param[in] problem PDDL problem text.
   * @param[in] observation Short natural-language description of what triggered the solve.
   * @param[in] action_file Content of the action-hub log file (produced by action_hub_callback).
   * @return SolverArray with one Solver entry per plugin that produced a result.
   */
  plansys2_solver_msgs::msg::SolverArray get_solve_array(
    const std::string & domain, const std::string & problem, const std::string & observation, const std::string & action_file);

  /**
   * @brief Subscription callback for /actions_hub. Appends a structured entry to the
   *        action-hub log file with per-action metadata (type, args, success, completion,
   *        status). Deduplicates against the previous message to avoid flooding the log
   *        with near-identical feedback ticks.
   *
   * @param[in] msg Incoming ActionExecution message. Ownership is taken: after the
   *                dedup check the message is moved into previous_action_msg_.
   */
  void action_hub_callback(plansys2_msgs::msg::ActionExecution::UniquePtr msg);

private:
  pluginlib::ClassLoader<plansys2::SolverBase> plugin_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> workers_ids_;
  std::vector<std::string> worker_types_;
  rclcpp::Duration solve_timeout_;

  SolverMap solvers_;

  rclcpp::Service<plansys2_solver_msgs::srv::GetSolve>::SharedPtr get_solve_service_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_subs_;

  plansys2_msgs::msg::ActionExecution::UniquePtr previous_action_msg_;
  std::string action_file_path_;
};

template<typename NodeT>
void declare_parameter_if_not_declared(
  NodeT node,
  const std::string & param_name,
  const rclcpp::ParameterValue & default_value = rclcpp::ParameterValue(),
  const rcl_interfaces::msg::ParameterDescriptor & parameter_descriptor =
  rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name)) {
    node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
}

/**
 * @brief Get the plugin type parameter for a given plugin name.
 *
 * @tparam NodeT Type of the ROS2 node.
 * @param[in] node Node to get the parameter from.
 * @param[in] plugin_name Name of the plugin.
 * @return String containing the plugin type.
 */
template<typename NodeT>
std::string get_plugin_type_param(
  NodeT node,
  const std::string & plugin_name)
{
  declare_parameter_if_not_declared(node, plugin_name + ".plugin", rclcpp::ParameterValue(""));
  std::string plugin_type;
  if (!node->get_parameter(plugin_name + ".plugin", plugin_type)) {
    RCLCPP_FATAL(node->get_logger(), "'plugin' param not defined for %s", plugin_name.c_str());
    exit(-1);
  }
  return plugin_type;
}

inline std::string read_file(const std::string & file_path)
{
  std::ifstream file(file_path);
  if (!file) {
    return "";
  }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

}

#endif  // PLANSYS2_SOLVER__SOLVERNODE_HPP_