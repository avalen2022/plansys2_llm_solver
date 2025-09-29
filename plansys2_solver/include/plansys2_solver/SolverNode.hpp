#ifndef PLANSYS2_SOLVER__SOLVERNODE_HPP_
#define PLANSYS2_SOLVER__SOLVERNODE_HPP_

#include <memory>
#include <unordered_map>
#include <string>
#include <vector>

#include "plansys2_solver/SolverBase.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_msgs/msg/solver_array.hpp"
#include "plansys2_msgs/srv/get_solve.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "pluginlib/class_loader.hpp"

namespace plansys2
{
class SolverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /**
   * @brief Constructor for the PlannerNode.
   */
  SolverNode();

  /**
   * @brief Destructor for the PlannerNode.
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
   * @brief Service callback to generate a plan for a PDDL problem.
   *
   * @param[in] request_header ROS service request header.
   * @param[in] request Service request containing domain and problem PDDL strings.
   * @param[out] response Service response containing the generated plan and success status.
   */
  void get_solve_service_callback(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<plansys2_msgs::srv::GetSolve::Request> request,
    const std::shared_ptr<plansys2_msgs::srv::GetSolve::Response> response);

  plansys2_msgs::msg::SolverArray get_solve_array(
    const std::string & domain, const std::string & problem, std::string action_file);

  void action_hub_callback(plansys2_msgs::msg::ActionExecution::UniquePtr msg);

private:
  pluginlib::ClassLoader<plansys2::SolverBase> lp_loader_;
  std::vector<std::string> default_ids_;
  std::vector<std::string> default_types_;
  std::vector<std::string> workers_ids_;
  std::vector<std::string> worker_types_;
  rclcpp::Duration resolution_timeout_;

  SolverMap resolutors_;

  rclcpp::Service<plansys2_msgs::srv::GetSolve>::SharedPtr get_solve_service_;
  rclcpp::Subscription<plansys2_msgs::msg::ActionExecution>::SharedPtr action_subs_;

  plansys2_msgs::msg::ActionExecution::UniquePtr msg_;
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

}

#endif  // PLANSYS2_SOLVER__SOLVERNODE_HPP_