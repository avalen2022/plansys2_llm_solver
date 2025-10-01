#ifndef PLANSYS2_SOLVER__SOLVERBASE_HPP_
#define PLANSYS2_SOLVER__SOLVERBASE_HPP_

#include <memory>
#include <string>

#include "plansys2_msgs/msg/solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals; 

namespace plansys2
{

class SolverBase
{
public:
  using Ptr = std::shared_ptr<plansys2::SolverBase>;
  SolverBase() {}
  virtual ~SolverBase() {}

  virtual void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, const std::string & plugin_name) = 0;

  virtual void initialize(const std::string & node_name) = 0;

  virtual std::optional<plansys2_msgs::msg::Solver> solve(
    const std::string & domain, const std::string & problem,
    const std::string & question,
    const std::string & action_file,
    const std::string & node_namespace = "",
    const rclcpp::Duration resolution_timeout = 120s) = 0;

  virtual void cancel() {cancel_requested_ = true;}

protected:
  // Lifecycle node pointer.
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;
  // Flag indicating if cancellation was requested.
  bool cancel_requested_;
};

} // namespace plansys2

#endif  // PLANSYS2_SOLVER__SOLVERBASE_HPP_