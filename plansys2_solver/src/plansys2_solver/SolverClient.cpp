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

#include "plansys2_solver/SolverClient.hpp"

namespace plansys2
{

SolverClient::SolverClient()
{
  node_ = rclcpp::Node::make_shared("solver_client");

  get_solve_client_ = node_->create_client<plansys2_solver_msgs::srv::GetSolve>("solver/get_solve");

  double timeout = 240;
  node_->declare_parameter("solver_timeout", timeout);

  node_->get_parameter("solver_timeout", timeout);
  solve_timeout_ = rclcpp::Duration((int32_t)timeout, 0);
  RCLCPP_INFO(
    node_->get_logger(), "Solver Client created with timeout %g",
    solve_timeout_.seconds());
}

std::optional<plansys2_solver_msgs::msg::Solver>
SolverClient::getReplanificateSolve(
  const std::string & domain, const std::string & problem,
  const std::string & observation,
  const std::string & node_namespace)
{
  (void)node_namespace;
  while (!get_solve_client_->wait_for_service(std::chrono::seconds(30))) {
    if (!rclcpp::ok()) {
      return {};
    }
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_solve_client_->get_service_name() <<
        " service  client: waiting for service to appear...");
  }
  int32_t timeout = solve_timeout_.seconds();
  if (timeout <= 0) {
    RCLCPP_WARN(node_->get_logger(), "Solver timeout was %d, falling back to 150s", timeout);
    timeout = 150;
  }

  RCLCPP_DEBUG(node_->get_logger(), "Get Solver service call with time out %d", timeout);

  auto request = std::make_shared<plansys2_solver_msgs::srv::GetSolve::Request>();
  request->domain = domain;
  request->problem = problem;
  request->observation = observation;

  auto future_result = get_solve_client_->async_send_request(request);

  auto outresult = rclcpp::spin_until_future_complete(
    node_, future_result,
    std::chrono::seconds(timeout));
  if (outresult != rclcpp::FutureReturnCode::SUCCESS) {
    if (outresult == rclcpp::FutureReturnCode::TIMEOUT) {
      RCLCPP_ERROR(node_->get_logger(), "Get Solver service call timed out");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "Get Solver service call failed");
    }
    return {};
  }

  auto result = *future_result.get();

  if (result.status == plansys2_solver_msgs::srv::GetSolve::Response::SUCCESS) {
    return result.solver;
  } else {
    RCLCPP_ERROR_STREAM(
      node_->get_logger(),
      get_solve_client_->get_service_name() << ": " <<
        result.error_info);
    return {};
  }
}

std::optional<plansys2_solver_msgs::msg::SolverArray>
plansys2::SolverClient::getReplanificateSolveArray(
    const std::string & domain,
    const std::string & problem,
    const std::string & observation,
    const std::string & node_namespace)
{
    (void) domain;
    (void) problem;
    (void) observation;
    (void) node_namespace;
    return {};
}

}  // namespace plansys2
