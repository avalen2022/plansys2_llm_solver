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

#ifndef PLANSYS2_SOLVER__SOLVERCLIENT_HPP_
#define PLANSYS2_SOLVER__SOLVERCLIENT_HPP_

#include <optional>
#include <algorithm>
#include <string>
#include <vector>
#include <memory>

#include "plansys2_solver/SolverInterface.hpp"

#include "plansys2_solver_msgs/srv/get_solve.hpp"

#include "rclcpp/rclcpp.hpp"

namespace plansys2
{
class SolverClient : public SolverInterface
{
public:
    SolverClient();

    std::optional<plansys2_solver_msgs::msg::Solver> getReplanificateSolve(
        const std::string & domain, const std::string & problem,
        const std::string & observation,
        const std::string & node_namespace = "") override;


    std::optional<plansys2_solver_msgs::msg::SolverArray> getReplanificateSolveArray(
        const std::string & domain, const std::string & problem,
        const std::string & observation,
        const std::string & node_namespace = "") override;

private:
    rclcpp::Client<plansys2_solver_msgs::srv::GetSolve>::SharedPtr
        get_solve_client_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Duration solve_timeout_ = rclcpp::Duration(300, 0);
};

}

#endif  // PLANSYS2_SOLVER__SOLVERCLIENT_HPP_