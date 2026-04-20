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

#ifndef PLANSYS2_SOLVER__SOLVERINTERFACE_HPP_
#define PLANSYS2_SOLVER__SOLVERINTERFACE_HPP_

#include "plansys2_solver_msgs/msg/solver.hpp"
#include "plansys2_solver_msgs/msg/solver_array.hpp"

namespace plansys2
{
class SolverInterface
{
public:

    SolverInterface() {}

    virtual std::optional<plansys2_solver_msgs::msg::Solver> getReplanificateSolve(
    const std::string & domain, const std::string & problem,
    const std::string & observation,
    const std::string & node_namespace) = 0;

    virtual std::optional<plansys2_solver_msgs::msg::SolverArray> getReplanificateSolveArray(
    const std::string & domain, const std::string & problem,
    const std::string & observation,
    const std::string & node_namespace) = 0;
};

}

#endif  // PLANSYS2_SOLVER__SOLVERINTERFACE_HPP_