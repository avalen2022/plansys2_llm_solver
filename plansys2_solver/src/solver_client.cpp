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
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  plansys2::SolverClient solver_client;

  std::string domain = "(define (domain demo))";
  std::string problem = "(define (problem demo_problem) (:domain demo))";
  std::string observation = "Do an echo of the domain i send it to you, just rewrite the same text i put it on domain";
  std::string node_namespace = "";

  auto result = solver_client.getReplanificateSolve(domain, problem, observation, node_namespace);

  if (result.has_value()) {
    RCLCPP_INFO(rclcpp::get_logger("main"), "Solver respondió correctamente");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "No se obtuvo respuesta del solver");
  }

  rclcpp::shutdown();
  return 0;
}
