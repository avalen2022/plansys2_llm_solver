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
