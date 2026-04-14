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
        const std::string & prompt,
        const std::string & node_namespace = "");


    std::optional<plansys2_solver_msgs::msg::SolverArray> getReplanificateSolveArray(
        const std::string & domain, const std::string & problem,
        const std::string & prompt,
        const std::string & node_namespace = "");

private:
    rclcpp::Client<plansys2_solver_msgs::srv::GetSolve>::SharedPtr
        get_solve_client_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Duration solver_timeout_ = rclcpp::Duration(300, 0);
};

}

#endif  // PLANSYS2_SOLVER__SOLVERBASE_HPP_