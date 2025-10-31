#ifndef PLANSYS2_SOLVER__SOLVERINTERFACE_HPP_
#define PLANSYS2_SOLVER__SOLVERINTERFACE_HPP_

#include "plansys2_msgs/msg/solver.hpp"
#include "plansys2_msgs/msg/solver_array.hpp"

namespace plansys2
{
class SolverInterface
{
public:

    SolverInterface() {}

    virtual std::optional<plansys2_msgs::msg::Solver> getReplanificateSolve(
    const std::string & domain, const std::string & problem,
    const std::string & prompt,
    const std::string & node_namespace) = 0;

    virtual std::optional<plansys2_msgs::msg::SolverArray> getReplanificateSolveArray(
    const std::string & domain, const std::string & problem,
    const std::string & prompt,
    const std::string & node_namespace) = 0;
};

}

#endif  // PLANSYS2_SOLVER__SOLVERINTERFACE_HPP_