#include <string>
#include <memory>
#include <iostream>
#include <fstream>

#include "plansys2_solver/SolverNode.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace plansys2
{

SolverNode::SolverNode()
: rclcpp_lifecycle::LifecycleNode("solver"),
  lp_loader_("plansys2_solver", "plansys2::SolverBase"),
  default_ids_{},
  default_types_{},
  resolution_timeout_(120s)
{
  declare_parameter("solver_plugins", default_ids_);
  double timeout = resolution_timeout_.seconds();
  declare_parameter("solver_timeout", timeout);
}

SolverNode::~SolverNode()
{
  std::vector<std::string> loaded_libraries = lp_loader_.getRegisteredLibraries();

  for (const auto & library : loaded_libraries) {
    try {
      lp_loader_.unloadLibraryForClass(library);
      std::cout << "Successfully unloaded library: " << library << std::endl;
    } catch (const pluginlib::LibraryUnloadException & e) {
      std::cerr << "Failed to unload library: " << library <<
        ". Error: " << e.what() << std::endl;
    }
  }
}

using CallbackReturnT =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
SolverNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void) state;
  auto node = shared_from_this();
  double timeout;

  RCLCPP_INFO(this->get_logger(), "[%s] Configuring...", get_name());

  get_parameter("solver_plugins", workers_ids_);
  get_parameter("solver_timeout", timeout);

  resolution_timeout_ = rclcpp::Duration((int32_t)timeout, 0);

  if (!workers_ids_.empty()) {
    if (workers_ids_ == default_ids_) {
      for (size_t i = 0; i < default_ids_.size(); ++i) {
        plansys2::declare_parameter_if_not_declared(
        node, default_ids_[i] + ".plugin",
        rclcpp::ParameterValue(default_types_[i]));
      }
    }
    worker_types_.resize(workers_ids_.size());

    for (size_t i = 0; i != worker_types_.size(); i++) {
      try {
        worker_types_[i] = plansys2::get_plugin_type_param(node, workers_ids_[i]);
        plansys2::SolverBase::Ptr solver =
        lp_loader_.createUniqueInstance(worker_types_[i]);

        solver->configure(node, workers_ids_[i]);

        RCLCPP_INFO(
        this->get_logger(), "Created solver : %s of type %s",
        workers_ids_[i].c_str(), worker_types_[i].c_str());
        resolutors_.insert({workers_ids_[i], solver});
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(this->get_logger(), "Failed to create solver. Exception: %s", ex.what());
        exit(-1);
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s] Error not plugin set", get_name());
    return CallbackReturnT::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "[%s] Solver Timeout %g", get_name(), resolution_timeout_.seconds());

  get_solve_service_ = create_service<plansys2_msgs::srv::GetSolve>(
    "solver/get_solve",
    std::bind(
      &SolverNode::get_solve_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  std::string nm = get_namespace();
  action_subs_ = this->create_subscription<plansys2_msgs::msg::ActionExecution>(
    nm + "actions_hub", rclcpp::SensorDataQoS().reliable(),
    std::bind(&SolverNode::action_hub_callback, this, _1));

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SolverNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(this->get_logger(), "[%s] Activating...", get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Activated", get_name());
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SolverNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(this->get_logger(), "[%s] Deactivating...", get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Deactivated", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SolverNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(this->get_logger(), "[%s] Cleaning up...", get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Cleaned up", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SolverNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_INFO(this->get_logger(), "[%s] Shutting down...", get_name());
  RCLCPP_INFO(this->get_logger(), "[%s] Shutted down", get_name());

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SolverNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void) state;
  RCLCPP_ERROR(this->get_logger(), "[%s] Error transition", get_name());

  return CallbackReturnT::SUCCESS;
}

void SolverNode::action_hub_callback(plansys2_msgs::msg::ActionExecution::UniquePtr msg)
{
  if (msg == msg_ || (msg_ != nullptr && msg_->completion + 0.30f >= msg->completion)) {
    return;
  }

  std::ofstream action_file_("/tmp/solver_action_hub.txt", std::ios::app);
  if (action_file_.is_open()) {
    action_file_ << "-----------------------------\n";
    action_file_ << "Node ID: " << msg->node_id << "\n";
    action_file_ << "Type: " << msg->type << "\n";

    action_file_ << "Action: " << msg->action << "\n";
    action_file_ << "Arguments:\n";
    for (const auto &arg : msg->arguments) {
      action_file_ << "  - " << arg << "\n";
    }

    action_file_ << "Success: " << (msg->success ? "true" : "false") << "\n";
    action_file_ << "Completion: " << msg->completion << "\n";
    action_file_ << "Status: " << msg->status << "\n";

    action_file_ << "-----------------------------\n\n";
  } else {
    RCLCPP_WARN(this->get_logger(), "No se pudo abrir el archivo de log");
  }

  msg_ = std::move(msg);
}

void
SolverNode::get_solve_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_msgs::srv::GetSolve::Request> request,
  const std::shared_ptr<plansys2_msgs::srv::GetSolve::Response> response)
{
  (void) request_header;
  auto solves = get_solve_array(request->domain, request->problem, "action_file.txt");

  if (!solves.solver_array.empty()) {
    response->status = plansys2_msgs::srv::GetSolve::Response::SUCCESS;
    response->solver = solves.solver_array.front();
  } else {
    response->status = plansys2_msgs::srv::GetSolve::Response::ERROR;
    response->error_info = "Resolution not found";
  }
}

plansys2_msgs::msg::SolverArray
SolverNode::get_solve_array(const std::string & domain, const std::string & problem, const std::string action_file)
{
  std::map<std::string, std::future<std::optional<plansys2_msgs::msg::Solver>>> futures;
  std::map<std::string, std::optional<plansys2_msgs::msg::Solver>> results;

  for (auto & resol : resolutors_) {
    futures[resol.first] = std::async(std::launch::async,
      &plansys2::SolverBase::solve, resol.second,
      domain, problem, action_file, get_namespace(), resolution_timeout_);
  }

  auto start = now();

  size_t pending_result = resolutors_.size();
  while (pending_result > 0 && now() - start < resolution_timeout_) {
    for (auto & fut : futures) {
      if (results.find(fut.first) == results.end()) {
        if (fut.second.wait_for(1ms) == std::future_status::ready) {
          results[fut.first] = fut.second.get();
          pending_result--;
        }
      }
    }
  }

  for (auto & resol : resolutors_) {
    if (results.find(resol.first) == results.end()) {
      resol.second->cancel();
    }
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (auto & fut : futures) {
    if (results.find(fut.first) == results.end()) {
      try {
        fut.second.get();
      } catch (const std::exception & e) {
        RCLCPP_WARN_STREAM(
          get_logger(), "Exception while destroying future for "
            << fut.first << ": " << e.what());
      }
    }
  }

  plansys2_msgs::msg::SolverArray solves;
  for (auto & result : results) {
    if (result.second.has_value()) {
      solves.solver_array.push_back(result.second.value());
    }
  }

  return solves;
}

}
