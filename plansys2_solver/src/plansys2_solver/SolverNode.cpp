#include <string>
#include <memory>
#include <fstream>

#include "plansys2_solver/SolverNode.hpp"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace plansys2
{

SolverNode::SolverNode()
: rclcpp_lifecycle::LifecycleNode("solver"),
  plugin_loader_("plansys2_solver", "plansys2::SolverBase"),
  default_ids_{},
  default_types_{},
  solve_timeout_(120s)
{
  declare_parameter("solver_plugins", default_ids_);
  double timeout = solve_timeout_.seconds();
  declare_parameter("solver_timeout", timeout);
}

SolverNode::~SolverNode()
{
  solvers_.clear();

  std::vector<std::string> loaded_libraries = plugin_loader_.getRegisteredLibraries();

  for (const auto & library : loaded_libraries) {
    try {
      plugin_loader_.unloadLibraryForClass(library);
      RCLCPP_DEBUG(get_logger(), "Unloaded library: %s", library.c_str());
    } catch (const pluginlib::LibraryUnloadException & e) {
      RCLCPP_WARN(get_logger(), "Failed to unload library %s: %s", library.c_str(), e.what());
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

  std::string nm = get_namespace();
  if (nm != "/") {
    std::filesystem::create_directories("/tmp" + nm);
    action_file_path_ = "/tmp" + nm + "/solver_action_hub.txt";
  } else {
    nm = "";
    action_file_path_ = "/tmp/solver_action_hub.txt";
  }

  std::ofstream action_file_(action_file_path_, std::ios::out);
  if (!action_file_) {
    RCLCPP_ERROR(this->get_logger(), "[%s] Error can't open the .txt", get_name());
    return CallbackReturnT::FAILURE;
  }
  action_file_.close();


  solve_timeout_ = rclcpp::Duration((int32_t)timeout, 0);

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
        plugin_loader_.createUniqueInstance(worker_types_[i]);

        solver->configure(node, workers_ids_[i]);

        RCLCPP_INFO(
        this->get_logger(), "Created solver : %s of type %s",
        workers_ids_[i].c_str(), worker_types_[i].c_str());
        solvers_.insert({workers_ids_[i], solver});
      } catch (const pluginlib::PluginlibException & ex) {
        RCLCPP_FATAL(this->get_logger(), "Failed to create solver. Exception: %s", ex.what());
        exit(-1);
      }
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "[%s] Error not plugin set", get_name());
    return CallbackReturnT::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "[%s] Solver Timeout %g", get_name(), solve_timeout_.seconds());

  get_solve_service_ = create_service<plansys2_solver_msgs::srv::GetSolve>(
    "solver/get_solve",
    std::bind(
      &SolverNode::get_solve_service_callback,
      this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3));

  action_subs_ = this->create_subscription<plansys2_msgs::msg::ActionExecution>(
    nm + "/actions_hub", rclcpp::SensorDataQoS().reliable(),
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
  RCLCPP_INFO(this->get_logger(), "[%s] Shut down", get_name());

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
  // Deduplicate against the previous message: skip if the same action/type/status/success
  // and completion has not advanced by at least 0.30 — avoids flooding the log with
  // near-identical FEEDBACK ticks while still capturing meaningful progress.
  if (previous_action_msg_ != nullptr &&
      msg->action == previous_action_msg_->action &&
      msg->type == previous_action_msg_->type &&
      msg->status == previous_action_msg_->status &&
      msg->success == previous_action_msg_->success &&
      std::abs(msg->completion - previous_action_msg_->completion) < 0.30f)
  {
    return;
  }

  std::ofstream action_file_(action_file_path_, std::ios::app);
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
    RCLCPP_WARN(this->get_logger(), "Failed to open action hub log file");
  }

  previous_action_msg_ = std::move(msg);
}

void
SolverNode::get_solve_service_callback(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<plansys2_solver_msgs::srv::GetSolve::Request> request,
  const std::shared_ptr<plansys2_solver_msgs::srv::GetSolve::Response> response)
{
  (void) request_header;
  std::string action_file = read_file(action_file_path_);
  auto solves = get_solve_array(request->domain, request->problem, request->observation, action_file);

  if (!solves.solver_array.empty()) {
    const auto & first = solves.solver_array.front();
    response->solver = first;
    if (first.classification == plansys2_solver_msgs::msg::Solver::ERROR) {
      response->status = plansys2_solver_msgs::srv::GetSolve::Response::ERROR;
      response->error_info = "Solver returned ERROR classification (LLM output unparseable)";
    } else {
      response->status = plansys2_solver_msgs::srv::GetSolve::Response::SUCCESS;
    }
  } else {
    response->status = plansys2_solver_msgs::srv::GetSolve::Response::ERROR;
    response->error_info = "Resolution not found";
  }
}

plansys2_solver_msgs::msg::SolverArray
SolverNode::get_solve_array(const std::string & domain, const std::string & problem, const std::string & observation, const std::string & action_file)
{
  std::map<std::string, std::future<std::optional<plansys2_solver_msgs::msg::Solver>>> futures;
  std::map<std::string, std::optional<plansys2_solver_msgs::msg::Solver>> results;

  for (auto & [solver_id, solver] : solvers_) {
    futures[solver_id] = std::async(std::launch::async,
      &plansys2::SolverBase::solve, solver,
      domain, problem, observation, action_file, get_namespace(), solve_timeout_);
  }

  auto start = now();

  size_t pending_result = solvers_.size();
  while (pending_result > 0 && now() - start < solve_timeout_) {
    for (auto & fut : futures) {
      if (results.find(fut.first) == results.end()) {
        if (fut.second.wait_for(1ms) == std::future_status::ready) {
          results[fut.first] = fut.second.get();
          pending_result--;
        }
      }
    }
  }

  for (auto & [solver_id, solver] : solvers_) {
    if (results.find(solver_id) == results.end()) {
      solver->cancel();
    }
  }
  // Give cancelled solvers a short window to observe cancel_requested_ and return
  // before we block on their futures below. 100 ms is a safety margin over the
  // typical cancel-observation latency in SolverBase::cancel().
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

  plansys2_solver_msgs::msg::SolverArray solves;
  for (auto & result : results) {
    if (result.second.has_value()) {
      solves.solver_array.push_back(result.second.value());
    }
  }

  return solves;
}

}
