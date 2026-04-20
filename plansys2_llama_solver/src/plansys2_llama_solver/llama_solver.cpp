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


#include <cerrno>
#include <chrono>
#include <fstream>
#include <fcntl.h>
#include <poll.h>
#include <thread>
#include <vector>
#include <sys/wait.h>
#include <unistd.h>

#include "plansys2_llama_solver/llama_solver.hpp"
#include "rclcpp/logging.hpp"

namespace plansys2
{

LLAMASolver::LLAMASolver()
{
}

LLAMASolver::~LLAMASolver()
{
  shutdown_llm_server();
}

void LLAMASolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  SolverBase::configure(lc_node, plugin_name);

  launch_extra_args_parameter_name_ = plugin_name + ".launch_extra_args";
  prompt_extra_args_parameter_name_ = plugin_name + ".prompt_extra_args";
  output_dir_parameter_name_ = plugin_name + ".output_dir";
  llm_debug_parameter_name_ = plugin_name + ".llm_debug";
  pre_launch_parameter_name_ = plugin_name + ".pre_launch";

  if (!lc_node_->has_parameter(launch_extra_args_parameter_name_)) {
    lc_node_->declare_parameter<std::vector<std::string>>(
      launch_extra_args_parameter_name_, std::vector<std::string>{});
  }
  if (!lc_node_->has_parameter(prompt_extra_args_parameter_name_)) {
    lc_node_->declare_parameter<std::vector<std::string>>(
      prompt_extra_args_parameter_name_, std::vector<std::string>{});
  }
  if (!lc_node_->has_parameter(output_dir_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(
      output_dir_parameter_name_, std::filesystem::temp_directory_path());
  }
  if (!lc_node_->has_parameter(llm_debug_parameter_name_)) {
    lc_node_->declare_parameter<bool>(llm_debug_parameter_name_, false);
  }
  if (!lc_node_->has_parameter(pre_launch_parameter_name_)) {
    lc_node_->declare_parameter<bool>(pre_launch_parameter_name_, true);
  }

  model_yaml_parameter_name_ = plugin_name + ".model_yaml";
  if (!lc_node_->has_parameter(model_yaml_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(
      model_yaml_parameter_name_, "~/TFG/src/llm/llama_ros/llama_bringup/models/Qwen2.5-3B.yaml");
  }

  llm_debug_ = lc_node_->get_parameter(llm_debug_parameter_name_).as_bool();
  pre_launch_ = lc_node_->get_parameter(pre_launch_parameter_name_).as_bool();
  launch_extras_ =
    lc_node_->get_parameter(launch_extra_args_parameter_name_).as_string_array();

  yaml_path_ = lc_node_->get_parameter(model_yaml_parameter_name_).as_string();
  {
    const char * home_dir = std::getenv("HOME");
    if (!yaml_path_.empty() && yaml_path_[0] == '~' && home_dir) {
      yaml_path_.replace(0, 1, std::string(home_dir));
    }
  }

  if (pre_launch_) {
    launch_llm_server();
  }
}

std::optional<std::filesystem::path>
LLAMASolver::create_folders(const std::string & node_namespace)
{
  auto output_dir = lc_node_->get_parameter(output_dir_parameter_name_).value_to_string();

  // Allow usage of the HOME directory with the `~` character, returning if there is an error.
  const char * home_dir = std::getenv("HOME");
  if (output_dir[0] == '~' && home_dir) {
    output_dir.replace(0, 1, home_dir);
  } else if (!home_dir) {
    RCLCPP_ERROR(
      lc_node_->get_logger(), "Invalid use of the ~ character in the path: %s", output_dir.c_str()
    );
    return std::nullopt;
  }

  // Create the necessary folders, returning if there is an error.
  auto output_path = std::filesystem::path(output_dir);
  if (node_namespace != "") {
    for (auto p : std::filesystem::path(node_namespace) ) {
      if (p != std::filesystem::current_path().root_directory()) {
        output_path /= p;
      }
    }
    try {
      std::filesystem::create_directories(output_path);
    } catch (std::filesystem::filesystem_error & err) {
      RCLCPP_ERROR(lc_node_->get_logger(), "Error writing directories: %s", err.what());
      return std::nullopt;
    }
  }
  return output_path;
}

std::optional<plansys2_solver_msgs::msg::Solver> LLAMASolver::solve(
  const std::string & domain, const std::string & problem,
  const std::string & observation,
  const std::string & action_file,
  const std::string & node_namespace,
  const rclcpp::Duration solve_timeout)
{
  cancel_requested_ = false;

  const auto t_solve_start = std::chrono::steady_clock::now();

  const auto output_dir_maybe = create_folders(node_namespace);
  if (!output_dir_maybe) {
    return {};
  }
  const auto & output_dir = output_dir_maybe.value();

  const auto domain_file_path = output_dir / std::filesystem::path("solver_domain.pddl");
  std::ofstream domain_out(domain_file_path);
  domain_out << domain;
  domain_out.close();

  const auto problem_file_path = output_dir / std::filesystem::path("solver_problem.pddl");
  std::ofstream problem_out(problem_file_path);
  problem_out << problem;
  problem_out.close();

  const auto observation_file_path = output_dir / std::filesystem::path("solver_observation.pddl");
  std::ofstream observation_out(observation_file_path);
  observation_out << observation;
  observation_out.close();


  const auto resolution_file_path = output_dir / std::filesystem::path("solver_resolution.txt");

  auto prompt_extras =
    lc_node_->get_parameter(prompt_extra_args_parameter_name_).as_string_array();

  auto logger = lc_node_->get_logger();

  RCLCPP_INFO(logger,
    "[llama-solver] Starting solve (timeout=%.0fs, summarize=%s, llm_debug=%s, "
    "prompt_debug=%s, pre_launch=%s)",
    solve_timeout.seconds(),
    summarize_mode_.c_str(),
    llm_debug_ ? "true" : "false",
    prompt_debug_ ? "true" : "false",
    pre_launch_ ? "true" : "false");

  // Fall back to on-the-fly launch if pre_launch_ said so but server is gone.
  if (!pre_launch_) {
    launch_llm_server();
  } else if (launch_pid_ == -1) {
    RCLCPP_WARN(logger,
      "[llama-solver] pre_launch=true but server is not running; "
      "falling back to on-the-fly launch");
    launch_llm_server();
  }

  bool limited = (summarize_mode_ != "full");
  std::string action_summary = summarizeActionLog(action_file, limited);

  std::string prompt_text = makePrompt(domain, problem, action_summary, observation);

  if (prompt_debug_) {
    RCLCPP_INFO(logger, "[llama-prompt] Sending prompt (%zu chars):\n%s",
      prompt_text.size(), prompt_text.c_str());
  }

  int prompt_pipe[2];
  pipe(prompt_pipe);

  pid_t prompt_pid = fork();
  if (prompt_pid == 0) {
    close(prompt_pipe[0]);
    dup2(prompt_pipe[1], STDOUT_FILENO);
    close(prompt_pipe[1]);

    std::vector<std::string> prompt_argv =
      {"ros2", "llama", "prompt", prompt_text, "-t", "0.0"};
    prompt_argv.insert(prompt_argv.end(), prompt_extras.begin(), prompt_extras.end());

    std::vector<char *> argv_ptrs;
    argv_ptrs.reserve(prompt_argv.size() + 1);
    for (auto & s : prompt_argv) {
      argv_ptrs.push_back(s.data());
    }
    argv_ptrs.push_back(nullptr);

    execvp("ros2", argv_ptrs.data());
    _exit(EXIT_FAILURE);
  }

  // Parent: poll the pipe so we can observe cancel_requested_ while the LLM
  // streams. On cancel, SIGTERM the prompt child so the read wakes and we bail.
  close(prompt_pipe[1]);
  std::ofstream resolution_out(resolution_file_path);
  std::string raw_response;
  char buf[512];
  bool cancelled = false;

  struct pollfd pfd;
  pfd.fd = prompt_pipe[0];
  pfd.events = POLLIN;

  while (true) {
    int poll_ret = poll(&pfd, 1, 100);  // 100 ms wake to check cancel_requested_
    if (poll_ret < 0) {
      if (errno == EINTR) {
        continue;
      }
      break;
    }
    if (poll_ret == 0) {
      if (cancel_requested_) {
        cancelled = true;
        kill(prompt_pid, SIGTERM);
        break;
      }
      continue;
    }
    ssize_t n = read(prompt_pipe[0], buf, sizeof(buf) - 1);
    if (n <= 0) {
      break;
    }
    buf[n] = '\0';
    resolution_out.write(buf, n);
    raw_response.append(buf, n);
    if (prompt_debug_) {
      RCLCPP_INFO(logger, "[llama-response] %s", buf);
    }
  }
  close(prompt_pipe[0]);
  resolution_out.close();
  waitpid(prompt_pid, NULL, 0);

  // Only tear down the LLM server in per-solve mode. In pre_launch mode the
  // server is kept alive and torn down by the destructor.
  if (!pre_launch_) {
    shutdown_llm_server();
  }

  const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - t_solve_start).count();

  if (cancelled) {
    RCLCPP_WARN(logger, "[llama-solver] Cancelled mid-solve after %ld ms", elapsed_ms);
    return std::nullopt;
  }

  RCLCPP_INFO(logger, "[llama-solver] solve() completed in %ld ms (%zu chars response)",
    elapsed_ms, raw_response.size());

  auto solution = parseResponse(raw_response);
  solution.time = static_cast<float>(elapsed_ms) / 1000.0f;
  return solution;
}

void LLAMASolver::initialize(const std::string & node_name)
{
  std::cout << "Initializing solver with node: " << node_name << std::endl;
}

void LLAMASolver::launch_llm_server()
{
  if (launch_pid_ != -1) {
    return;
  }

  auto logger = lc_node_->get_logger();

  int launch_pipe[2] = {-1, -1};
  if (llm_debug_) {
    pipe(launch_pipe);
  }

  pid_t pid = fork();
  if (pid == 0) {
    if (llm_debug_) {
      close(launch_pipe[0]);
      dup2(launch_pipe[1], STDOUT_FILENO);
      dup2(launch_pipe[1], STDERR_FILENO);
      close(launch_pipe[1]);
    } else {
      int fd = open("/dev/null", O_WRONLY);
      if (fd != -1) {
        dup2(fd, STDOUT_FILENO);
        dup2(fd, STDERR_FILENO);
        close(fd);
      }
    }

    std::vector<std::string> launch_argv = {"ros2", "llama", "launch", yaml_path_};
    launch_argv.insert(launch_argv.end(), launch_extras_.begin(), launch_extras_.end());

    std::vector<char *> argv_ptrs;
    argv_ptrs.reserve(launch_argv.size() + 1);
    for (auto & s : launch_argv) {
      argv_ptrs.push_back(s.data());
    }
    argv_ptrs.push_back(nullptr);

    execvp("ros2", argv_ptrs.data());
    _exit(EXIT_FAILURE);
  }

  launch_pid_ = pid;
  RCLCPP_INFO(logger, "[llama-launch] Starting model: %s [pid=%d | pre_launch=%s]",
    yaml_path_.c_str(), static_cast<int>(launch_pid_),
    pre_launch_ ? "true" : "false");

  if (llm_debug_) {
    close(launch_pipe[1]);
    launch_drain_ = std::thread([fd = launch_pipe[0], logger]() {
      char buf[512];
      std::string line_buf;
      ssize_t n;
      while ((n = read(fd, buf, sizeof(buf) - 1)) > 0) {
        buf[n] = '\0';
        line_buf += buf;
        size_t pos;
        while ((pos = line_buf.find('\n')) != std::string::npos) {
          std::string line = line_buf.substr(0, pos);
          if (!line.empty()) {
            RCLCPP_INFO(logger, "[llama-launch] %s", line.c_str());
          }
          line_buf.erase(0, pos + 1);
        }
      }
      if (!line_buf.empty()) {
        RCLCPP_INFO(logger, "[llama-launch] %s", line_buf.c_str());
      }
      close(fd);
    });
  }
}

void LLAMASolver::shutdown_llm_server()
{
  if (launch_pid_ == -1) {
    return;
  }

  // Avoid touching lc_node_ here: the destructor path can run while the owning
  // lifecycle node is already being torn down. The solve() path prints its own
  // shutdown context if needed.
  kill(launch_pid_, SIGINT);
  waitpid(launch_pid_, NULL, 0);
  launch_pid_ = -1;

  if (launch_drain_.joinable()) {
    launch_drain_.join();
  }
}
}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::LLAMASolver, plansys2::SolverBase);
