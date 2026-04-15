// Copyright 2019 Intelligent Robotics Lab
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


#include <chrono>
#include <fstream>
#include <fcntl.h>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>

#include "plansys2_llama_solver/llama_solver.hpp"
#include "rclcpp/logging.hpp"

namespace plansys2
{

LLAMASolver::LLAMASolver()
{
}

void LLAMASolver::configure(
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node,
  const std::string & plugin_name)
{
  // Generic params (summarize_mode, prompt_debug) read by base class.
  SolverBase::configure(lc_node, plugin_name);

  // Plugin-specific params (namespaced under plugin name).
  arguments_parameter_name_ = plugin_name + ".arguments";
  output_dir_parameter_name_ = plugin_name + ".output_dir";
  llm_debug_parameter_ = plugin_name + ".llm_debug";

  if (!lc_node_->has_parameter(arguments_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(arguments_parameter_name_, "");
  }
  if (!lc_node_->has_parameter(output_dir_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(
      output_dir_parameter_name_, std::filesystem::temp_directory_path());
  }
  if (!lc_node_->has_parameter(llm_debug_parameter_)) {
    lc_node_->declare_parameter<bool>(llm_debug_parameter_, false);
  }

  model_yaml_parameter_name_ = plugin_name + ".model_yaml";
  if (!lc_node_->has_parameter(model_yaml_parameter_name_)) {
    lc_node_->declare_parameter<std::string>(
      model_yaml_parameter_name_, "~/TFG/src/llm/llama_ros/llama_bringup/models/Qwen2.5-3B.yaml");
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
  const std::string & question,
  const std::string & action_file,
  const std::string & node_namespace,
  const rclcpp::Duration resolution_timeout)
{
  cancel_requested_ = false;

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

  const auto question_file_path = output_dir / std::filesystem::path("solver_question.pddl");
  std::ofstream question_out(question_file_path);
  question_out << question;
  question_out.close();


  const auto resolution_file_path = output_dir / std::filesystem::path("solver_resolution.txt");
  const auto solver_file_path = output_dir / std::filesystem::path("solver");

  const auto args = lc_node_->get_parameter(arguments_parameter_name_).value_to_string();
  bool llm_debug = lc_node_->get_parameter(llm_debug_parameter_).as_bool();
  auto logger = lc_node_->get_logger();

  RCLCPP_INFO(logger,
    "[llama-solver] Starting solve (timeout=%.0fs, summarize=%s, llm_debug=%s, prompt_debug=%s)",
    resolution_timeout.seconds(),
    summarize_mode_.c_str(),
    llm_debug ? "true" : "false",
    prompt_debug_ ? "true" : "false");

  // --- Fork: launch LLM server ---
  int launch_pipe[2] = {-1, -1};
  if (llm_debug) {
    pipe(launch_pipe);
  }

  pid_t pid = fork();
  if (pid == 0) {
    if (llm_debug) {
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

    const char * home_dir = std::getenv("HOME");
    std::string yaml_path = lc_node_->get_parameter(model_yaml_parameter_name_).as_string();
    if (!yaml_path.empty() && yaml_path[0] == '~' && home_dir) {
      yaml_path.replace(0, 1, std::string(home_dir));
    }
    RCLCPP_INFO(logger, "[llama-launch] Starting model: %s", yaml_path.c_str());
    execlp("ros2", "ros2", "llama", "launch", yaml_path.c_str(), NULL);
    exit(EXIT_FAILURE);
  }

  // Drain launch stdout in background thread (real-time logging)
  std::thread launch_drain;
  if (llm_debug) {
    close(launch_pipe[1]);
    launch_drain = std::thread([fd = launch_pipe[0], logger]() {
      char buf[512];
      std::string line_buf;
      ssize_t n;
      while ((n = read(fd, buf, sizeof(buf) - 1)) > 0) {
        buf[n] = '\0';
        line_buf += buf;
        // Log complete lines as they arrive
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

  // Summarize the raw action hub into a compact log for the LLM.
  bool limited = (summarize_mode_ != "full");
  std::string action_summary = summarizeActionLog(action_file, limited);

  // Build prompt
  std::string prompt_text = makePrompt(domain, problem, action_summary, question);

  if (prompt_debug_) {
    RCLCPP_INFO(logger, "[llama-prompt] Sending prompt (%zu chars):\n%s",
      prompt_text.size(), prompt_text.c_str());
  }

  // --- Fork: send prompt via pipe (tee to file + optional logging) ---
  int prompt_pipe[2];
  pipe(prompt_pipe);

  auto t_start = std::chrono::steady_clock::now();

  pid_t prompt_pid = fork();
  if (prompt_pid == 0) {
    close(prompt_pipe[0]);
    dup2(prompt_pipe[1], STDOUT_FILENO);
    close(prompt_pipe[1]);
    execlp("ros2", "ros2", "llama", "prompt", prompt_text.c_str(), "-t", "0.0", NULL);
    _exit(EXIT_FAILURE);
  }

  // Parent: read pipe, write to resolution file, optionally log in real-time
  close(prompt_pipe[1]);
  std::ofstream resolution_out(resolution_file_path);
  std::string raw_response;
  char buf[512];
  ssize_t n;

  while ((n = read(prompt_pipe[0], buf, sizeof(buf) - 1)) > 0) {
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

  auto t_end = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();

  RCLCPP_INFO(logger, "[llama-solver] LLM inference completed in %ld ms (%zu chars response)",
    elapsed_ms, raw_response.size());

  // Shutdown LLM server
  kill(pid, SIGINT);
  waitpid(pid, NULL, 0);
  if (launch_drain.joinable()) {
    launch_drain.join();
  }

  auto solution = parseResponse(raw_response);
  solution.time = static_cast<float>(elapsed_ms) / 1000.0f;
  return solution;
}

void LLAMASolver::initialize(const std::string & node_name)
{
  std::cout << "Initializing solver with node: " << node_name << std::endl;
}
}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::LLAMASolver, plansys2::SolverBase);
