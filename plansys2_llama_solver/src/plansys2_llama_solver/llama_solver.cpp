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
#include <sstream>
#include <thread>
#include <sys/wait.h>
#include <unistd.h>

#include <nlohmann/json.hpp>

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
  lc_node_ = lc_node;

  arguments_parameter_name_ = plugin_name + ".arguments";
  output_dir_parameter_name_ = plugin_name + ".output_dir";
  llm_debug_parameter_ = plugin_name + ".llm_debug";
  summarize_mode_parameter_ = plugin_name + ".summarize_mode";

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
  prompt_debug_parameter_ = plugin_name + ".prompt_debug";
  if (!lc_node_->has_parameter(prompt_debug_parameter_)) {
    lc_node_->declare_parameter<bool>(prompt_debug_parameter_, false);
  }
  if (!lc_node_->has_parameter(summarize_mode_parameter_)) {
    // "limited" = only FINISH/CANCEL entries (compact, fits small context windows)
    // "full" = all entries in compact one-line format (needs larger context window)
    lc_node_->declare_parameter<std::string>(summarize_mode_parameter_, "limited");
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

std::string LLAMASolver::summarize_action_log(
  const std::string & raw_log, bool limited)
{
  // Parse the raw action hub log and produce a compact one-line-per-entry summary.
  // Two modes controlled by limited:
  //   true  (default) — only FINISH/CANCEL entries (compact, fits small context windows)
  //   false ("full")  — all entries in compact format (needs larger context window)
  // Type mapping: 1=REQUEST, 2=RESPONSE, 3=CONFIRM, 4=REJECT, 5=FEEDBACK, 6=FINISH, 7=CANCEL
  static const char * type_names[] = {
    "", "REQUEST", "RESPONSE", "CONFIRM", "REJECT", "FEEDBACK", "FINISH", "CANCEL"};

  std::string summary;
  std::istringstream stream(raw_log);
  std::string line;

  std::string current_action;
  std::string current_args;
  std::string current_status;
  int current_type = 0;
  bool current_success = false;
  bool in_entry = false;
  bool reading_args = false;

  while (std::getline(stream, line)) {
    if (line.find("----") == 0) {
      if (in_entry && !current_action.empty()) {
        bool keep = !limited ||
          (current_type == 6 || current_type == 7);

        if (keep) {
          const char * tname = (current_type >= 1 && current_type <= 7)
            ? type_names[current_type] : "UNKNOWN";
          std::string result = current_success ? "SUCCESS" : "FAILED";

          if (limited) {
            // Compact: no type tag needed (all are FINISH/CANCEL)
            summary += current_action + "(" + current_args + "): " + result;
          } else {
            summary += std::string("[") + tname + "] " +
              current_action + "(" + current_args + "): " + result;
          }
          if (!current_status.empty()) {
            summary += ". " + current_status;
          }
          summary += "\n";
        }
      }
      in_entry = true;
      current_action.clear();
      current_args.clear();
      current_status.clear();
      current_type = 0;
      current_success = false;
      reading_args = false;
      continue;
    }

    if (line.find("Action: ") == 0) {
      current_action = line.substr(8);
      reading_args = false;
    } else if (line.find("Type: ") == 0) {
      current_type = std::atoi(line.substr(6).c_str());
    } else if (line.find("Arguments:") == 0) {
      reading_args = true;
    } else if (reading_args && line.find("  - ") == 0) {
      if (!current_args.empty()) current_args += " ";
      current_args += line.substr(4);
    } else if (line.find("Success: ") == 0) {
      current_success = (line.substr(9) == "true");
      reading_args = false;
    } else if (line.find("Status: ") == 0) {
      current_status = line.substr(8);
      reading_args = false;
    } else {
      reading_args = false;
    }
  }

  if (summary.empty()) {
    return "(no action log available)";
  }
  return summary;
}

std::optional<plansys2_msgs::msg::Solver> LLAMASolver::solve(
  const std::string & domain, const std::string & problem,
  const std::string & question,
  const std::string & action_file,
  const std::string & node_namespace,
  const rclcpp::Duration resolution_timeout)
{
  cancel_requested_ = false;
  plansys2_msgs::msg::Solver solution;

  const auto output_dir_maybe = create_folders(node_namespace);
  if (!output_dir_maybe) {
    return {};
  }
  const auto & output_dir = output_dir_maybe.value();
  RCLCPP_DEBUG(
    lc_node_->get_logger(), "Writing solver results on %s and domain/problem copies done.", output_dir.string().c_str());

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
  bool prompt_debug = lc_node_->get_parameter(prompt_debug_parameter_).as_bool();
  auto logger = lc_node_->get_logger();

  RCLCPP_INFO(logger,
    "[llama-solver] Starting solve (timeout=%.0fs, llm_debug=%s, prompt_debug=%s)",
    resolution_timeout.seconds(),
    llm_debug ? "true" : "false",
    prompt_debug ? "true" : "false");

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
  std::string mode = lc_node_->get_parameter(summarize_mode_parameter_).as_string();
  bool limited = (mode != "full");
  std::string action_summary = summarize_action_log(action_file, limited);

  RCLCPP_INFO(logger,
    "[llama-solver] Action log summarized: %zu chars (raw: %zu chars)",
    action_summary.size(), action_file.size());

  // Build prompt
  std::string prompt_text =
    "You are a PDDL state update assistant. Given a PDDL domain, problem state, "
    "action execution log, and an observation, determine what state changes are needed.\n\n"
    "RULES:\n"
    "- Only change predicates directly affected by the observation\n"
    "- Do NOT change predicates for objects not mentioned\n"
    "- If an object moved from A to B: remove (object_at obj A), add (object_at obj B)\n"
    "- If no changes are needed, classify as CORRECT\n\n"
    "--- Domain ---\n" + domain + "\n\n"
    "--- Problem ---\n" + problem + "\n\n"
    "--- Action execution log ---\n" + action_summary + "\n\n"
    "--- Observation ---\n" + question + "\n\n"
    "Reply ONLY with a JSON object in this exact format:\n"
    "{\n"
    "  \"classification\": \"MODIFY_PLAN\" or \"CORRECT\" or \"UNSOLVABLE\",\n"
    "  \"reasoning\": \"brief explanation\",\n"
    "  \"remove_predicates\": [\"(predicate1)\", \"(predicate2)\"],\n"
    "  \"add_predicates\": [\"(predicate1)\", \"(predicate2)\"],\n"
    "  \"add_instances\": [],\n"
    "  \"domain_changes\": []\n"
    "}";

  if (prompt_debug) {
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
    if (prompt_debug) {
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

  solution.resolution = raw_response;

  // Parse classification from JSON to set modifier_flag correctly
  try {
    // Extract JSON from response (handles LLM wrapping with extra text)
    std::string json_str = raw_response;
    auto json_start = raw_response.find('{');
    auto json_end = raw_response.rfind('}');
    if (json_start != std::string::npos && json_end != std::string::npos && json_end > json_start) {
      json_str = raw_response.substr(json_start, json_end - json_start + 1);
    }

    auto j = nlohmann::json::parse(json_str);
    std::string classification = j.value("classification", "MODIFY_PLAN");
    solution.modifier_flag = (classification != "CORRECT");

    std::string reasoning = j.value("reasoning", "");
    RCLCPP_INFO(lc_node_->get_logger(),
      "[llama-solver] Classification: %s | modifier_flag: %s | Reasoning: %s",
      classification.c_str(),
      solution.modifier_flag ? "true" : "false",
      reasoning.c_str());
  } catch (const nlohmann::json::exception & e) {
    // JSON parse failed — fallback to assuming changes are needed
    RCLCPP_WARN(lc_node_->get_logger(),
      "[%s-llama] Failed to parse JSON classification, defaulting to modifier_flag=true: %s",
      lc_node_->get_name(), e.what());
    solution.modifier_flag = true;
  }

  return solution;
}

void LLAMASolver::initialize(const std::string & node_name)
{
  std::cout << "Inicializando solver con nodo: " << node_name << std::endl;
}
}  // namespace plansys2

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(plansys2::LLAMASolver, plansys2::SolverBase);
