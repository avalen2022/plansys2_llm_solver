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


#include <fstream>
#include <fcntl.h>
#include <sys/wait.h>

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
  bool flag = lc_node_->get_parameter(llm_debug_parameter_).as_bool();


  RCLCPP_DEBUG(
    lc_node_->get_logger(),
    "[%s-llama] called with timeout %f seconds with args [%s] with output dir %s",
    lc_node_->get_name(), resolution_timeout.seconds(), args.c_str(), output_dir.c_str());

  pid_t pid = fork();

  if (pid == 0) {
    int fd = open("/dev/null", O_WRONLY);
    if (fd != -1 && flag == false) {
      dup2(fd, STDOUT_FILENO);
      dup2(fd, STDERR_FILENO);
      close(fd);
    }

    const char * home_dir = std::getenv("HOME");
    std::string yaml_path = std::string(home_dir) + "/TFG/src/llama_ros/llama_bringup/models/Phi-4.yaml";
    execlp("ros2", "ros2", "llama", "launch", yaml_path.c_str(), NULL);
    exit(EXIT_FAILURE);
  }

  // No sleep needed: ros2 llama prompt internally calls wait_for_server()
  // which blocks until llama_ros finishes loading the model

  // Build prompt: solver owns the JSON format requirement (needs it to parse classification),
  // controller owns the observation/question content
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
    "--- Action execution log ---\n" + action_file + "\n\n"
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

  // Fork + execlp to send prompt — passes prompt as argv, no shell interpretation
  pid_t prompt_pid = fork();
  if (prompt_pid == 0) {
    int fd = open(resolution_file_path.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd != -1) {
      dup2(fd, STDOUT_FILENO);
      close(fd);
    }
    execlp("ros2", "ros2", "llama", "prompt", prompt_text.c_str(), "-t", "0.0", NULL);
    _exit(EXIT_FAILURE);
  }
  waitpid(prompt_pid, NULL, 0);

  kill(pid, SIGINT);
  waitpid(pid, NULL, 0);

  // Read LLM response from file
  std::string raw_response;
  std::ifstream file(resolution_file_path);
  if (file) {
    std::stringstream buffer;
    buffer << file.rdbuf();
    raw_response = buffer.str();
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

    RCLCPP_DEBUG(lc_node_->get_logger(),
      "[%s-llama] Classification: %s, modifier_flag: %s",
      lc_node_->get_name(), classification.c_str(),
      solution.modifier_flag ? "true" : "false");
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
