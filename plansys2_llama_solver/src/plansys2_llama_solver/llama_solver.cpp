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

  std::string prompt_text =
    "\"-- Contents ---\n"
    "Domain:\n" + domain + "\n\n"
    "Problem:\n" + problem + "\n\n"
    "Action_hub:\n" + action_file + "\n\n"
    "Question:\n" + question + "\"";

  std::string prompt_cmd = "ros2 llama prompt " + prompt_text + " -t 0.0 > " + resolution_file_path.c_str();
  int ret = std::system(prompt_cmd.c_str());

  kill(pid, SIGINT);
  waitpid(pid, NULL, 0);

  solution.modifier_flag = true;

  std::ifstream file(resolution_file_path);
  if (file) {
    std::stringstream buffer;
    buffer << file.rdbuf();
    solution.resolution = buffer.str();
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
