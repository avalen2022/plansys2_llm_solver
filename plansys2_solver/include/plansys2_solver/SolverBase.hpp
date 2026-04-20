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

#ifndef PLANSYS2_SOLVER__SOLVERBASE_HPP_
#define PLANSYS2_SOLVER__SOLVERBASE_HPP_

#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "plansys2_msgs/msg/action_execution.hpp"
#include "plansys2_solver_msgs/msg/solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

namespace plansys2
{

class SolverBase
{
public:
  using Ptr = std::shared_ptr<plansys2::SolverBase>;
  SolverBase() = default;
  virtual ~SolverBase() = default;

  virtual void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, const std::string & plugin_name)
  {
    (void)plugin_name;
    lc_node_ = lc_node;

    if (!lc_node_->has_parameter("summarize_mode")) {
      lc_node_->declare_parameter<std::string>("summarize_mode", "limited");
    }
    lc_node_->get_parameter("summarize_mode", summarize_mode_);

    if (!lc_node_->has_parameter("prompt_debug")) {
      lc_node_->declare_parameter<bool>("prompt_debug", false);
    }
    lc_node_->get_parameter("prompt_debug", prompt_debug_);
  }

  virtual void initialize(const std::string & node_name) = 0;

  virtual std::optional<plansys2_solver_msgs::msg::Solver> solve(
    const std::string & domain, const std::string & problem,
    const std::string & observation,
    const std::string & action_file,
    const std::string & node_namespace = "",
    const rclcpp::Duration solve_timeout = 120s) = 0;

  virtual void cancel() {cancel_requested_ = true;}

protected:
  // UNKNOWN for out-of-range types — no array indexing.
  static inline const char * actionTypeName(int16_t type)
  {
    using plansys2_msgs::msg::ActionExecution;
    switch (type) {
      case ActionExecution::REQUEST:  return "REQUEST";
      case ActionExecution::RESPONSE: return "RESPONSE";
      case ActionExecution::CONFIRM:  return "CONFIRM";
      case ActionExecution::REJECT:   return "REJECT";
      case ActionExecution::FEEDBACK: return "FEEDBACK";
      case ActionExecution::FINISH:   return "FINISH";
      case ActionExecution::CANCEL:   return "CANCEL";
      default:                        return "UNKNOWN";
    }
  }

  // Parse action_hub entries separated by a row of dashes; limited=true keeps
  // only FINISH/CANCEL entries, dropping the chatty intermediate traffic.
  static inline std::string summarizeActionLog(const std::string & raw_log, bool limited = true)
  {
    using plansys2_msgs::msg::ActionExecution;

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
            (current_type == ActionExecution::FINISH ||
             current_type == ActionExecution::CANCEL);

          if (keep) {
            const char * tname = actionTypeName(current_type);
            std::string result = current_success ? "SUCCESS" : "FAILED";

            if (limited) {
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

  static inline std::string makePrompt(
    const std::string & domain,
    const std::string & problem,
    const std::string & action_summary,
    const std::string & observation)
  {
    return
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
      "--- Observation ---\n" + observation + "\n\n"
      "Reply ONLY with a JSON object in this exact format:\n"
      "{\n"
      "  \"classification\": \"MODIFY_PLAN\" or \"CORRECT\" or \"UNSOLVABLE\",\n"
      "  \"reasoning\": \"brief explanation\",\n"
      "  \"remove_predicates\": [\"(predicate1)\", \"(predicate2)\"],\n"
      "  \"add_predicates\": [\"(predicate1)\", \"(predicate2)\"],\n"
      "  \"add_instances\": [],\n"
      "  \"domain_changes\": []\n"
      "}";
  }

  // Any failure yields classification=ERROR so callers can distinguish
  // "LLM didn't answer" from "LLM wants a replan".
  static inline plansys2_solver_msgs::msg::Solver parseResponse(const std::string & raw_response)
  {
    plansys2_solver_msgs::msg::Solver solution;
    solution.resolution = raw_response;
    solution.classification = plansys2_solver_msgs::msg::Solver::ERROR;

    if (raw_response.empty()) {
      return solution;
    }

    // Outermost {...}; LLMs sometimes wrap JSON in commentary.
    const auto json_start = raw_response.find('{');
    const auto json_end = raw_response.rfind('}');
    if (json_start == std::string::npos || json_end == std::string::npos ||
      json_end <= json_start)
    {
      return solution;
    }
    const std::string json_str = raw_response.substr(json_start, json_end - json_start + 1);

    try {
      auto j = nlohmann::json::parse(json_str);
      if (!j.is_object() || !j.contains("classification") ||
        !j["classification"].is_string())
      {
        return solution;
      }

      const std::string classification_str = j["classification"].get<std::string>();
      if (classification_str == "CORRECT") {
        solution.classification = plansys2_solver_msgs::msg::Solver::CORRECT;
      } else if (classification_str == "MODIFY_PLAN") {
        solution.classification = plansys2_solver_msgs::msg::Solver::MODIFY_PLAN;
      } else if (classification_str == "MODIFY_DOMAIN") {
        solution.classification = plansys2_solver_msgs::msg::Solver::MODIFY_DOMAIN;
      } else if (classification_str == "UNSOLVABLE") {
        solution.classification = plansys2_solver_msgs::msg::Solver::UNSOLVABLE;
      } else {
        return solution;
      }

      // Optional arrays: absent or wrong type is not ERROR; classification already valid.
      auto appendStrings = [](const nlohmann::json & arr, std::vector<std::string> & out) {
        if (!arr.is_array()) {
          return;
        }
        for (const auto & p : arr) {
          if (p.is_string()) {
            out.push_back(p.get<std::string>());
          }
        }
      };
      if (j.contains("remove_predicates")) {
        appendStrings(j["remove_predicates"], solution.remove_predicates);
      }
      if (j.contains("add_predicates")) {
        appendStrings(j["add_predicates"], solution.add_predicates);
      }
      if (j.contains("add_instances")) {
        appendStrings(j["add_instances"], solution.add_instances);
      }
      if (j.contains("domain_changes")) {
        appendStrings(j["domain_changes"], solution.domain_changes);
      }
    } catch (const nlohmann::json::exception &) {
      solution.classification = plansys2_solver_msgs::msg::Solver::ERROR;
    }

    return solution;
  }

  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;
  bool cancel_requested_ = false;

  // "limited" = only FINISH/CANCEL entries; "full" = all entries.
  std::string summarize_mode_ = "limited";
  bool prompt_debug_ = false;
};

}  // namespace plansys2

#endif  // PLANSYS2_SOLVER__SOLVERBASE_HPP_
