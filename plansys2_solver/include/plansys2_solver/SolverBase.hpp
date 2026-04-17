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

// Pin the ActionExecution enum values we compare against. This does not protect runtime
// behaviour (we use the named constants below), but it documents the assumption at compile
// time and will fire loudly if upstream plansys2_msgs ever renumbers these fields.
static_assert(plansys2_msgs::msg::ActionExecution::FINISH == 6,
  "summarizeActionLog assumes FINISH == 6 (log-file numeric stability)");
static_assert(plansys2_msgs::msg::ActionExecution::CANCEL == 7,
  "summarizeActionLog assumes CANCEL == 7 (log-file numeric stability)");

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
  // Map an ActionExecution::type value to its printable name. Anything outside the known
  // enum range is reported as UNKNOWN instead of indexing a lookup table blindly.
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

  // Parse the action-hub log (written by SolverNode::action_hub_callback, which uses a
  // "-----" separator and "Field: value" lines) and build a compact text summary the LLM
  // can consume. When `limited` is true (default) only FINISH/CANCEL entries are kept —
  // enough to know which action closed and how — dropping the chatty REQUEST/RESPONSE/
  // CONFIRM/REJECT/FEEDBACK traffic that bloats the prompt.
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

  // Build the standard prompt for the LLM with domain, problem, action log, and observation.
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

  // Parse raw LLM response into a populated Solver message.
  // Extracts JSON from wrapping text, populates classification + structured arrays.
  // On parse failure, returns msg with classification = MODIFY_PLAN.
  static inline plansys2_solver_msgs::msg::Solver parseResponse(const std::string & raw_response)
  {
    plansys2_solver_msgs::msg::Solver solution;
    solution.resolution = raw_response;

    try {
      std::string json_str = raw_response;
      auto json_start = raw_response.find('{');
      auto json_end = raw_response.rfind('}');
      if (json_start != std::string::npos && json_end != std::string::npos &&
        json_end > json_start)
      {
        json_str = raw_response.substr(json_start, json_end - json_start + 1);
      }

      auto j = nlohmann::json::parse(json_str);

      std::string classification_str = j.value("classification", "MODIFY_PLAN");
      if (classification_str == "CORRECT") {
        solution.classification = plansys2_solver_msgs::msg::Solver::CORRECT;
      } else if (classification_str == "MODIFY_DOMAIN") {
        solution.classification = plansys2_solver_msgs::msg::Solver::MODIFY_DOMAIN;
      } else if (classification_str == "UNSOLVABLE") {
        solution.classification = plansys2_solver_msgs::msg::Solver::UNSOLVABLE;
      } else {
        solution.classification = plansys2_solver_msgs::msg::Solver::MODIFY_PLAN;
      }

      if (j.contains("remove_predicates")) {
        for (const auto & p : j["remove_predicates"]) {
          solution.remove_predicates.push_back(p.get<std::string>());
        }
      }
      if (j.contains("add_predicates")) {
        for (const auto & p : j["add_predicates"]) {
          solution.add_predicates.push_back(p.get<std::string>());
        }
      }
      if (j.contains("add_instances")) {
        for (const auto & p : j["add_instances"]) {
          solution.add_instances.push_back(p.get<std::string>());
        }
      }
      if (j.contains("domain_changes")) {
        for (const auto & p : j["domain_changes"]) {
          solution.domain_changes.push_back(p.get<std::string>());
        }
      }
    } catch (const nlohmann::json::exception &) {
      solution.classification = plansys2_solver_msgs::msg::Solver::MODIFY_PLAN;
    }

    return solution;
  }

  // Lifecycle node pointer.
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;
  // Flag indicating if cancellation was requested.
  bool cancel_requested_ = false;

  // Generic solver parameters (read by base configure).
  // "limited" = only FINISH/CANCEL entries; "full" = all entries
  std::string summarize_mode_{"limited"};
  // Log the full prompt and response
  bool prompt_debug_{false};
};

}  // namespace plansys2

#endif  // PLANSYS2_SOLVER__SOLVERBASE_HPP_
