#ifndef PLANSYS2_SOLVER__SOLVERBASE_HPP_
#define PLANSYS2_SOLVER__SOLVERBASE_HPP_

#include <memory>
#include <optional>
#include <sstream>
#include <string>

#include <nlohmann/json.hpp>

#include "plansys2_msgs/msg/solver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

namespace plansys2
{

class SolverBase
{
public:
  using Ptr = std::shared_ptr<plansys2::SolverBase>;
  SolverBase() {}
  virtual ~SolverBase() {}

  virtual void configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node, const std::string & plugin_name) = 0;

  virtual void initialize(const std::string & node_name) = 0;

  virtual std::optional<plansys2_msgs::msg::Solver> solve(
    const std::string & domain, const std::string & problem,
    const std::string & question,
    const std::string & action_file,
    const std::string & node_namespace = "",
    const rclcpp::Duration resolution_timeout = 120s) = 0;

  virtual void cancel() {cancel_requested_ = true;}

protected:
  // Summarize the raw action hub log into a compact format for the LLM.
  // limited=true: only FINISH/CANCEL entries; limited=false: all entries
  static inline std::string summarizeActionLog(const std::string & raw_log, bool limited = true)
  {
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
    const std::string & question)
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
  }

  // Parse raw LLM response into a populated Solver message.
  // Extracts JSON from wrapping text, populates classification + structured arrays.
  // On parse failure, returns msg with classification = MODIFY_PLAN.
  static inline plansys2_msgs::msg::Solver parseResponse(const std::string & raw_response)
  {
    plansys2_msgs::msg::Solver solution;
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
        solution.classification = plansys2_msgs::msg::Solver::CORRECT;
      } else if (classification_str == "MODIFY_DOMAIN") {
        solution.classification = plansys2_msgs::msg::Solver::MODIFY_DOMAIN;
      } else if (classification_str == "UNSOLVABLE") {
        solution.classification = plansys2_msgs::msg::Solver::UNSOLVABLE;
      } else {
        solution.classification = plansys2_msgs::msg::Solver::MODIFY_PLAN;
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
      solution.classification = plansys2_msgs::msg::Solver::MODIFY_PLAN;
    }

    return solution;
  }

  // Lifecycle node pointer.
  rclcpp_lifecycle::LifecycleNode::SharedPtr lc_node_;
  // Flag indicating if cancellation was requested.
  bool cancel_requested_;
};

}  // namespace plansys2

#endif  // PLANSYS2_SOLVER__SOLVERBASE_HPP_
