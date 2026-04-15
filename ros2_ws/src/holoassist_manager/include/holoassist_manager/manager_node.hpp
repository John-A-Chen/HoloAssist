// Copyright 2026 HoloAssist
// SPDX-License-Identifier: MIT

#ifndef HOLOASSIST_MANAGER__MANAGER_NODE_HPP_
#define HOLOASSIST_MANAGER__MANAGER_NODE_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

namespace holoassist_manager
{

// ── Operating modes ──────────────────────────────────────────────────────────

enum class OperatingMode : uint8_t
{
  MANUAL = 0,  // Full human-in-the-loop teleoperation
  HYBRID = 1   // Combined manual + autonomous behaviour
};

inline std::string mode_to_string(OperatingMode m)
{
  switch (m) {
    case OperatingMode::MANUAL: return "MANUAL";
    case OperatingMode::HYBRID: return "HYBRID";
    default: return "UNKNOWN";
  }
}

inline OperatingMode string_to_mode(const std::string & s)
{
  if (s == "HYBRID") {return OperatingMode::HYBRID;}
  return OperatingMode::MANUAL;  // safe default
}

// ── Subsystem descriptor ─────────────────────────────────────────────────────

struct SubsystemInfo
{
  std::string name;
  std::string heartbeat_topic;
  double timeout_sec{5.0};

  using Clock = std::chrono::steady_clock;
  Clock::time_point last_seen{};

  bool alive() const
  {
    if (last_seen == Clock::time_point{}) {return false;}
    auto elapsed = std::chrono::duration<double>(Clock::now() - last_seen).count();
    return elapsed < timeout_sec;
  }
};

// ── Manager node ─────────────────────────────────────────────────────────────

class ManagerNode : public rclcpp::Node
{
public:
  explicit ManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // Mode helpers
  void set_mode(OperatingMode new_mode);
  void publish_mode();

  // Heartbeat callback
  void on_heartbeat(SubsystemInfo & subsystem, const std_msgs::msg::String::SharedPtr msg);

  // Diagnostics timer
  void publish_diagnostics();

  // Service callbacks
  void srv_set_manual(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr resp);
  void srv_set_hybrid(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr resp);
  void srv_get_mode(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr resp);
  void srv_system_status(
    const std_srvs::srv::Trigger::Request::SharedPtr req,
    std_srvs::srv::Trigger::Response::SharedPtr resp);

  // State
  OperatingMode mode_;
  std::vector<SubsystemInfo> subsystems_;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;

  // Heartbeat subscriptions (prevent GC)
  std::vector<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr> heartbeat_subs_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_manual_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr set_hybrid_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr get_mode_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr system_status_srv_;

  // Timer
  rclcpp::TimerBase::SharedPtr diag_timer_;
};

}  // namespace holoassist_manager

#endif  // HOLOASSIST_MANAGER__MANAGER_NODE_HPP_
