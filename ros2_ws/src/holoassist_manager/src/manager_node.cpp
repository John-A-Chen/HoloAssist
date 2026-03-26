// Copyright 2026 HoloAssist
// SPDX-License-Identifier: MIT

#include "holoassist_manager/manager_node.hpp"

#include <algorithm>
#include <functional>
#include <sstream>

namespace holoassist_manager
{

// ── Default subsystems to supervise ──────────────────────────────────────────

static const std::vector<std::pair<std::string, std::string>> kDefaultSubsystems = {
  {"xr_interface",  "/xr_interface/heartbeat"},
  {"perception",    "/perception/heartbeat"},
  {"planning",      "/planning/heartbeat"},
  {"control",       "/control/heartbeat"},
  {"rviz",          "/rviz/heartbeat"},
  {"unity_bridge",  "/unity_bridge/heartbeat"},
  {"robot_bringup", "/robot_bringup/heartbeat"},
};

// ── Constructor ──────────────────────────────────────────────────────────────

ManagerNode::ManagerNode(const rclcpp::NodeOptions & options)
: Node("holoassist_manager", options)
{
  // Parameters
  this->declare_parameter<std::string>("initial_mode", "MANUAL");
  this->declare_parameter<double>("status_publish_rate", 1.0);
  this->declare_parameter<double>("heartbeat_timeout_sec", 5.0);

  const auto initial_mode_str = this->get_parameter("initial_mode").as_string();
  const auto status_rate = this->get_parameter("status_publish_rate").as_double();
  const auto default_timeout = this->get_parameter("heartbeat_timeout_sec").as_double();

  mode_ = string_to_mode(initial_mode_str);

  // Build subsystem registry
  for (const auto & [name, topic] : kDefaultSubsystems) {
    SubsystemInfo info;
    info.name = name;
    info.heartbeat_topic = topic;
    info.timeout_sec = default_timeout;
    subsystems_.push_back(info);
  }

  // Publishers – mode uses a latching-style QoS
  rclcpp::QoS latching_qos(1);
  latching_qos.reliable().transient_local();

  mode_pub_ = this->create_publisher<std_msgs::msg::String>("~/mode", latching_qos);
  diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "~/diagnostics", 10);

  // Services
  using std::placeholders::_1;
  using std::placeholders::_2;

  set_manual_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/set_manual", std::bind(&ManagerNode::srv_set_manual, this, _1, _2));
  set_hybrid_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/set_hybrid", std::bind(&ManagerNode::srv_set_hybrid, this, _1, _2));
  get_mode_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/get_mode", std::bind(&ManagerNode::srv_get_mode, this, _1, _2));
  system_status_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/system_status", std::bind(&ManagerNode::srv_system_status, this, _1, _2));

  // Heartbeat subscriptions
  for (auto & sub : subsystems_) {
    heartbeat_subs_.push_back(
      this->create_subscription<std_msgs::msg::String>(
        sub.heartbeat_topic, 10,
        [this, &sub](const std_msgs::msg::String::SharedPtr msg) {
          this->on_heartbeat(sub, msg);
        }));
  }

  // Diagnostics timer
  const double period_sec = 1.0 / std::max(status_rate, 0.01);
  diag_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(period_sec),
    std::bind(&ManagerNode::publish_diagnostics, this));

  // Publish the initial mode immediately
  publish_mode();

  RCLCPP_INFO(this->get_logger(), "HoloAssist Manager started – mode=%s",
    mode_to_string(mode_).c_str());
}

// ── Mode helpers ─────────────────────────────────────────────────────────────

void ManagerNode::set_mode(OperatingMode new_mode)
{
  const auto old = mode_;
  mode_ = new_mode;
  publish_mode();
  if (old != new_mode) {
    RCLCPP_INFO(this->get_logger(), "Mode changed: %s -> %s",
      mode_to_string(old).c_str(), mode_to_string(new_mode).c_str());
  }
}

void ManagerNode::publish_mode()
{
  std_msgs::msg::String msg;
  msg.data = mode_to_string(mode_);
  mode_pub_->publish(msg);
}

// ── Heartbeat callback ──────────────────────────────────────────────────────

void ManagerNode::on_heartbeat(
  SubsystemInfo & subsystem,
  const std_msgs::msg::String::SharedPtr /*msg*/)
{
  subsystem.last_seen = SubsystemInfo::Clock::now();
}

// ── Diagnostics timer ────────────────────────────────────────────────────────

void ManagerNode::publish_diagnostics()
{
  diagnostic_msgs::msg::DiagnosticArray diag;
  diag.header.stamp = this->now().operator builtin_interfaces::msg::Time();

  for (const auto & sub : subsystems_) {
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "holoassist/" + sub.name;
    status.hardware_id = sub.name;

    if (sub.alive()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "alive";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "no heartbeat";
    }

    diagnostic_msgs::msg::KeyValue kv_topic;
    kv_topic.key = "heartbeat_topic";
    kv_topic.value = sub.heartbeat_topic;
    status.values.push_back(kv_topic);

    diagnostic_msgs::msg::KeyValue kv_last;
    kv_last.key = "last_seen";
    kv_last.value = std::to_string(
      std::chrono::duration<double>(sub.last_seen.time_since_epoch()).count());
    status.values.push_back(kv_last);

    diag.status.push_back(status);
  }

  // Overall mode entry
  diagnostic_msgs::msg::DiagnosticStatus mode_status;
  mode_status.name = "holoassist/mode";
  mode_status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  mode_status.message = mode_to_string(mode_);
  diag.status.push_back(mode_status);

  diag_pub_->publish(diag);
}

// ── Service callbacks ────────────────────────────────────────────────────────

void ManagerNode::srv_set_manual(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  set_mode(OperatingMode::MANUAL);
  resp->success = true;
  resp->message = "Mode set to MANUAL";
}

void ManagerNode::srv_set_hybrid(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  set_mode(OperatingMode::HYBRID);
  resp->success = true;
  resp->message = "Mode set to HYBRID";
}

void ManagerNode::srv_get_mode(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  resp->success = true;
  resp->message = mode_to_string(mode_);
}

void ManagerNode::srv_system_status(
  const std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  std::vector<std::string> alive;
  std::vector<std::string> down;
  for (const auto & s : subsystems_) {
    if (s.alive()) {
      alive.push_back(s.name);
    } else {
      down.push_back(s.name);
    }
  }

  auto join = [](const std::vector<std::string> & v) {
      std::ostringstream oss;
      for (size_t i = 0; i < v.size(); ++i) {
        if (i > 0) {oss << ", ";}
        oss << v[i];
      }
      return oss.str();
    };

  resp->success = down.empty();
  resp->message = "mode=" + mode_to_string(mode_) +
    "  alive=[" + join(alive) + "]" +
    "  down=[" + join(down) + "]";
}

}  // namespace holoassist_manager

// ── Entry point ──────────────────────────────────────────────────────────────

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<holoassist_manager::ManagerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
