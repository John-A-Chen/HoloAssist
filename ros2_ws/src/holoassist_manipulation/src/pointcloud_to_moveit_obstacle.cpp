#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class PointCloudToMoveItObstacle : public rclcpp::Node
{
public:
  PointCloudToMoveItObstacle()
  : Node("pointcloud_to_moveit_obstacle"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    declare_parameter<std::string>("pointcloud_topic", "/holo_assist_depth_tracker/pointcloud");
    declare_parameter<std::string>("planning_frame", "base_link");
    declare_parameter<std::string>("collision_object_id", "depth_pointcloud_obstacle");
    declare_parameter<double>("max_range_m", 2.0);
    declare_parameter<double>("padding_m", 0.05);
    declare_parameter<double>("min_size_m", 0.08);
    declare_parameter<int>("min_points", 150);
    declare_parameter<double>("update_rate_hz", 5.0);
    declare_parameter<int>("sample_step", 1);
    declare_parameter<int>("log_every_n_updates", 25);

    pointcloud_topic_ = get_parameter("pointcloud_topic").as_string();
    planning_frame_ = get_parameter("planning_frame").as_string();
    collision_object_id_ = get_parameter("collision_object_id").as_string();
    max_range_m_ = get_parameter("max_range_m").as_double();
    padding_m_ = get_parameter("padding_m").as_double();
    min_size_m_ = get_parameter("min_size_m").as_double();
    min_points_ = get_parameter("min_points").as_int();
    update_rate_hz_ = get_parameter("update_rate_hz").as_double();
    sample_step_ = get_parameter("sample_step").as_int();
    log_every_n_updates_ = get_parameter("log_every_n_updates").as_int();

    if (max_range_m_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "max_range_m must be > 0. Using 2.0.");
      max_range_m_ = 2.0;
    }
    if (padding_m_ < 0.0) {
      RCLCPP_WARN(get_logger(), "padding_m must be >= 0. Using 0.0.");
      padding_m_ = 0.0;
    }
    if (min_size_m_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "min_size_m must be > 0. Using 0.08.");
      min_size_m_ = 0.08;
    }
    if (min_points_ < 1) {
      RCLCPP_WARN(get_logger(), "min_points must be >= 1. Using 1.");
      min_points_ = 1;
    }
    if (update_rate_hz_ <= 0.0) {
      RCLCPP_WARN(get_logger(), "update_rate_hz must be > 0. Using 5.0.");
      update_rate_hz_ = 5.0;
    }
    if (sample_step_ < 1) {
      RCLCPP_WARN(get_logger(), "sample_step must be >= 1. Using 1.");
      sample_step_ = 1;
    }
    if (log_every_n_updates_ < 1) {
      log_every_n_updates_ = 1;
    }

    min_update_period_ = rclcpp::Duration::from_seconds(1.0 / update_rate_hz_);

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&PointCloudToMoveItObstacle::onPointCloud, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "pointcloud_to_moveit_obstacle started. topic=%s planning_frame=%s max_range=%.2f m",
      pointcloud_topic_.c_str(),
      planning_frame_.c_str(),
      max_range_m_);
  }

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    const auto now = this->get_clock()->now();
    if (last_update_time_.nanoseconds() > 0 && (now - last_update_time_) < min_update_period_) {
      return;
    }
    last_update_time_ = now;

    if (msg->data.empty()) {
      removeCollisionObject("empty pointcloud");
      return;
    }

    float min_x = std::numeric_limits<float>::infinity();
    float min_y = std::numeric_limits<float>::infinity();
    float min_z = std::numeric_limits<float>::infinity();
    float max_x = -std::numeric_limits<float>::infinity();
    float max_y = -std::numeric_limits<float>::infinity();
    float max_z = -std::numeric_limits<float>::infinity();
    std::size_t valid_count = 0;

    try {
      sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
      sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
      sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

      std::size_t idx = 0;
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++idx) {
        if (sample_step_ > 1 && (idx % static_cast<std::size_t>(sample_step_) != 0)) {
          continue;
        }

        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {
          continue;
        }

        const float range = std::sqrt(x * x + y * y + z * z);
        if (range <= 0.0f || range > static_cast<float>(max_range_m_)) {
          continue;
        }

        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        min_z = std::min(min_z, z);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
        max_z = std::max(max_z, z);
        ++valid_count;
      }
    } catch (const std::runtime_error & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "PointCloud missing x/y/z fields: %s",
        ex.what());
      return;
    }

    if (static_cast<int>(valid_count) < min_points_) {
      removeCollisionObject("not enough in-range points");
      return;
    }

    const double sx = std::max(
      static_cast<double>(max_x - min_x) + 2.0 * padding_m_,
      min_size_m_);
    const double sy = std::max(
      static_cast<double>(max_y - min_y) + 2.0 * padding_m_,
      min_size_m_);
    const double sz = std::max(
      static_cast<double>(max_z - min_z) + 2.0 * padding_m_,
      min_size_m_);

    const double cx = 0.5 * static_cast<double>(min_x + max_x);
    const double cy = 0.5 * static_cast<double>(min_y + max_y);
    const double cz = 0.5 * static_cast<double>(min_z + max_z);

    geometry_msgs::msg::Pose pose;
    std::string frame_id = msg->header.frame_id;

    if (!planning_frame_.empty() && planning_frame_ != msg->header.frame_id) {
      try {
        const auto tf_msg = tf_buffer_.lookupTransform(
          planning_frame_,
          msg->header.frame_id,
          tf2::TimePointZero);

        tf2::Transform tf_planning_from_cloud;
        tf2::fromMsg(tf_msg.transform, tf_planning_from_cloud);

        const tf2::Vector3 center_cloud(cx, cy, cz);
        const tf2::Vector3 center_planning = tf_planning_from_cloud * center_cloud;
        pose.position.x = center_planning.x();
        pose.position.y = center_planning.y();
        pose.position.z = center_planning.z();
        pose.orientation = tf_msg.transform.rotation;
        frame_id = planning_frame_;
      } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "TF lookup failed (%s -> %s): %s",
          msg->header.frame_id.c_str(),
          planning_frame_.c_str(),
          ex.what());
        return;
      }
    } else {
      pose.position.x = cx;
      pose.position.y = cy;
      pose.position.z = cz;
      pose.orientation.w = 1.0;
    }

    moveit_msgs::msg::CollisionObject co;
    co.id = collision_object_id_;
    co.header.frame_id = frame_id;
    co.header.stamp = msg->header.stamp;
    co.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {sx, sy, sz};
    co.primitives.push_back(box);
    co.primitive_poses.push_back(pose);

    const bool ok = planning_scene_interface_.applyCollisionObjects({co});
    if (!ok) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to apply collision object. Is move_group running?");
      return;
    }

    object_active_ = true;
    ++update_count_;
    if (update_count_ % static_cast<std::size_t>(log_every_n_updates_) == 0) {
      RCLCPP_INFO(
        get_logger(),
        "Updated MoveIt obstacle '%s' size=[%.3f, %.3f, %.3f]m frame=%s points=%zu",
        collision_object_id_.c_str(),
        sx,
        sy,
        sz,
        frame_id.c_str(),
        valid_count);
    }
  }

  void removeCollisionObject(const std::string & reason)
  {
    if (!object_active_) {
      return;
    }

    moveit_msgs::msg::CollisionObject co;
    co.id = collision_object_id_;
    co.header.frame_id = planning_frame_;
    co.operation = moveit_msgs::msg::CollisionObject::REMOVE;

    const bool ok = planning_scene_interface_.applyCollisionObjects({co});
    if (ok) {
      object_active_ = false;
      RCLCPP_INFO(get_logger(), "Removed MoveIt obstacle '%s' (%s)", collision_object_id_.c_str(), reason.c_str());
    } else {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "Failed to remove collision object '%s'",
        collision_object_id_.c_str());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  std::string pointcloud_topic_;
  std::string planning_frame_;
  std::string collision_object_id_;
  double max_range_m_{2.0};
  double padding_m_{0.05};
  double min_size_m_{0.08};
  int min_points_{150};
  double update_rate_hz_{5.0};
  int sample_step_{1};
  int log_every_n_updates_{25};

  rclcpp::Duration min_update_period_{0, 0};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  bool object_active_{false};
  std::size_t update_count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudToMoveItObstacle>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
