#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <chrono>
#include <string>
#include <vector>

using namespace std::chrono_literals;

struct FrameConfig {
  std::string frame;
  std::string parent;
  int type;
  std::array<double, 3> scale;
  std::array<double, 4> color; // r, g, b, a
};

class TFMarkerBridge : public rclcpp::Node {
public:
  TFMarkerBridge() : Node("tf_marker_bridge") {
    // Frames to visualize — add more entries here as needed
    frames_ = {
      {"unity_cube", "base_link", visualization_msgs::msg::Marker::CUBE,
       {0.1, 0.1, 0.1}, {1.0, 0.2, 0.8, 1.0}},
    };

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
        "/unity_markers", 10);

    timer_ = this->create_wall_timer(100ms, [this]() { publishMarkers(); });

    RCLCPP_INFO(this->get_logger(), "TF Marker Bridge started");
  }

private:
  void publishMarkers() {
    for (size_t i = 0; i < frames_.size(); ++i) {
      const auto &cfg = frames_[i];

      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_buffer_->lookupTransform(cfg.parent, cfg.frame,
                                        tf2::TimePointZero);
      } catch (const tf2::TransformException &) {
        continue;
      }

      visualization_msgs::msg::Marker m;
      m.header.frame_id = cfg.parent;
      m.header.stamp = this->get_clock()->now();
      m.ns = "unity_objects";
      m.id = static_cast<int>(i);
      m.type = cfg.type;
      m.action = visualization_msgs::msg::Marker::ADD;

      m.pose.position.x = t.transform.translation.x;
      m.pose.position.y = t.transform.translation.y;
      m.pose.position.z = t.transform.translation.z;
      m.pose.orientation = t.transform.rotation;

      m.scale.x = cfg.scale[0];
      m.scale.y = cfg.scale[1];
      m.scale.z = cfg.scale[2];

      m.color.r = cfg.color[0];
      m.color.g = cfg.color[1];
      m.color.b = cfg.color[2];
      m.color.a = cfg.color[3];

      marker_pub_->publish(m);
    }
  }

  std::vector<FrameConfig> frames_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TFMarkerBridge>());
  rclcpp::shutdown();
  return 0;
}
