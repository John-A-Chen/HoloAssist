#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <moveit/move_group_interface/move_group_interface.h>

class ClickedPointToMoveIt : public rclcpp::Node
{
public:
  ClickedPointToMoveIt() : Node("clicked_point_to_moveit")
  {
    declare_parameter<std::string>("move_group_name", "ur_manipulator");
    declare_parameter<std::string>("planning_frame", "base_link");
    declare_parameter<std::string>("tcp_frame", "tool0");
    declare_parameter<double>("tcp_yaw_rad", 0.0);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>("clicked_goal_marker", 10);

    sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/clicked_point", 10,
      std::bind(&ClickedPointToMoveIt::onClickedPoint, this, std::placeholders::_1));

    // MoveGroupInterface needs a separate node handle (recommended pattern)
    moveit_node_ = rclcpp::Node::make_shared("moveit_helper_node");
    move_group_name_ = get_parameter("move_group_name").as_string();
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(moveit_node_, move_group_name_);

    RCLCPP_INFO(get_logger(), "Ready. Click points in RViz (Publish Point tool) on /clicked_point");
  }

  rclcpp::Node::SharedPtr getMoveItNode() const
  {
    return moveit_node_;
  }

private:
  void onClickedPoint(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    auto planning_frame = get_parameter("planning_frame").as_string();
    auto yaw = get_parameter("tcp_yaw_rad").as_double();
    (void)yaw;
    auto tcp_frame = get_parameter("tcp_frame").as_string();

    geometry_msgs::msg::PoseStamped target;
    target.header.stamp = now();
    target.header.frame_id = planning_frame;

    // Position from clicked point (assumes point is already in planning_frame)
    target.pose.position.x = msg->point.x;
    target.pose.position.y = msg->point.y;
    target.pose.position.z = msg->point.z;

    // Simple fixed orientation: yaw about Z, flat tool
    // Quaternion from roll=pi, pitch=0, yaw=yaw is often needed depending on UR tool definition
    // Keep it simple here: identity (you will tune later)
    target.pose.orientation.w = 1.0;

    publishMarker(target);

    if (!tcp_frame.empty()) {
      move_group_->setEndEffectorLink(tcp_frame);
    }
    move_group_->setPoseReferenceFrame(planning_frame);
    move_group_->setPoseTarget(target);
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    auto ok = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!ok)
    {
      RCLCPP_WARN(get_logger(), "Planning failed");
      return;
    }

    RCLCPP_INFO(get_logger(), "Executing plan");
    move_group_->execute(plan);
  }

  void publishMarker(const geometry_msgs::msg::PoseStamped& pose)
  {
    visualization_msgs::msg::Marker m;
    m.header = pose.header;
    m.ns = "clicked_goal";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::SPHERE;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose = pose.pose;
    m.scale.x = 0.03; m.scale.y = 0.03; m.scale.z = 0.03;
    m.color.a = 1.0; m.color.r = 0.1; m.color.g = 0.9; m.color.b = 0.1;
    marker_pub_->publish(m);
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Node::SharedPtr moveit_node_;
  std::string move_group_name_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClickedPointToMoveIt>();

  // Need executor that spins both nodes (main node + moveit helper node)
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(node->getMoveItNode());
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
