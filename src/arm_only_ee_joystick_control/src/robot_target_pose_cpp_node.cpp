#include <chrono>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


class RobotTargetPoseCppNode : public rclcpp::Node
{
public:
  RobotTargetPoseCppNode()
  : Node("robot_target_pose_cpp_node")
  {
    input_point_topic_ = declare_parameter<std::string>(
      "input_point_topic", "/vlm_robot_tf/grasp_point");

    target_pose_topic_ = declare_parameter<std::string>(
      "target_pose_topic", "/vlm_moveit/grasp_target_pose");

    pregrasp_pose_topic_ = declare_parameter<std::string>(
      "pregrasp_pose_topic", "/vlm_moveit/pregrasp_pose");

    marker_topic_ = declare_parameter<std::string>(
      "marker_topic", "/vlm_moveit/target_markers");

    target_frame_ = declare_parameter<std::string>("target_frame", "ARM_BASE_LINK");

    target_offset_x_ = declare_parameter<double>("target_offset_x", 0.0);
    target_offset_y_ = declare_parameter<double>("target_offset_y", 0.0);
    target_offset_z_ = declare_parameter<double>("target_offset_z", 0.08);

    pregrasp_extra_z_ = declare_parameter<double>("pregrasp_extra_z", 0.10);

    qx_ = declare_parameter<double>("target_qx", 0.0);
    qy_ = declare_parameter<double>("target_qy", 0.0);
    qz_ = declare_parameter<double>("target_qz", 0.0);
    qw_ = declare_parameter<double>("target_qw", 1.0);

    target_marker_scale_ = declare_parameter<double>("target_marker_scale", 0.025);
    pregrasp_marker_scale_ = declare_parameter<double>("pregrasp_marker_scale", 0.020);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      input_point_topic_,
      qos,
      std::bind(&RobotTargetPoseCppNode::pointCallback, this, std::placeholders::_1));

    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      target_pose_topic_, qos);

    pregrasp_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      pregrasp_pose_topic_, qos);

    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_, qos);

    republish_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&RobotTargetPoseCppNode::republish, this));

    RCLCPP_INFO(get_logger(), "robot_target_pose_cpp_node started");
    RCLCPP_INFO(get_logger(), "input_point_topic=%s", input_point_topic_.c_str());
    RCLCPP_INFO(get_logger(), "target_pose_topic=%s", target_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "pregrasp_pose_topic=%s", pregrasp_pose_topic_.c_str());
    RCLCPP_INFO(get_logger(), "target_offset_z=%.3f", target_offset_z_);
    RCLCPP_INFO(get_logger(), "pregrasp_extra_z=%.3f", pregrasp_extra_z_);
  }

private:
  geometry_msgs::msg::PoseStamped makePose(
    const geometry_msgs::msg::PointStamped & point,
    double extra_z)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = get_clock()->now();
    pose.header.frame_id = target_frame_;

    pose.pose.position.x = point.point.x + target_offset_x_;
    pose.pose.position.y = point.point.y + target_offset_y_;
    pose.pose.position.z = point.point.z + target_offset_z_ + extra_z;

    pose.pose.orientation.x = qx_;
    pose.pose.orientation.y = qy_;
    pose.pose.orientation.z = qz_;
    pose.pose.orientation.w = qw_;

    return pose;
  }

  visualization_msgs::msg::Marker makeSphereMarker(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & ns,
    int id,
    double scale,
    double r,
    double g,
    double b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = pose.header;
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = pose.pose;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.95;

    return marker;
  }

  visualization_msgs::msg::Marker makeArrowMarker(
    const geometry_msgs::msg::PoseStamped & pregrasp,
    const geometry_msgs::msg::PoseStamped & target)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = target.header;
    marker.ns = "pregrasp_to_target";
    marker.id = 10;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    geometry_msgs::msg::Point p0;
    p0.x = pregrasp.pose.position.x;
    p0.y = pregrasp.pose.position.y;
    p0.z = pregrasp.pose.position.z;

    geometry_msgs::msg::Point p1;
    p1.x = target.pose.position.x;
    p1.y = target.pose.position.y;
    p1.z = target.pose.position.z;

    marker.points.push_back(p0);
    marker.points.push_back(p1);

    marker.scale.x = 0.010;
    marker.scale.y = 0.025;
    marker.scale.z = 0.025;

    marker.color.r = 0.2;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.95;

    return marker;
  }

  visualization_msgs::msg::Marker makeTextMarker(
    const geometry_msgs::msg::PoseStamped & target)
  {
    visualization_msgs::msg::Marker marker;
    marker.header = target.header;
    marker.ns = "target_text";
    marker.id = 20;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose = target.pose;
    marker.pose.position.z += 0.04;

    marker.scale.z = 0.035;

    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 0.95;

    char buf[256];
    std::snprintf(
      buf,
      sizeof(buf),
      "target pose\\n%.3f %.3f %.3f",
      target.pose.position.x,
      target.pose.position.y,
      target.pose.position.z);

    marker.text = buf;

    return marker;
  }

  visualization_msgs::msg::MarkerArray makeMarkers(
    const geometry_msgs::msg::PoseStamped & target,
    const geometry_msgs::msg::PoseStamped & pregrasp)
  {
    visualization_msgs::msg::MarkerArray arr;

    arr.markers.push_back(
      makeSphereMarker(
        target,
        "grasp_target_pose",
        0,
        target_marker_scale_,
        1.0,
        0.0,
        0.0));

    arr.markers.push_back(
      makeSphereMarker(
        pregrasp,
        "pregrasp_pose",
        1,
        pregrasp_marker_scale_,
        0.0,
        0.7,
        1.0));

    arr.markers.push_back(makeArrowMarker(pregrasp, target));
    arr.markers.push_back(makeTextMarker(target));

    return arr;
  }

  void publishAll()
  {
    if (!has_result_) {
      return;
    }

    target_pose_.header.stamp = get_clock()->now();
    pregrasp_pose_.header.stamp = target_pose_.header.stamp;

    auto markers = makeMarkers(target_pose_, pregrasp_pose_);

    target_pose_pub_->publish(target_pose_);
    pregrasp_pose_pub_->publish(pregrasp_pose_);
    marker_pub_->publish(markers);
  }

  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != target_frame_) {
      RCLCPP_WARN(
        get_logger(),
        "input point frame_id is %s, expected %s",
        msg->header.frame_id.c_str(),
        target_frame_.c_str());
    }

    std::lock_guard<std::mutex> lock(mutex_);

    target_pose_ = makePose(*msg, 0.0);
    pregrasp_pose_ = makePose(*msg, pregrasp_extra_z_);

    has_result_ = true;

    publishAll();

    RCLCPP_INFO(
      get_logger(),
      "target pose: x=%.3f, y=%.3f, z=%.3f",
      target_pose_.pose.position.x,
      target_pose_.pose.position.y,
      target_pose_.pose.position.z);

    RCLCPP_INFO(
      get_logger(),
      "pregrasp pose: x=%.3f, y=%.3f, z=%.3f",
      pregrasp_pose_.pose.position.x,
      pregrasp_pose_.pose.position.y,
      pregrasp_pose_.pose.position.z);
  }

  void republish()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publishAll();
  }

private:
  std::mutex mutex_;

  std::string input_point_topic_;
  std::string target_pose_topic_;
  std::string pregrasp_pose_topic_;
  std::string marker_topic_;
  std::string target_frame_;

  double target_offset_x_;
  double target_offset_y_;
  double target_offset_z_;
  double pregrasp_extra_z_;

  double qx_;
  double qy_;
  double qz_;
  double qw_;

  double target_marker_scale_;
  double pregrasp_marker_scale_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr point_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pregrasp_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr republish_timer_;

  bool has_result_{false};
  geometry_msgs::msg::PoseStamped target_pose_;
  geometry_msgs::msg::PoseStamped pregrasp_pose_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotTargetPoseCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
