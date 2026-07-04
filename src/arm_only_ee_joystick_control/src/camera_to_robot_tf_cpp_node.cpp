#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


class CameraToRobotTfCppNode : public rclcpp::Node
{
public:
  CameraToRobotTfCppNode()
  : Node("camera_to_robot_tf_cpp_node")
  {
    target_frame_ = declare_parameter<std::string>("target_frame", "ARM_BASE_LINK");

    input_cloud_topic_ = declare_parameter<std::string>(
      "input_cloud_topic", "/vlm_camera_scan/object_cloud");
    input_grasp_topic_ = declare_parameter<std::string>(
      "input_grasp_point_topic", "/vlm_camera_scan/grasp_point");
    input_push_topic_ = declare_parameter<std::string>(
      "input_push_point_topic", "/vlm_camera_scan/push_point");

    output_cloud_topic_ = declare_parameter<std::string>(
      "output_cloud_topic", "/vlm_robot_tf/object_cloud");
    output_grasp_topic_ = declare_parameter<std::string>(
      "output_grasp_point_topic", "/vlm_robot_tf/grasp_point");
    output_push_topic_ = declare_parameter<std::string>(
      "output_push_point_topic", "/vlm_robot_tf/push_point");
    output_marker_topic_ = declare_parameter<std::string>(
      "output_marker_topic", "/vlm_robot_tf/object_markers");

    grasp_marker_scale_ = declare_parameter<double>("grasp_marker_scale", 0.015);
    push_marker_scale_ = declare_parameter<double>("push_marker_scale", 0.012);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_cloud_topic_,
      qos,
      std::bind(&CameraToRobotTfCppNode::cloudCallback, this, std::placeholders::_1));

    grasp_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      input_grasp_topic_,
      qos,
      std::bind(&CameraToRobotTfCppNode::graspCallback, this, std::placeholders::_1));

    push_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      input_push_topic_,
      qos,
      std::bind(&CameraToRobotTfCppNode::pushCallback, this, std::placeholders::_1));

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_cloud_topic_, qos);
    grasp_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(output_grasp_topic_, qos);
    push_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(output_push_topic_, qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(output_marker_topic_, qos);

    marker_timer_ = create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&CameraToRobotTfCppNode::publishMarkers, this));

    RCLCPP_INFO(get_logger(), "camera_to_robot_tf_cpp_node started");
    RCLCPP_INFO(get_logger(), "target_frame=%s", target_frame_.c_str());
  }

private:
  bool lookupTransformToTarget(
    const std::string & source_frame,
    geometry_msgs::msg::TransformStamped & out_tf)
  {
    try {
      out_tf = tf_buffer_->lookupTransform(
        target_frame_,
        source_frame,
        tf2::TimePointZero);
      return true;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "TF not ready: %s <- %s: %s",
        target_frame_.c_str(),
        source_frame.c_str(),
        e.what());
      return false;
    }
  }

  tf2::Transform toTf2Transform(const geometry_msgs::msg::TransformStamped & tf_msg)
  {
    const auto & t = tf_msg.transform.translation;
    const auto & q_msg = tf_msg.transform.rotation;

    tf2::Quaternion q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    q.normalize();

    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(t.x, t.y, t.z));
    tf.setRotation(q);
    return tf;
  }

  geometry_msgs::msg::PointStamped transformPoint(
    const geometry_msgs::msg::PointStamped & in,
    const geometry_msgs::msg::TransformStamped & tf_msg)
  {
    const tf2::Transform tf = toTf2Transform(tf_msg);

    const tf2::Vector3 p_in(in.point.x, in.point.y, in.point.z);
    const tf2::Vector3 p_out = tf * p_in;

    geometry_msgs::msg::PointStamped out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = target_frame_;
    out.point.x = p_out.x();
    out.point.y = p_out.y();
    out.point.z = p_out.z();
    return out;
  }

  sensor_msgs::msg::PointCloud2 transformCloud(
    const sensor_msgs::msg::PointCloud2 & in,
    const geometry_msgs::msg::TransformStamped & tf_msg)
  {
    const tf2::Transform tf = toTf2Transform(tf_msg);

    std::vector<tf2::Vector3> points;
    points.reserve(static_cast<size_t>(in.width) * static_cast<size_t>(in.height));

    sensor_msgs::PointCloud2ConstIterator<float> ix(in, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iy(in, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iz(in, "z");

    for (; ix != ix.end(); ++ix, ++iy, ++iz) {
      if (!std::isfinite(*ix) || !std::isfinite(*iy) || !std::isfinite(*iz)) {
        continue;
      }

      const tf2::Vector3 p_in(*ix, *iy, *iz);
      points.push_back(tf * p_in);
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = get_clock()->now();
    out.header.frame_id = target_frame_;
    out.height = 1;
    out.width = static_cast<uint32_t>(points.size());
    out.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> ox(out, "x");
    sensor_msgs::PointCloud2Iterator<float> oy(out, "y");
    sensor_msgs::PointCloud2Iterator<float> oz(out, "z");

    for (const auto & p : points) {
      *ox = static_cast<float>(p.x());
      *oy = static_cast<float>(p.y());
      *oz = static_cast<float>(p.z());
      ++ox;
      ++oy;
      ++oz;
    }

    return out;
  }

  visualization_msgs::msg::Marker makeMarker(
    const geometry_msgs::msg::PointStamped & point,
    const std::string & ns,
    int id,
    int type,
    double scale,
    double r,
    double g,
    double b)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = get_clock()->now();
    marker.header.frame_id = target_frame_;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = point.point.x;
    marker.pose.position.y = point.point.y;
    marker.pose.position.z = point.point.z;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.95;

    return marker;
  }

  void publishMarkers()
  {
    visualization_msgs::msg::MarkerArray arr;

    {
      std::lock_guard<std::mutex> lock(point_mutex_);

      if (has_grasp_) {
        arr.markers.push_back(
          makeMarker(
            last_grasp_,
            "robot_tf_grasp_point",
            0,
            visualization_msgs::msg::Marker::SPHERE,
            grasp_marker_scale_,
            0.0,
            1.0,
            0.0));
      }

      if (has_push_) {
        arr.markers.push_back(
          makeMarker(
            last_push_,
            "robot_tf_push_point",
            1,
            visualization_msgs::msg::Marker::CUBE,
            push_marker_scale_,
            1.0,
            0.5,
            0.0));
      }
    }

    if (!arr.markers.empty()) {
      marker_pub_->publish(arr);
    }
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "input cloud has empty frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    if (!lookupTransformToTarget(msg->header.frame_id, tf_msg)) {
      return;
    }

    const auto out = transformCloud(*msg, tf_msg);
    cloud_pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "published robot-frame cloud: frame=%s, points=%u",
      out.header.frame_id.c_str(),
      out.width * out.height);
  }

  void graspCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "input grasp point has empty frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    if (!lookupTransformToTarget(msg->header.frame_id, tf_msg)) {
      return;
    }

    const auto out = transformPoint(*msg, tf_msg);
    grasp_pub_->publish(out);

    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      last_grasp_ = out;
      has_grasp_ = true;
    }

    publishMarkers();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "grasp robot frame: x=%.3f, y=%.3f, z=%.3f",
      out.point.x,
      out.point.y,
      out.point.z);
  }

  void pushCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (msg->header.frame_id.empty()) {
      RCLCPP_WARN(get_logger(), "input push point has empty frame_id");
      return;
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    if (!lookupTransformToTarget(msg->header.frame_id, tf_msg)) {
      return;
    }

    const auto out = transformPoint(*msg, tf_msg);
    push_pub_->publish(out);

    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      last_push_ = out;
      has_push_ = true;
    }

    publishMarkers();

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "push robot frame: x=%.3f, y=%.3f, z=%.3f",
      out.point.x,
      out.point.y,
      out.point.z);
  }

private:
  std::string target_frame_;

  std::string input_cloud_topic_;
  std::string input_grasp_topic_;
  std::string input_push_topic_;

  std::string output_cloud_topic_;
  std::string output_grasp_topic_;
  std::string output_push_topic_;
  std::string output_marker_topic_;

  double grasp_marker_scale_;
  double push_marker_scale_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr grasp_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr push_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr grasp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr push_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  rclcpp::TimerBase::SharedPtr marker_timer_;

  std::mutex point_mutex_;
  bool has_grasp_{false};
  bool has_push_{false};
  geometry_msgs::msg::PointStamped last_grasp_;
  geometry_msgs::msg::PointStamped last_push_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraToRobotTfCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
