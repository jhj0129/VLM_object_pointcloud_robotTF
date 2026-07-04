#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"


class CameraOnlyScanOnceCppNode : public rclcpp::Node
{
public:
  CameraOnlyScanOnceCppNode()
  : Node("camera_only_scan_once_cpp_node")
  {
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/test_rs/color/image_raw");
    depth_topic_ = declare_parameter<std::string>("depth_topic", "/test_rs/depth/image_raw");
    info_topic_ = declare_parameter<std::string>("info_topic", "/test_rs/color/camera_info");

    frame_id_ = declare_parameter<std::string>("frame_id", "camera_color_optical_frame");
    target_color_ = declare_parameter<std::string>("target_color", "blue");

    capture_frames_ = declare_parameter<int>("capture_frames", 20);
    depth_scale_ = declare_parameter<double>("depth_scale", 0.001);
    raw_depth_min_ = declare_parameter<double>("raw_depth_min", 0.10);
    raw_depth_max_ = declare_parameter<double>("raw_depth_max", 1.50);

    min_mask_pixels_ = declare_parameter<int>("min_mask_pixels", 80);
    min_valid_depth_pixels_ = declare_parameter<int>("min_valid_depth_pixels", 50);
    max_cloud_points_ = declare_parameter<int>("max_cloud_points", 5000);

    grasp_marker_scale_ = declare_parameter<double>("grasp_marker_scale", 0.015);
    push_marker_scale_ = declare_parameter<double>("push_marker_scale", 0.012);

    publish_period_ = declare_parameter<double>("publish_period", 1.0);

    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/vlm_camera_scan/object_cloud");
    grasp_topic_ = declare_parameter<std::string>("grasp_point_topic", "/vlm_camera_scan/grasp_point");
    push_topic_ = declare_parameter<std::string>("push_point_topic", "/vlm_camera_scan/push_point");
    marker_topic_ = declare_parameter<std::string>("marker_topic", "/vlm_camera_scan/object_markers");
    annotated_topic_ = declare_parameter<std::string>("annotated_topic", "/vlm_camera_scan/annotated_image");
    mask_topic_ = declare_parameter<std::string>("mask_topic", "/vlm_camera_scan/mask_image");

    auto sensor_qos = rclcpp::SensorDataQoS();

    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
      rgb_topic_,
      sensor_qos,
      std::bind(&CameraOnlyScanOnceCppNode::rgbCallback, this, std::placeholders::_1));

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic_,
      sensor_qos,
      std::bind(&CameraOnlyScanOnceCppNode::depthCallback, this, std::placeholders::_1));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic_,
      sensor_qos,
      std::bind(&CameraOnlyScanOnceCppNode::infoCallback, this, std::placeholders::_1));

    auto result_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_, result_qos);
    grasp_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(grasp_topic_, result_qos);
    push_pub_ = create_publisher<geometry_msgs::msg::PointStamped>(push_topic_, result_qos);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, result_qos);
    annotated_pub_ = create_publisher<sensor_msgs::msg::Image>(annotated_topic_, result_qos);
    mask_pub_ = create_publisher<sensor_msgs::msg::Image>(mask_topic_, result_qos);

    collect_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&CameraOnlyScanOnceCppNode::tryCollectFrame, this));

    republish_timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(publish_period_ * 1000.0)),
      std::bind(&CameraOnlyScanOnceCppNode::republishResult, this));

    RCLCPP_INFO(get_logger(), "camera_only_scan_once_cpp_node started");
    RCLCPP_INFO(get_logger(), "rgb_topic=%s", rgb_topic_.c_str());
    RCLCPP_INFO(get_logger(), "depth_topic=%s", depth_topic_.c_str());
    RCLCPP_INFO(get_logger(), "info_topic=%s", info_topic_.c_str());
    RCLCPP_INFO(get_logger(), "target_color=%s", target_color_.c_str());
    RCLCPP_INFO(get_logger(), "frame_id=%s", frame_id_.c_str());
    RCLCPP_INFO(get_logger(), "capture_frames=%d", capture_frames_);
  }

private:
  void rgbCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_rgb_msg_ = msg;
  }

  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_depth_msg_ = msg;
  }

  void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_info_msg_ = msg;
  }

  static int64_t stampToNs(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<int64_t>(stamp.sec) * 1000000000LL + static_cast<int64_t>(stamp.nanosec);
  }

  cv::Mat imageToRgbMat(const sensor_msgs::msg::Image & msg)
  {
    if (msg.encoding != "rgb8" && msg.encoding != "bgr8") {
      throw std::runtime_error("Unsupported RGB encoding: " + msg.encoding);
    }

    cv::Mat input(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_8UC3,
      const_cast<unsigned char *>(msg.data.data()),
      static_cast<size_t>(msg.step));

    cv::Mat output;

    if (msg.encoding == "rgb8") {
      output = input.clone();
    } else {
      cv::cvtColor(input, output, cv::COLOR_BGR2RGB);
    }

    return output;
  }

  cv::Mat imageToDepthMat(const sensor_msgs::msg::Image & msg)
  {
    if (msg.encoding != "16UC1" && msg.encoding != "mono16") {
      throw std::runtime_error("Unsupported depth encoding: " + msg.encoding);
    }

    cv::Mat input(
      static_cast<int>(msg.height),
      static_cast<int>(msg.width),
      CV_16UC1,
      const_cast<unsigned char *>(msg.data.data()),
      static_cast<size_t>(msg.step));

    return input.clone();
  }

  void tryCollectFrame()
  {
    if (scan_done_) {
      return;
    }

    sensor_msgs::msg::Image::SharedPtr rgb_msg;
    sensor_msgs::msg::Image::SharedPtr depth_msg;
    sensor_msgs::msg::CameraInfo::SharedPtr info_msg;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      if (!latest_rgb_msg_ || !latest_depth_msg_ || !latest_info_msg_) {
        return;
      }

      rgb_msg = latest_rgb_msg_;
      depth_msg = latest_depth_msg_;
      info_msg = latest_info_msg_;
    }

    const int64_t depth_stamp_ns = stampToNs(depth_msg->header.stamp);

    if (depth_stamp_ns == last_collected_depth_stamp_ns_) {
      return;
    }

    last_collected_depth_stamp_ns_ = depth_stamp_ns;

    try {
      cv::Mat rgb = imageToRgbMat(*rgb_msg);
      cv::Mat depth = imageToDepthMat(*depth_msg);

      rgb_frames_.push_back(rgb);
      depth_frames_.push_back(depth);

      if (static_cast<int>(depth_frames_.size()) % 5 == 0) {
        RCLCPP_INFO(
          get_logger(),
          "captured frames: %zu/%d",
          depth_frames_.size(),
          capture_frames_);
      }

      if (static_cast<int>(depth_frames_.size()) >= capture_frames_) {
        computeOnce(*info_msg);
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN(get_logger(), "frame conversion failed: %s", e.what());
    }
  }

  cv::Mat makeColorMask(const cv::Mat & rgb)
  {
    cv::Mat hsv;
    cv::cvtColor(rgb, hsv, cv::COLOR_RGB2HSV);

    cv::Mat mask;

    if (target_color_ == "blue") {
      cv::inRange(hsv, cv::Scalar(90, 70, 40), cv::Scalar(135, 255, 255), mask);
    } else if (target_color_ == "green") {
      cv::inRange(hsv, cv::Scalar(40, 60, 40), cv::Scalar(85, 255, 255), mask);
    } else if (target_color_ == "red") {
      cv::Mat mask1;
      cv::Mat mask2;
      cv::inRange(hsv, cv::Scalar(0, 70, 40), cv::Scalar(10, 255, 255), mask1);
      cv::inRange(hsv, cv::Scalar(170, 70, 40), cv::Scalar(180, 255, 255), mask2);
      mask = mask1 | mask2;
    } else {
      throw std::runtime_error("Unsupported target_color: " + target_color_);
    }

    cv::Mat kernel = cv::Mat::ones(3, 3, CV_8U);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

    return mask;
  }

  bool extractLargestComponent(const cv::Mat & mask, cv::Mat & object_mask, cv::Rect & bbox, double & area)
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
      area = 0.0;
      return false;
    }

    auto best_it = std::max_element(
      contours.begin(),
      contours.end(),
      [](const auto & a, const auto & b) {
        return cv::contourArea(a) < cv::contourArea(b);
      });

    area = cv::contourArea(*best_it);

    if (area < static_cast<double>(min_mask_pixels_)) {
      return false;
    }

    object_mask = cv::Mat::zeros(mask.size(), CV_8U);
    std::vector<std::vector<cv::Point>> selected{*best_it};
    cv::drawContours(object_mask, selected, 0, cv::Scalar(255), cv::FILLED);

    bbox = cv::boundingRect(*best_it);
    return true;
  }

  float medianOfSmallVector(std::vector<float> & values)
  {
    if (values.empty()) {
      return std::numeric_limits<float>::quiet_NaN();
    }

    const size_t mid = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + mid, values.end());
    float med = values[mid];

    if (values.size() % 2 == 0 && mid > 0) {
      std::nth_element(values.begin(), values.begin() + mid - 1, values.end());
      med = 0.5f * (med + values[mid - 1]);
    }

    return med;
  }

  cv::Mat computeMedianDepthMeters()
  {
    const int height = depth_frames_[0].rows;
    const int width = depth_frames_[0].cols;

    cv::Mat depth_median(height, width, CV_32FC1);
    depth_median.setTo(std::numeric_limits<float>::quiet_NaN());

    std::vector<float> values;
    values.reserve(depth_frames_.size());

    for (int v = 0; v < height; ++v) {
      for (int u = 0; u < width; ++u) {
        values.clear();

        for (const auto & depth_raw : depth_frames_) {
          const uint16_t raw = depth_raw.at<uint16_t>(v, u);
          const float d = static_cast<float>(raw) * static_cast<float>(depth_scale_);

          if (std::isfinite(d) && d >= raw_depth_min_ && d <= raw_depth_max_) {
            values.push_back(d);
          }
        }

        if (!values.empty()) {
          depth_median.at<float>(v, u) = medianOfSmallVector(values);
        }
      }
    }

    return depth_median;
  }

  float medianCoordinate(std::vector<cv::Point3f> points, int axis)
  {
    std::vector<float> values;
    values.reserve(points.size());

    for (const auto & p : points) {
      if (axis == 0) {
        values.push_back(p.x);
      } else if (axis == 1) {
        values.push_back(p.y);
      } else {
        values.push_back(p.z);
      }
    }

    return medianOfSmallVector(values);
  }

  sensor_msgs::msg::PointCloud2 makeCloudMsg(
    const std_msgs::msg::Header & header,
    const std::vector<cv::Point3f> & points)
  {
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = header;
    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(points.size());

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (const auto & p : points) {
      *iter_x = p.x;
      *iter_y = p.y;
      *iter_z = p.z;

      ++iter_x;
      ++iter_y;
      ++iter_z;
    }

    cloud.is_dense = false;
    return cloud;
  }

  geometry_msgs::msg::PointStamped makePointMsg(
    const std_msgs::msg::Header & header,
    const cv::Point3f & p)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header = header;
    msg.point.x = p.x;
    msg.point.y = p.y;
    msg.point.z = p.z;
    return msg;
  }

  visualization_msgs::msg::MarkerArray makeMarkersMsg(
    const std_msgs::msg::Header & header,
    const cv::Point3f & grasp,
    const cv::Point3f & push)
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker grasp_marker;
    grasp_marker.header = header;
    grasp_marker.ns = "scan_grasp_point";
    grasp_marker.id = 0;
    grasp_marker.type = visualization_msgs::msg::Marker::SPHERE;
    grasp_marker.action = visualization_msgs::msg::Marker::ADD;
    grasp_marker.pose.position.x = grasp.x;
    grasp_marker.pose.position.y = grasp.y;
    grasp_marker.pose.position.z = grasp.z;
    grasp_marker.pose.orientation.w = 1.0;
    grasp_marker.scale.x = grasp_marker_scale_;
    grasp_marker.scale.y = grasp_marker_scale_;
    grasp_marker.scale.z = grasp_marker_scale_;
    grasp_marker.color.r = 0.0;
    grasp_marker.color.g = 1.0;
    grasp_marker.color.b = 0.0;
    grasp_marker.color.a = 0.95;
    arr.markers.push_back(grasp_marker);

    visualization_msgs::msg::Marker push_marker;
    push_marker.header = header;
    push_marker.ns = "scan_push_point";
    push_marker.id = 1;
    push_marker.type = visualization_msgs::msg::Marker::CUBE;
    push_marker.action = visualization_msgs::msg::Marker::ADD;
    push_marker.pose.position.x = push.x;
    push_marker.pose.position.y = push.y;
    push_marker.pose.position.z = push.z;
    push_marker.pose.orientation.w = 1.0;
    push_marker.scale.x = push_marker_scale_;
    push_marker.scale.y = push_marker_scale_;
    push_marker.scale.z = push_marker_scale_;
    push_marker.color.r = 1.0;
    push_marker.color.g = 0.5;
    push_marker.color.b = 0.0;
    push_marker.color.a = 0.95;
    arr.markers.push_back(push_marker);

    return arr;
  }

  sensor_msgs::msg::Image makeImageMsg(
    const std_msgs::msg::Header & header,
    const cv::Mat & image,
    const std::string & encoding)
  {
    sensor_msgs::msg::Image msg;
    msg.header = header;
    msg.height = static_cast<uint32_t>(image.rows);
    msg.width = static_cast<uint32_t>(image.cols);
    msg.encoding = encoding;
    msg.is_bigendian = false;
    msg.step = static_cast<uint32_t>(image.step);
    msg.data.resize(image.total() * image.elemSize());
    std::memcpy(msg.data.data(), image.data, msg.data.size());
    return msg;
  }

  void computeOnce(const sensor_msgs::msg::CameraInfo & info)
  {
    scan_done_ = true;

    const auto t0 = now();

    const cv::Mat rgb = rgb_frames_.back().clone();
    const cv::Mat depth_m = computeMedianDepthMeters();

    cv::Mat mask;

    try {
      mask = makeColorMask(rgb);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "mask failed: %s", e.what());
      return;
    }

    cv::Mat object_mask;
    cv::Rect bbox;
    double area = 0.0;

    std_msgs::msg::Header header;
    header.stamp = get_clock()->now();
    header.frame_id = frame_id_;

    if (!extractLargestComponent(mask, object_mask, bbox, area)) {
      RCLCPP_ERROR(
        get_logger(),
        "scan failed: target mask not found or too small, area=%.1f",
        area);

      result_mask_msg_ = makeImageMsg(header, mask, "mono8");
      result_ready_mask_only_ = true;
      mask_pub_->publish(result_mask_msg_);
      return;
    }

    const double fx = info.k[0];
    const double fy = info.k[4];
    const double cx = info.k[2];
    const double cy = info.k[5];

    std::vector<cv::Point3f> valid_points;
    valid_points.reserve(static_cast<size_t>(cv::countNonZero(object_mask)));

    for (int v = 0; v < object_mask.rows; ++v) {
      for (int u = 0; u < object_mask.cols; ++u) {
        if (object_mask.at<uint8_t>(v, u) == 0) {
          continue;
        }

        const float z = depth_m.at<float>(v, u);

        if (!std::isfinite(z) || z < raw_depth_min_ || z > raw_depth_max_) {
          continue;
        }

        const float x = static_cast<float>((static_cast<double>(u) - cx) * z / fx);
        const float y = static_cast<float>((static_cast<double>(v) - cy) * z / fy);

        valid_points.emplace_back(x, y, z);
      }
    }

    const int valid_count = static_cast<int>(valid_points.size());

    if (valid_count < min_valid_depth_pixels_) {
      RCLCPP_ERROR(
        get_logger(),
        "scan failed: not enough valid depth in mask: %d",
        valid_count);

      result_mask_msg_ = makeImageMsg(header, object_mask, "mono8");
      result_ready_mask_only_ = true;
      mask_pub_->publish(result_mask_msg_);
      return;
    }

    cv::Point3f center;
    center.x = medianCoordinate(valid_points, 0);
    center.y = medianCoordinate(valid_points, 1);
    center.z = medianCoordinate(valid_points, 2);

    cv::Point3f grasp_point = center;
    cv::Point3f push_point = center;

    std::vector<cv::Point3f> cloud_points;

    if (valid_count > max_cloud_points_) {
      cloud_points.reserve(static_cast<size_t>(max_cloud_points_));

      for (int i = 0; i < max_cloud_points_; ++i) {
        const int idx = static_cast<int>(
          std::round(
            static_cast<double>(i) *
            static_cast<double>(valid_count - 1) /
            static_cast<double>(max_cloud_points_ - 1)));

        cloud_points.push_back(valid_points[static_cast<size_t>(idx)]);
      }
    } else {
      cloud_points = valid_points;
    }

    cv::Mat annotated = rgb.clone();
    cv::rectangle(annotated, bbox, cv::Scalar(255, 0, 0), 2);

    const int center_u = bbox.x + bbox.width / 2;
    const int center_v = bbox.y + bbox.height / 2;
    cv::circle(annotated, cv::Point(center_u, center_v), 4, cv::Scalar(0, 255, 0), cv::FILLED);

    char text[256];
    std::snprintf(
      text,
      sizeof(text),
      "scan xyz=(%.3f, %.3f, %.3f)",
      grasp_point.x,
      grasp_point.y,
      grasp_point.z);

    cv::putText(
      annotated,
      text,
      cv::Point(std::max(0, bbox.x), std::max(15, bbox.y - 5)),
      cv::FONT_HERSHEY_SIMPLEX,
      0.4,
      cv::Scalar(255, 255, 255),
      1,
      cv::LINE_AA);

    result_cloud_msg_ = makeCloudMsg(header, cloud_points);
    result_grasp_msg_ = makePointMsg(header, grasp_point);
    result_push_msg_ = makePointMsg(header, push_point);
    result_markers_msg_ = makeMarkersMsg(header, grasp_point, push_point);
    result_annotated_msg_ = makeImageMsg(header, annotated, "rgb8");
    result_mask_msg_ = makeImageMsg(header, object_mask, "mono8");

    result_ready_ = true;
    result_ready_mask_only_ = false;

    publishResult();

    const double dt = (now() - t0).seconds();

    RCLCPP_INFO(get_logger(), "==== SCAN RESULT ====");
    RCLCPP_INFO(get_logger(), "captured_frames=%d", capture_frames_);
    RCLCPP_INFO(
      get_logger(),
      "bbox_2d=[%d, %d, %d, %d]",
      bbox.x,
      bbox.y,
      bbox.x + bbox.width,
      bbox.y + bbox.height);
    RCLCPP_INFO(get_logger(), "mask_pixels=%d", cv::countNonZero(object_mask));
    RCLCPP_INFO(get_logger(), "valid_depth_pixels=%d", valid_count);
    RCLCPP_INFO(get_logger(), "cloud_points=%zu", cloud_points.size());
    RCLCPP_INFO(
      get_logger(),
      "grasp_point camera frame: x=%.3f, y=%.3f, z=%.3f",
      grasp_point.x,
      grasp_point.y,
      grasp_point.z);
    RCLCPP_INFO(
      get_logger(),
      "push_point camera frame:  x=%.3f, y=%.3f, z=%.3f",
      push_point.x,
      push_point.y,
      push_point.z);
    RCLCPP_INFO(get_logger(), "compute_time=%.3f sec", dt);
    RCLCPP_INFO(get_logger(), "=====================");
  }

  void updateResultStamps()
  {
    const auto stamp = get_clock()->now();

    if (result_ready_) {
      result_cloud_msg_.header.stamp = stamp;
      result_grasp_msg_.header.stamp = stamp;
      result_push_msg_.header.stamp = stamp;
      result_annotated_msg_.header.stamp = stamp;
      result_mask_msg_.header.stamp = stamp;

      for (auto & marker : result_markers_msg_.markers) {
        marker.header.stamp = stamp;
      }
    } else if (result_ready_mask_only_) {
      result_mask_msg_.header.stamp = stamp;
    }
  }

  void publishResult()
  {
    updateResultStamps();

    if (result_ready_) {
      cloud_pub_->publish(result_cloud_msg_);
      grasp_pub_->publish(result_grasp_msg_);
      push_pub_->publish(result_push_msg_);
      marker_pub_->publish(result_markers_msg_);
      annotated_pub_->publish(result_annotated_msg_);
      mask_pub_->publish(result_mask_msg_);
    } else if (result_ready_mask_only_) {
      mask_pub_->publish(result_mask_msg_);
    }
  }

  void republishResult()
  {
    if (scan_done_ && (result_ready_ || result_ready_mask_only_)) {
      publishResult();
    }
  }

private:
  std::mutex mutex_;

  std::string rgb_topic_;
  std::string depth_topic_;
  std::string info_topic_;

  std::string frame_id_;
  std::string target_color_;

  int capture_frames_;
  double depth_scale_;
  double raw_depth_min_;
  double raw_depth_max_;
  int min_mask_pixels_;
  int min_valid_depth_pixels_;
  int max_cloud_points_;
  double publish_period_;
  double grasp_marker_scale_;
  double push_marker_scale_;

  std::string cloud_topic_;
  std::string grasp_topic_;
  std::string push_topic_;
  std::string marker_topic_;
  std::string annotated_topic_;
  std::string mask_topic_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr grasp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr push_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr annotated_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_pub_;

  rclcpp::TimerBase::SharedPtr collect_timer_;
  rclcpp::TimerBase::SharedPtr republish_timer_;

  sensor_msgs::msg::Image::SharedPtr latest_rgb_msg_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_msg_;
  sensor_msgs::msg::CameraInfo::SharedPtr latest_info_msg_;

  int64_t last_collected_depth_stamp_ns_{-1};

  std::vector<cv::Mat> rgb_frames_;
  std::vector<cv::Mat> depth_frames_;

  bool scan_done_{false};
  bool result_ready_{false};
  bool result_ready_mask_only_{false};

  sensor_msgs::msg::PointCloud2 result_cloud_msg_;
  geometry_msgs::msg::PointStamped result_grasp_msg_;
  geometry_msgs::msg::PointStamped result_push_msg_;
  visualization_msgs::msg::MarkerArray result_markers_msg_;
  sensor_msgs::msg::Image result_annotated_msg_;
  sensor_msgs::msg::Image result_mask_msg_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraOnlyScanOnceCppNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
