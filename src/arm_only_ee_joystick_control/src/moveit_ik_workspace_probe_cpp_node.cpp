#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

template <typename T>
T get_param(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

static std_msgs::msg::ColorRGBA make_color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit_ik_workspace_probe_cpp_node",
    options);

  const std::string planning_group = get_param<std::string>(node, "planning_group", "arm");
  const std::string tip_link = get_param<std::string>(node, "tip_link", "gripper_tcp");
  const std::string frame_id = get_param<std::string>(node, "frame_id", "ARM_BASE_LINK");
  const std::string marker_topic = get_param<std::string>(node, "marker_topic", "/vlm_debug/ik_workspace_markers");
  const std::string csv_path = get_param<std::string>(node, "csv_path", "/tmp/vlm_side_grasp_ik_workspace.csv");

  const double x_min = get_param<double>(node, "x_min", 0.10);
  const double x_max = get_param<double>(node, "x_max", 0.75);
  const double y_min = get_param<double>(node, "y_min", -0.35);
  const double y_max = get_param<double>(node, "y_max", 0.35);
  const double z_min = get_param<double>(node, "z_min", -0.10);
  const double z_max = get_param<double>(node, "z_max", 0.55);
  const double step = get_param<double>(node, "step", 0.05);

  const double qx = get_param<double>(node, "target_qx", 1.000);
  const double qy = get_param<double>(node, "target_qy", 0.000);
  const double qz = get_param<double>(node, "target_qz", -0.004);
  const double qw = get_param<double>(node, "target_qw", -0.002);

  const double ik_timeout = get_param<double>(node, "ik_timeout", 0.01);
  const int attempts = get_param<int>(node, "attempts", 8);
  const double pos_tol = get_param<double>(node, "pos_tol", 0.005);
  const double rot_tol = get_param<double>(node, "rot_tol", 0.10);

  RCLCPP_INFO(node->get_logger(), "IK workspace probe started");
  RCLCPP_INFO(node->get_logger(), "group=%s tip=%s frame=%s", planning_group.c_str(), tip_link.c_str(), frame_id.c_str());
  RCLCPP_INFO(node->get_logger(), "range x=[%.3f %.3f], y=[%.3f %.3f], z=[%.3f %.3f], step=%.3f",
              x_min, x_max, y_min, y_max, z_min, z_max, step);
  RCLCPP_INFO(node->get_logger(), "target quaternion xyzw=(%.4f %.4f %.4f %.4f)", qx, qy, qz, qw);
  RCLCPP_INFO(node->get_logger(), "csv=%s", csv_path.c_str());

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, planning_group);
  move_group->setEndEffectorLink(tip_link);

  auto robot_model = move_group->getRobotModel();
  const auto * jmg = robot_model->getJointModelGroup(planning_group);

  if (!jmg) {
    RCLCPP_ERROR(node->get_logger(), "JointModelGroup not found: %s", planning_group.c_str());
    rclcpp::shutdown();
    return 1;
  }

  moveit::core::RobotState seed_state(robot_model);
  auto current_state = move_group->getCurrentState(2.0);
  if (current_state) {
    seed_state = *current_state;
    RCLCPP_INFO(node->get_logger(), "using current robot state as IK seed");
  } else {
    seed_state.setToDefaultValues();
    RCLCPP_WARN(node->get_logger(), "current state not available. using default state as IK seed");
  }

  Eigen::Quaterniond q_target(qw, qx, qy, qz);
  q_target.normalize();

  auto pub = node->create_publisher<visualization_msgs::msg::MarkerArray>(
    marker_topic,
    rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

  visualization_msgs::msg::Marker reachable_marker;
  reachable_marker.header.frame_id = frame_id;
  reachable_marker.ns = "ik_reachable";
  reachable_marker.id = 1;
  reachable_marker.type = visualization_msgs::msg::Marker::POINTS;
  reachable_marker.action = visualization_msgs::msg::Marker::ADD;
  reachable_marker.scale.x = step * 0.35;
  reachable_marker.scale.y = step * 0.35;
  reachable_marker.color = make_color(0.0f, 1.0f, 0.0f, 0.85f);

  visualization_msgs::msg::Marker fail_marker;
  fail_marker.header.frame_id = frame_id;
  fail_marker.ns = "ik_failed";
  fail_marker.id = 2;
  fail_marker.type = visualization_msgs::msg::Marker::POINTS;
  fail_marker.action = visualization_msgs::msg::Marker::ADD;
  fail_marker.scale.x = step * 0.25;
  fail_marker.scale.y = step * 0.25;
  fail_marker.color = make_color(1.0f, 0.0f, 0.0f, 0.25f);

  std::ofstream csv(csv_path);
  csv << "x,y,z,reachable,pos_error,rot_error_deg,used_attempt";
  for (const auto & name : jmg->getVariableNames()) {
    csv << "," << name;
  }
  csv << "\n";

  int total = 0;
  int reachable_count = 0;

  for (double z = z_min; z <= z_max + 1e-9; z += step) {
    for (double y = y_min; y <= y_max + 1e-9; y += step) {
      for (double x = x_min; x <= x_max + 1e-9; x += step) {
        total++;

        Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
        target.linear() = q_target.toRotationMatrix();
        target.translation() = Eigen::Vector3d(x, y, z);

        bool ok = false;
        double best_pos_error = 999.0;
        double best_rot_error = 999.0;
        int used_attempt = -1;
        moveit::core::RobotState best_state(seed_state);

        for (int a = 0; a < attempts; ++a) {
          moveit::core::RobotState test_state(seed_state);

          if (a > 0) {
            test_state.setToRandomPositions(jmg);
            test_state.update();
          }

          bool ik_ok = test_state.setFromIK(jmg, target, tip_link, ik_timeout);
          if (!ik_ok) {
            continue;
          }

          test_state.update();

          const Eigen::Isometry3d & fk = test_state.getGlobalLinkTransform(tip_link);

          double pos_error = (fk.translation() - target.translation()).norm();

          Eigen::Quaterniond q_fk(fk.rotation());
          q_fk.normalize();

          double dot = std::abs(q_fk.dot(q_target));
          dot = std::min(1.0, std::max(0.0, dot));
          double rot_error = 2.0 * std::acos(dot);

          if (pos_error < best_pos_error) {
            best_pos_error = pos_error;
            best_rot_error = rot_error;
            best_state = test_state;
            used_attempt = a;
          }

          if (pos_error <= pos_tol && rot_error <= rot_tol) {
            ok = true;
            break;
          }
        }

        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;

        if (ok) {
          reachable_count++;
          reachable_marker.points.push_back(p);
        } else {
          fail_marker.points.push_back(p);
        }

        csv << std::fixed << std::setprecision(5)
            << x << "," << y << "," << z << ","
            << (ok ? 1 : 0) << ","
            << best_pos_error << ","
            << best_rot_error * 180.0 / M_PI << ","
            << used_attempt;

        for (const auto & name : jmg->getVariableNames()) {
          csv << "," << best_state.getVariablePosition(name);
        }
        csv << "\n";
      }
    }

    RCLCPP_INFO(node->get_logger(), "finished z=%.3f", z);
  }

  csv.close();

  reachable_marker.header.stamp = node->now();
  fail_marker.header.stamp = node->now();

  visualization_msgs::msg::MarkerArray arr;
  arr.markers.push_back(reachable_marker);
  arr.markers.push_back(fail_marker);
  pub->publish(arr);

  double rate = total > 0 ? 100.0 * static_cast<double>(reachable_count) / static_cast<double>(total) : 0.0;

  RCLCPP_INFO(node->get_logger(), "IK workspace probe done");
  RCLCPP_INFO(node->get_logger(), "reachable=%d / total=%d, rate=%.2f %%", reachable_count, total, rate);
  RCLCPP_INFO(node->get_logger(), "CSV saved: %s", csv_path.c_str());
  RCLCPP_INFO(node->get_logger(), "Marker published: %s", marker_topic.c_str());

  rclcpp::sleep_for(std::chrono::seconds(2));
  rclcpp::shutdown();
  return 0;
}
