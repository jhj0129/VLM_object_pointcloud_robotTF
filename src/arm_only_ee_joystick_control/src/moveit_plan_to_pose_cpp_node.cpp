#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/display_trajectory.hpp"


template <typename T>
T get_or_declare(
  const rclcpp::Node::SharedPtr & node,
  const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit_plan_to_pose_cpp_node",
    options);

  const std::string input_pose_topic = get_or_declare<std::string>(
    node,
    "input_pose_topic",
    "/vlm_moveit/pregrasp_pose");

  const std::string planning_group = get_or_declare<std::string>(
    node,
    "planning_group",
    "arm");

  const std::string end_effector_link = get_or_declare<std::string>(
    node,
    "end_effector_link",
    "LINK6");

  const std::string display_trajectory_topic = get_or_declare<std::string>(
    node,
    "display_trajectory_topic",
    "/display_planned_path");

  const bool use_position_only = get_or_declare<bool>(
    node,
    "use_position_only",
    true);

  const bool plan_once = get_or_declare<bool>(
    node,
    "plan_once",
    true);

  const double planning_time = get_or_declare<double>(
    node,
    "planning_time",
    5.0);

  const int planning_attempts = get_or_declare<int>(
    node,
    "planning_attempts",
    10);

  const double velocity_scaling = get_or_declare<double>(
    node,
    "velocity_scaling",
    0.10);

  const double accel_scaling = get_or_declare<double>(
    node,
    "accel_scaling",
    0.10);

  RCLCPP_INFO(node->get_logger(), "moveit_plan_to_pose_cpp_node started");
  RCLCPP_INFO(node->get_logger(), "input_pose_topic=%s", input_pose_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "planning_group=%s", planning_group.c_str());
  RCLCPP_INFO(node->get_logger(), "end_effector_link=%s", end_effector_link.c_str());
  RCLCPP_INFO(node->get_logger(), "use_position_only=%s", use_position_only ? "true" : "false");
  RCLCPP_INFO(node->get_logger(), "plan_once=%s", plan_once ? "true" : "false");

  auto move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    node,
    planning_group);

  if (!end_effector_link.empty()) {
    move_group->setEndEffectorLink(end_effector_link);
  }

  move_group->setPlanningTime(planning_time);
  move_group->setNumPlanningAttempts(planning_attempts);
  move_group->setMaxVelocityScalingFactor(velocity_scaling);
  move_group->setMaxAccelerationScalingFactor(accel_scaling);

  auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  auto display_pub = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    display_trajectory_topic,
    qos);

  auto attempted = std::make_shared<std::atomic_bool>(false);
  auto planning_now = std::make_shared<std::atomic_bool>(false);
  auto move_group_mutex = std::make_shared<std::mutex>();

  auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    input_pose_topic,
    qos,
    [
      move_group,
      display_pub,
      attempted,
      planning_now,
      move_group_mutex,
      node,
      use_position_only,
      plan_once,
      end_effector_link
    ](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      if (plan_once && attempted->load()) {
        RCLCPP_INFO_THROTTLE(
          node->get_logger(),
          *node->get_clock(),
          3000,
          "plan_once=true and already attempted. Skip repeated pose.");
        return;
      }

      bool expected = false;
      if (!planning_now->compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(node->get_logger(), "planning is already running. Skip this pose.");
        return;
      }

      attempted->store(true);

      geometry_msgs::msg::PoseStamped target = *msg;

      RCLCPP_INFO(
        node->get_logger(),
        "received target pose: frame=%s, pos=(%.3f, %.3f, %.3f)",
        target.header.frame_id.c_str(),
        target.pose.position.x,
        target.pose.position.y,
        target.pose.position.z);

      std::thread(
        [
          move_group,
          display_pub,
          planning_now,
          move_group_mutex,
          node,
          use_position_only,
          end_effector_link,
          target
        ]()
        {
          RCLCPP_INFO(node->get_logger(), "planning thread started");

          std::lock_guard<std::mutex> lock(*move_group_mutex);

          move_group->setStartStateToCurrentState();
          move_group->clearPoseTargets();

          if (use_position_only) {
            RCLCPP_INFO(node->get_logger(), "planning mode: POSITION ONLY");

            if (!end_effector_link.empty()) {
              move_group->setPositionTarget(
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z,
                end_effector_link);
            } else {
              move_group->setPositionTarget(
                target.pose.position.x,
                target.pose.position.y,
                target.pose.position.z);
            }
          } else {
            RCLCPP_INFO(node->get_logger(), "planning mode: FULL POSE");

            if (!end_effector_link.empty()) {
              move_group->setPoseTarget(target, end_effector_link);
            } else {
              move_group->setPoseTarget(target);
            }
          }

          moveit::planning_interface::MoveGroupInterface::Plan plan;

          const auto t0 = node->now();
          const auto result = move_group->plan(plan);
          const double dt = (node->now() - t0).seconds();

          if (result == moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_INFO(
              node->get_logger(),
              "PLAN SUCCESS, planning_time=%.3f sec, trajectory_points=%zu",
              dt,
              plan.trajectory_.joint_trajectory.points.size());

            moveit_msgs::msg::DisplayTrajectory display_msg;
            display_msg.trajectory_start = plan.start_state_;
            display_msg.trajectory.push_back(plan.trajectory_);
            display_pub->publish(display_msg);

            RCLCPP_INFO(
              node->get_logger(),
              "published planned trajectory to %s",
              display_pub->get_topic_name());
          } else {
            RCLCPP_ERROR(
              node->get_logger(),
              "PLAN FAILED, planning_time=%.3f sec, error_code=%d",
              dt,
              result.val);
          }

          move_group->clearPoseTargets();
          planning_now->store(false);

          RCLCPP_INFO(node->get_logger(), "planning thread finished");
        }
      ).detach();
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
