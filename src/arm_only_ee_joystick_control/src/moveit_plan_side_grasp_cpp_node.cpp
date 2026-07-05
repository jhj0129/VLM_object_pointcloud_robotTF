#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"


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


static moveit::core::RobotState make_state_from_plan_end(
  const moveit::core::RobotState & fallback_state,
  const moveit::planning_interface::MoveGroupInterface::Plan & plan)
{
  moveit::core::RobotState state(fallback_state);

  const auto & jt = plan.trajectory_.joint_trajectory;
  if (!jt.points.empty()) {
    const auto & last = jt.points.back();
    const size_t n = std::min(jt.joint_names.size(), last.positions.size());
    for (size_t i = 0; i < n; ++i) {
      state.setVariablePosition(jt.joint_names[i], last.positions[i]);
    }
    state.update();
  }

  return state;
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit_plan_side_grasp_cpp_node",
    options);

  const std::string pregrasp_pose_topic = get_or_declare<std::string>(
    node, "pregrasp_pose_topic", "/vlm_moveit/pregrasp_pose");

  const std::string grasp_pose_topic = get_or_declare<std::string>(
    node, "grasp_pose_topic", "/vlm_moveit/grasp_target_pose");

  const std::string planning_group = get_or_declare<std::string>(
    node, "planning_group", "arm");

  const std::string end_effector_link = get_or_declare<std::string>(
    node, "end_effector_link", "gripper_tcp");

  const std::string display_trajectory_topic = get_or_declare<std::string>(
    node, "display_trajectory_topic", "/display_planned_path");

  const bool plan_once = get_or_declare<bool>(
    node, "plan_once", true);

  const double planning_time = get_or_declare<double>(
    node, "planning_time", 8.0);

  const int planning_attempts = get_or_declare<int>(
    node, "planning_attempts", 20);

  const double velocity_scaling = get_or_declare<double>(
    node, "velocity_scaling", 0.10);

  const double accel_scaling = get_or_declare<double>(
    node, "accel_scaling", 0.10);

  const double cartesian_eef_step = get_or_declare<double>(
    node, "cartesian_eef_step", 0.005);

  const double cartesian_jump_threshold = get_or_declare<double>(
    node, "cartesian_jump_threshold", 0.0);

  const double cartesian_min_fraction = get_or_declare<double>(
    node, "cartesian_min_fraction", 0.95);

  const bool avoid_collisions = get_or_declare<bool>(
    node, "avoid_collisions", true);

  RCLCPP_INFO(node->get_logger(), "moveit_plan_side_grasp_cpp_node started");
  RCLCPP_INFO(node->get_logger(), "pregrasp_pose_topic=%s", pregrasp_pose_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "grasp_pose_topic=%s", grasp_pose_topic.c_str());
  RCLCPP_INFO(node->get_logger(), "planning_group=%s", planning_group.c_str());
  RCLCPP_INFO(node->get_logger(), "end_effector_link=%s", end_effector_link.c_str());
  RCLCPP_INFO(node->get_logger(), "PLAN ONLY: no execute call exists in this node");
  RCLCPP_INFO(node->get_logger(), "cartesian_eef_step=%.4f", cartesian_eef_step);
  RCLCPP_INFO(node->get_logger(), "cartesian_min_fraction=%.3f", cartesian_min_fraction);

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

  auto mutex = std::make_shared<std::mutex>();
  auto move_group_mutex = std::make_shared<std::mutex>();

  auto have_pregrasp = std::make_shared<bool>(false);
  auto have_grasp = std::make_shared<bool>(false);

  auto latest_pregrasp = std::make_shared<geometry_msgs::msg::PoseStamped>();
  auto latest_grasp = std::make_shared<geometry_msgs::msg::PoseStamped>();

  auto attempted = std::make_shared<std::atomic_bool>(false);
  auto planning_now = std::make_shared<std::atomic_bool>(false);

  auto try_plan =
    [
      node,
      move_group,
      display_pub,
      mutex,
      move_group_mutex,
      have_pregrasp,
      have_grasp,
      latest_pregrasp,
      latest_grasp,
      attempted,
      planning_now,
      plan_once,
      end_effector_link,
      cartesian_eef_step,
      cartesian_jump_threshold,
      cartesian_min_fraction,
      avoid_collisions
    ]()
    {
      geometry_msgs::msg::PoseStamped pregrasp;
      geometry_msgs::msg::PoseStamped grasp;

      {
        std::lock_guard<std::mutex> lock(*mutex);

        if (!(*have_pregrasp) || !(*have_grasp)) {
          return;
        }

        if (plan_once && attempted->load()) {
          RCLCPP_INFO_THROTTLE(
            node->get_logger(),
            *node->get_clock(),
            3000,
            "plan_once=true and side grasp already attempted. Skip repeated poses.");
          return;
        }

        bool expected = false;
        if (!planning_now->compare_exchange_strong(expected, true)) {
          RCLCPP_WARN(node->get_logger(), "side grasp planning is already running. Skip.");
          return;
        }

        attempted->store(true);
        pregrasp = *latest_pregrasp;
        grasp = *latest_grasp;
      }

      std::thread(
        [
          node,
          move_group,
          display_pub,
          move_group_mutex,
          planning_now,
          end_effector_link,
          cartesian_eef_step,
          cartesian_jump_threshold,
          cartesian_min_fraction,
          avoid_collisions,
          pregrasp,
          grasp
        ]()
        {
          RCLCPP_INFO(node->get_logger(), "side grasp planning thread started");

          const double dx = grasp.pose.position.x - pregrasp.pose.position.x;
          const double dy = grasp.pose.position.y - pregrasp.pose.position.y;
          const double dz = grasp.pose.position.z - pregrasp.pose.position.z;

          RCLCPP_INFO(
            node->get_logger(),
            "SIDE GRASP VECTOR pregrasp -> grasp = (%.3f, %.3f, %.3f), length=%.3f m",
            dx,
            dy,
            dz,
            std::sqrt(dx * dx + dy * dy + dz * dz));

          RCLCPP_INFO(
            node->get_logger(),
            "target 1 PREGRASP: frame=%s, pos=(%.3f, %.3f, %.3f), quat=(%.4f %.4f %.4f %.4f)",
            pregrasp.header.frame_id.c_str(),
            pregrasp.pose.position.x,
            pregrasp.pose.position.y,
            pregrasp.pose.position.z,
            pregrasp.pose.orientation.x,
            pregrasp.pose.orientation.y,
            pregrasp.pose.orientation.z,
            pregrasp.pose.orientation.w);

          RCLCPP_INFO(
            node->get_logger(),
            "target 2 GRASP: frame=%s, pos=(%.3f, %.3f, %.3f), quat=(%.4f %.4f %.4f %.4f)",
            grasp.header.frame_id.c_str(),
            grasp.pose.position.x,
            grasp.pose.position.y,
            grasp.pose.position.z,
            grasp.pose.orientation.x,
            grasp.pose.orientation.y,
            grasp.pose.orientation.z,
            grasp.pose.orientation.w);

          std::lock_guard<std::mutex> lock(*move_group_mutex);

          auto current_state = move_group->getCurrentState(2.0);
          if (!current_state) {
            RCLCPP_ERROR(node->get_logger(), "failed to get current robot state");
            planning_now->store(false);
            return;
          }

          // Plan 1: current -> pregrasp, FULL POSE.
          move_group->setStartStateToCurrentState();
          move_group->clearPoseTargets();
          move_group->setPoseTarget(pregrasp, end_effector_link);

          moveit::planning_interface::MoveGroupInterface::Plan plan1;

          const auto t1 = node->now();
          const auto result1 = move_group->plan(plan1);
          const double dt1 = (node->now() - t1).seconds();

          if (result1 != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(
              node->get_logger(),
              "SIDE GRASP PLAN 1 FAILED: current -> pregrasp FULL POSE, time=%.3f sec, error_code=%d",
              dt1,
              result1.val);
            move_group->clearPoseTargets();
            planning_now->store(false);
            return;
          }

          RCLCPP_INFO(
            node->get_logger(),
            "SIDE GRASP PLAN 1 SUCCESS: current -> pregrasp FULL POSE, time=%.3f sec, points=%zu",
            dt1,
            plan1.trajectory_.joint_trajectory.points.size());

          // Plan 2: pregrasp -> grasp, Cartesian straight approach.
          moveit::core::RobotState pregrasp_end_state =
            make_state_from_plan_end(*current_state, plan1);

          move_group->setStartState(pregrasp_end_state);
          move_group->clearPoseTargets();

          std::vector<geometry_msgs::msg::Pose> waypoints;
          waypoints.push_back(grasp.pose);

          moveit_msgs::msg::RobotTrajectory cartesian_traj;

          const auto t2 = node->now();
          const double fraction = move_group->computeCartesianPath(
            waypoints,
            cartesian_eef_step,
            cartesian_jump_threshold,
            cartesian_traj,
            avoid_collisions);
          const double dt2 = (node->now() - t2).seconds();

          if (fraction < cartesian_min_fraction) {
            RCLCPP_ERROR(
              node->get_logger(),
              "SIDE GRASP CARTESIAN FAILED: pregrasp -> grasp, fraction=%.3f < %.3f, time=%.3f sec, points=%zu",
              fraction,
              cartesian_min_fraction,
              dt2,
              cartesian_traj.joint_trajectory.points.size());

            moveit_msgs::msg::DisplayTrajectory display_msg;
            display_msg.trajectory_start = plan1.start_state_;
            display_msg.trajectory.push_back(plan1.trajectory_);
            display_pub->publish(display_msg);

            RCLCPP_WARN(
              node->get_logger(),
              "published only plan 1 because Cartesian approach failed");

            move_group->clearPoseTargets();
            planning_now->store(false);
            return;
          }

          RCLCPP_INFO(
            node->get_logger(),
            "SIDE GRASP CARTESIAN SUCCESS: pregrasp -> grasp, fraction=%.3f, time=%.3f sec, points=%zu",
            fraction,
            dt2,
            cartesian_traj.joint_trajectory.points.size());

          moveit_msgs::msg::DisplayTrajectory display_msg;
          display_msg.trajectory_start = plan1.start_state_;
          display_msg.trajectory.push_back(plan1.trajectory_);
          display_msg.trajectory.push_back(cartesian_traj);
          display_pub->publish(display_msg);

          RCLCPP_INFO(
            node->get_logger(),
            "SIDE GRASP PLAN ONLY SUCCESS: published full-pose plan1 + Cartesian plan2 to %s",
            display_pub->get_topic_name());

          move_group->clearPoseTargets();
          planning_now->store(false);
          RCLCPP_INFO(node->get_logger(), "side grasp planning thread finished");
        }
      ).detach();
    };

  auto pregrasp_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    pregrasp_pose_topic,
    qos,
    [
      mutex,
      have_pregrasp,
      latest_pregrasp,
      try_plan,
      node
    ](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      {
        std::lock_guard<std::mutex> lock(*mutex);
        *latest_pregrasp = *msg;
        *have_pregrasp = true;
      }

      RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        3000,
        "received pregrasp pose: pos=(%.3f, %.3f, %.3f)",
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);

      try_plan();
    });

  auto grasp_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    grasp_pose_topic,
    qos,
    [
      mutex,
      have_grasp,
      latest_grasp,
      try_plan,
      node
    ](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
      {
        std::lock_guard<std::mutex> lock(*mutex);
        *latest_grasp = *msg;
        *have_grasp = true;
      }

      RCLCPP_INFO_THROTTLE(
        node->get_logger(),
        *node->get_clock(),
        3000,
        "received grasp pose: pos=(%.3f, %.3f, %.3f)",
        msg->pose.position.x,
        msg->pose.position.y,
        msg->pose.position.z);

      try_plan();
    });

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
