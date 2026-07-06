#include <algorithm>
#include <cmath>
#include <iomanip>
#include <map>
#include <memory>
#include <sstream>
#include <set>
#include <string>
#include <vector>

#include <Eigen/Geometry>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include "std_msgs/msg/color_rgba.hpp"

#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/robot_trajectory/robot_trajectory.h"

#include "moveit_msgs/msg/display_trajectory.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

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

static double deg2rad(double deg)
{
  return deg * M_PI / 180.0;
}

static double clamp_value(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static std_msgs::msg::ColorRGBA color(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA c;
  c.r = r;
  c.g = g;
  c.b = b;
  c.a = a;
  return c;
}

static Eigen::Quaterniond make_quat_from_x_axis_and_roll(
  const Eigen::Vector3d & x_axis_input,
  double roll_rad)
{
  Eigen::Vector3d x_axis = x_axis_input.normalized();

  Eigen::Vector3d ref(0.0, 0.0, 1.0);
  if (std::abs(x_axis.dot(ref)) > 0.95) {
    ref = Eigen::Vector3d(0.0, 1.0, 0.0);
  }

  Eigen::Vector3d y_axis = ref.cross(x_axis).normalized();
  Eigen::Vector3d z_axis = x_axis.cross(y_axis).normalized();

  Eigen::Matrix3d base;
  base.col(0) = x_axis;
  base.col(1) = y_axis;
  base.col(2) = z_axis;

  Eigen::Matrix3d rolled =
    base * Eigen::AngleAxisd(roll_rad, Eigen::Vector3d::UnitX()).toRotationMatrix();

  Eigen::Quaterniond q(rolled);
  q.normalize();
  return q;
}

static Eigen::Quaterniond apply_local_x_roll_offset(
  const Eigen::Quaterniond & base_q,
  double roll_offset_rad)
{
  Eigen::Quaterniond q =
    base_q * Eigen::Quaterniond(Eigen::AngleAxisd(roll_offset_rad, Eigen::Vector3d::UnitX()));
  q.normalize();
  return q;
}

static geometry_msgs::msg::PoseStamped make_pose(
  const std::string & frame_id,
  const Eigen::Vector3d & p,
  const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = p.x();
  pose.pose.position.y = p.y();
  pose.pose.position.z = p.z();
  pose.pose.orientation.x = q.x();
  pose.pose.orientation.y = q.y();
  pose.pose.orientation.z = q.z();
  pose.pose.orientation.w = q.w();
  return pose;
}

static Eigen::Isometry3d pose_to_eigen(const geometry_msgs::msg::PoseStamped & pose)
{
  Eigen::Quaterniond q(
    pose.pose.orientation.w,
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z);
  q.normalize();

  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.linear() = q.toRotationMatrix();
  t.translation() = Eigen::Vector3d(
    pose.pose.position.x,
    pose.pose.position.y,
    pose.pose.position.z);
  return t;
}

static visualization_msgs::msg::Marker make_sphere(
  int id,
  const std::string & ns,
  const std::string & frame_id,
  const Eigen::Vector3d & p,
  double scale,
  const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::SPHERE;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = p.x();
  m.pose.position.y = p.y();
  m.pose.position.z = p.z();
  m.pose.orientation.w = 1.0;
  m.scale.x = scale;
  m.scale.y = scale;
  m.scale.z = scale;
  m.color = c;
  return m;
}

static visualization_msgs::msg::Marker make_arrow(
  int id,
  const std::string & ns,
  const std::string & frame_id,
  const Eigen::Vector3d & a,
  const Eigen::Vector3d & b,
  double shaft,
  double head,
  const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::ARROW;
  m.action = visualization_msgs::msg::Marker::ADD;

  geometry_msgs::msg::Point p0;
  p0.x = a.x();
  p0.y = a.y();
  p0.z = a.z();

  geometry_msgs::msg::Point p1;
  p1.x = b.x();
  p1.y = b.y();
  p1.z = b.z();

  m.points.push_back(p0);
  m.points.push_back(p1);

  m.scale.x = shaft;
  m.scale.y = head;
  m.scale.z = head;
  m.color = c;
  return m;
}

static visualization_msgs::msg::Marker make_text(
  int id,
  const std::string & ns,
  const std::string & frame_id,
  const Eigen::Vector3d & p,
  const std::string & text,
  const std_msgs::msg::ColorRGBA & c)
{
  visualization_msgs::msg::Marker m;
  m.header.frame_id = frame_id;
  m.ns = ns;
  m.id = id;
  m.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  m.action = visualization_msgs::msg::Marker::ADD;
  m.pose.position.x = p.x();
  m.pose.position.y = p.y();
  m.pose.position.z = p.z();
  m.pose.orientation.w = 1.0;
  m.scale.z = 0.035;
  m.color = c;
  m.text = text;
  return m;
}

struct Candidate
{
  int id = 0;
  std::string strategy;
  double roll_deg = 0.0;
  double pregrasp_dist = 0.10;
  Eigen::Vector3d approach_dir = Eigen::Vector3d(1.0, 0.0, 0.0);
  geometry_msgs::msg::PoseStamped grasp;
  geometry_msgs::msg::PoseStamped pregrasp;
};

struct CandidatePlan
{
  Candidate candidate;
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  moveit_msgs::msg::RobotTrajectory segment2;
  bool used_cartesian = false;
  double cartesian_fraction = 0.0;
  double motion_cost = 1e9;
  double top_error_deg = 180.0;
  double score = 1e9;
};

class GraspCandidatePlanner
{
public:
  GraspCandidatePlanner(
    const rclcpp::Node::SharedPtr & node,
    const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & move_group)
  : node_(node),
    move_group_(move_group)
  {
    planning_group_ = get_param<std::string>(node_, "planning_group", "arm");
    tip_link_ = get_param<std::string>(node_, "tip_link", "gripper_tcp");
    frame_id_ = get_param<std::string>(node_, "frame_id", "ARM_BASE_LINK");

    input_point_topic_ = get_param<std::string>(node_, "input_point_topic", "/vlm_robot_tf/grasp_point");
    joint_state_topic_ = get_param<std::string>(node_, "joint_state_topic", "/joint_states");

    selected_grasp_topic_ = get_param<std::string>(node_, "selected_grasp_topic", "/vlm_moveit/selected_grasp_pose");
    selected_pregrasp_topic_ = get_param<std::string>(node_, "selected_pregrasp_topic", "/vlm_moveit/selected_pregrasp_pose");
    marker_topic_ = get_param<std::string>(node_, "marker_topic", "/vlm_moveit/grasp_candidate_markers");
    display_topic_ = get_param<std::string>(node_, "display_topic", "/display_planned_path");

    planning_time_ = get_param<double>(node_, "planning_time", 5.0);
    planning_attempts_ = get_param<int>(node_, "planning_attempts", 10);
    goal_position_tolerance_ = get_param<double>(node_, "goal_position_tolerance", 0.025);
    goal_orientation_tolerance_ = get_param<double>(node_, "goal_orientation_tolerance", 0.60);
    velocity_scale_ = get_param<double>(node_, "velocity_scale", 0.15);
    acceleration_scale_ = get_param<double>(node_, "acceleration_scale", 0.15);

    ik_timeout_ = get_param<double>(node_, "ik_timeout", 0.03);
    ik_attempts_ = get_param<int>(node_, "ik_attempts", 4);

    eef_step_ = get_param<double>(node_, "eef_step", 0.005);
    jump_threshold_ = get_param<double>(node_, "jump_threshold", 0.0);
    min_cartesian_fraction_ = get_param<double>(node_, "min_cartesian_fraction", 0.70);
    allow_regular_grasp_fallback_ = get_param<bool>(node_, "allow_regular_grasp_fallback", true);

    max_candidates_to_plan_ = get_param<int>(node_, "max_candidates_to_plan", 200);
    plan_once_ = get_param<bool>(node_, "plan_once", true);
    avoid_collisions_ = get_param<bool>(node_, "avoid_collisions", true);
    require_current_state_ = get_param<bool>(node_, "require_current_state", true);

    low_z_threshold_ = get_param<double>(node_, "low_z_threshold", 0.12);
    mid_z_threshold_ = get_param<double>(node_, "mid_z_threshold", 0.22);

    side_base_qx_ = get_param<double>(node_, "side_base_qx", 1.000);
    side_base_qy_ = get_param<double>(node_, "side_base_qy", 0.000);
    side_base_qz_ = get_param<double>(node_, "side_base_qz", -0.004);
    side_base_qw_ = get_param<double>(node_, "side_base_qw", -0.002);
    side_base_q_ = Eigen::Quaterniond(side_base_qw_, side_base_qx_, side_base_qy_, side_base_qz_);
    side_base_q_.normalize();

    gripper_top_local_x_ = get_param<double>(node_, "gripper_top_local_x", 0.0);
    gripper_top_local_y_ = get_param<double>(node_, "gripper_top_local_y", 0.0);
    gripper_top_local_z_ = get_param<double>(node_, "gripper_top_local_z", -1.0);
    gripper_top_local_ = Eigen::Vector3d(gripper_top_local_x_, gripper_top_local_y_, gripper_top_local_z_);
    if (gripper_top_local_.norm() < 1e-9) {
      gripper_top_local_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    }
    gripper_top_local_.normalize();

    joint4_weight_ = get_param<double>(node_, "joint4_weight", 3.0);
    joint5_weight_ = get_param<double>(node_, "joint5_weight", 3.0);
    joint6_weight_ = get_param<double>(node_, "joint6_weight", 10.0);
    other_joint_weight_ = get_param<double>(node_, "other_joint_weight", 1.0);

    motion_score_weight_ = get_param<double>(node_, "motion_score_weight", 1000.0);
    top_score_weight_ = get_param<double>(node_, "top_score_weight", 10.0);
    regular_fallback_penalty_ = get_param<double>(node_, "regular_fallback_penalty", 5.0);
    cartesian_fraction_penalty_ = get_param<double>(node_, "cartesian_fraction_penalty", 2.0);

    move_group_->setEndEffectorLink(tip_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(planning_attempts_);
    move_group_->setGoalPositionTolerance(goal_position_tolerance_);
    move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_);
    move_group_->setMaxVelocityScalingFactor(velocity_scale_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scale_);

    robot_model_ = move_group_->getRobotModel();
    jmg_ = robot_model_->getJointModelGroup(planning_group_);

    if (!jmg_) {
      throw std::runtime_error("JointModelGroup not found: " + planning_group_);
    }

    selected_grasp_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      selected_grasp_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    selected_pregrasp_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      selected_pregrasp_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      display_topic_,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    auto qos_volatile = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto qos_transient = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_,
      rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->latest_joint_state_ = *msg;
        this->have_joint_state_ = true;
      });

    sub_volatile_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      input_point_topic_,
      qos_volatile,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        this->on_point(msg);
      });

    sub_transient_ = node_->create_subscription<geometry_msgs::msg::PointStamped>(
      input_point_topic_,
      qos_transient,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        this->on_point(msg);
      });

    RCLCPP_INFO(node_->get_logger(), "Grasp candidate planner ready");
    RCLCPP_INFO(node_->get_logger(), "joint state topic: %s", joint_state_topic_.c_str());
    RCLCPP_INFO(node_->get_logger(), "selection rule: strict side grasp, roll fixed to 0, then minimum joint motion");
    RCLCPP_INFO(node_->get_logger(), "side base quaternion xyzw=(%.4f %.4f %.4f %.4f)",
      side_base_qx_, side_base_qy_, side_base_qz_, side_base_qw_);
    RCLCPP_INFO(node_->get_logger(), "gripper top local axis=(%.2f %.2f %.2f)",
      gripper_top_local_.x(), gripper_top_local_.y(), gripper_top_local_.z());
    RCLCPP_INFO(node_->get_logger(), "joint weights: J4=%.1f J5=%.1f J6=%.1f other=%.1f",
      joint4_weight_, joint5_weight_, joint6_weight_, other_joint_weight_);
    RCLCPP_INFO(node_->get_logger(), "max candidates to plan=%d", max_candidates_to_plan_);
  }

private:
  std::vector<double> side_rolls() const
  {
    // For side grasp demo, keep gripper plates parallel to object sides.
    // Do not allow wrist roll candidates such as +/-45 deg.
    return {0};
  }

  std::vector<double> top_rolls() const
  {
    return {0, 45, 90, 135, 180, 225, 270, 315};
  }

  std::vector<double> distances() const
  {
    return {0.06, 0.08, 0.10, 0.12};
  }

  std::vector<Candidate> generate_candidates(const Eigen::Vector3d & object_p)
  {
    std::vector<Candidate> out;
    int id = 0;

    for (double roll_deg : side_rolls()) {
      Eigen::Quaterniond q = apply_local_x_roll_offset(side_base_q_, deg2rad(roll_deg));

      for (double d : distances()) {
        Candidate c;
        c.id = id++;
        c.strategy = "side";
        c.roll_deg = roll_deg;
        c.pregrasp_dist = d;
        c.approach_dir = Eigen::Vector3d(1.0, 0.0, 0.0);

        Eigen::Vector3d grasp_p = object_p;
        Eigen::Vector3d pregrasp_p = grasp_p - c.approach_dir * d;

        c.grasp = make_pose(frame_id_, grasp_p, q);
        c.pregrasp = make_pose(frame_id_, pregrasp_p, q);
        out.push_back(c);
      }
    }

    auto add_candidate_set =
      [&](const std::string & strategy,
          const Eigen::Vector3d & approach_dir,
          const std::vector<double> & rolls_deg,
          const std::vector<double> & dists) {
        for (double roll_deg : rolls_deg) {
          Eigen::Quaterniond q = make_quat_from_x_axis_and_roll(approach_dir, deg2rad(roll_deg));

          for (double d : dists) {
            Candidate c;
            c.id = id++;
            c.strategy = strategy;
            c.roll_deg = roll_deg;
            c.pregrasp_dist = d;
            c.approach_dir = approach_dir.normalized();

            Eigen::Vector3d grasp_p = object_p;
            Eigen::Vector3d pregrasp_p = grasp_p - c.approach_dir * d;

            c.grasp = make_pose(frame_id_, grasp_p, q);
            c.pregrasp = make_pose(frame_id_, pregrasp_p, q);
            out.push_back(c);
          }
        }
      };

    // Disabled for strict side grasp parallel demo.
    // Top-down will be re-enabled later in a separate auto strategy mode.
    /*
    add_candidate_set(
      "top_down",
      Eigen::Vector3d(0.0, 0.0, -1.0),
      top_rolls(),
      distances());
    */

    // Disabled for strict side grasp parallel demo.
    // Angled grasp can tilt the gripper relative to the object sides.
    /*
    add_candidate_set(
      "angled",
      Eigen::Vector3d(1.0, 0.0, -0.45).normalized(),
      side_rolls(),
      {0.06, 0.08, 0.10});
    */

    return out;
  }

  bool check_ik(
    const geometry_msgs::msg::PoseStamped & pose,
    const moveit::core::RobotState & seed_state)
  {
    Eigen::Isometry3d target = pose_to_eigen(pose);

    for (int a = 0; a < ik_attempts_; ++a) {
      moveit::core::RobotState st(seed_state);

      if (a > 0) {
        st.setToRandomPositions(jmg_);
        st.update();
      }

      bool ok = st.setFromIK(jmg_, target, tip_link_, ik_timeout_);
      if (ok) {
        return true;
      }
    }

    return false;
  }

  bool get_seed_state(moveit::core::RobotState & seed_out)
  {
    auto current_state = move_group_->getCurrentState(0.5);

    if (current_state) {
      seed_out = *current_state;
      RCLCPP_INFO(node_->get_logger(), "using MoveIt current robot state as candidate seed");
      return true;
    }

    if (have_joint_state_) {
      seed_out.setToDefaultValues();

      std::set<std::string> valid_names;
      for (const auto & name : jmg_->getVariableNames()) {
        valid_names.insert(name);
      }

      int used_count = 0;

      for (size_t i = 0; i < latest_joint_state_.name.size() && i < latest_joint_state_.position.size(); ++i) {
        const std::string & name = latest_joint_state_.name[i];

        if (valid_names.count(name) == 0) {
          continue;
        }

        seed_out.setVariablePosition(name, latest_joint_state_.position[i]);
        used_count++;
      }

      seed_out.update();

      if (used_count > 0) {
        RCLCPP_WARN(
          node_->get_logger(),
          "MoveIt current state timestamp was not usable. Using direct /joint_states seed instead. used_joints=%d",
          used_count);
        return true;
      }
    }

    if (require_current_state_) {
      RCLCPP_ERROR(
        node_->get_logger(),
        "current robot state is required but no usable joint state was received. Check /joint_states.");
      return false;
    }

    seed_out.setToDefaultValues();
    seed_out.update();
    RCLCPP_WARN(node_->get_logger(), "current state unavailable. using default state");
    return true;
  }

  bool plan_candidate(
    const Candidate & c,
    const moveit::core::RobotState & seed_state,
    moveit::planning_interface::MoveGroupInterface::Plan & plan1,
    moveit_msgs::msg::RobotTrajectory & segment2,
    bool & used_cartesian,
    double & cartesian_fraction)
  {
    used_cartesian = false;
    cartesian_fraction = 0.0;

    move_group_->clearPoseTargets();
    move_group_->setStartState(seed_state);
    move_group_->setPoseTarget(c.pregrasp, tip_link_);

    bool plan1_ok = static_cast<bool>(move_group_->plan(plan1));
    if (!plan1_ok) {
      return false;
    }

    robot_trajectory::RobotTrajectory rt(robot_model_, planning_group_);
    rt.setRobotTrajectoryMsg(seed_state, plan1.trajectory_);

    if (rt.getWayPointCount() == 0) {
      return false;
    }

    const moveit::core::RobotState & last_state = rt.getLastWayPoint();

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(c.grasp.pose);

    move_group_->clearPoseTargets();
    move_group_->setStartState(last_state);

    cartesian_fraction = move_group_->computeCartesianPath(
      waypoints,
      eef_step_,
      jump_threshold_,
      segment2,
      avoid_collisions_);

    if (cartesian_fraction >= min_cartesian_fraction_) {
      used_cartesian = true;
      return true;
    }

    if (!allow_regular_grasp_fallback_) {
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan2;
    move_group_->clearPoseTargets();
    move_group_->setStartState(last_state);
    move_group_->setPoseTarget(c.grasp, tip_link_);

    bool plan2_ok = static_cast<bool>(move_group_->plan(plan2));

    if (plan2_ok) {
      segment2 = plan2.trajectory_;
      used_cartesian = false;
      return true;
    }

    return false;
  }

  double joint_weight(const std::string & name) const
  {
    if (name.find("JOINT6") != std::string::npos) {
      return joint6_weight_;
    }
    if (name.find("JOINT5") != std::string::npos) {
      return joint5_weight_;
    }
    if (name.find("JOINT4") != std::string::npos) {
      return joint4_weight_;
    }
    return other_joint_weight_;
  }

  double trajectory_motion_cost(
    const moveit::core::RobotState & start_state,
    const moveit_msgs::msg::RobotTrajectory & a,
    const moveit_msgs::msg::RobotTrajectory & b) const
  {
    std::map<std::string, double> prev;

    for (const auto & name : jmg_->getVariableNames()) {
      prev[name] = start_state.getVariablePosition(name);
    }

    auto accumulate = [&](const moveit_msgs::msg::RobotTrajectory & traj) {
      double cost = 0.0;

      const auto & jt = traj.joint_trajectory;
      for (const auto & pt : jt.points) {
        for (size_t i = 0; i < jt.joint_names.size() && i < pt.positions.size(); ++i) {
          const std::string & name = jt.joint_names[i];
          double now = pt.positions[i];

          if (prev.count(name)) {
            double delta = std::abs(now - prev[name]);
            cost += joint_weight(name) * delta;
          }

          prev[name] = now;
        }
      }

      return cost;
    };

    double cost = 0.0;
    cost += accumulate(a);
    cost += accumulate(b);
    return cost;
  }

  double gripper_top_error_deg(const Candidate & c) const
  {
    Eigen::Quaterniond q(
      c.grasp.pose.orientation.w,
      c.grasp.pose.orientation.x,
      c.grasp.pose.orientation.y,
      c.grasp.pose.orientation.z);
    q.normalize();

    Eigen::Vector3d top_world = (q * gripper_top_local_).normalized();
    Eigen::Vector3d desired_up(0.0, 0.0, 1.0);

    double dot = clamp_value(top_world.dot(desired_up), -1.0, 1.0);
    double err = std::acos(dot);
    return err * 180.0 / M_PI;
  }

  double compute_score(
    double motion_cost,
    double top_error_deg,
    bool used_cartesian,
    double fraction) const
  {
    double score = 0.0;
    score += motion_score_weight_ * motion_cost;
    score += top_score_weight_ * (top_error_deg / 180.0);
    score += used_cartesian ? 0.0 : regular_fallback_penalty_;
    score += cartesian_fraction_penalty_ * (1.0 - clamp_value(fraction, 0.0, 1.0));
    return score;
  }

  bool is_better(const CandidatePlan & a, const CandidatePlan & b) const
  {
    if (a.score < b.score) {
      return true;
    }
    return false;
  }

  void publish_selected(
    const CandidatePlan & best,
    const moveit::core::RobotState & seed_state,
    int total_candidates,
    int ik_ok_count,
    int planned_success_count,
    int checked_plan_count)
  {
    auto now = node_->now();

    auto grasp = best.candidate.grasp;
    auto pregrasp = best.candidate.pregrasp;
    grasp.header.stamp = now;
    pregrasp.header.stamp = now;

    selected_grasp_pub_->publish(grasp);
    selected_pregrasp_pub_->publish(pregrasp);

    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    Eigen::Vector3d gp(
      best.candidate.grasp.pose.position.x,
      best.candidate.grasp.pose.position.y,
      best.candidate.grasp.pose.position.z);

    Eigen::Vector3d pp(
      best.candidate.pregrasp.pose.position.x,
      best.candidate.pregrasp.pose.position.y,
      best.candidate.pregrasp.pose.position.z);

    Eigen::Quaterniond q(
      best.candidate.grasp.pose.orientation.w,
      best.candidate.grasp.pose.orientation.x,
      best.candidate.grasp.pose.orientation.y,
      best.candidate.grasp.pose.orientation.z);
    q.normalize();

    Eigen::Vector3d top_world = (q * gripper_top_local_).normalized();

    arr.markers.push_back(make_sphere(1, "selected_grasp", frame_id_, gp, 0.035, color(1.0f, 0.0f, 0.0f, 0.95f)));
    arr.markers.push_back(make_sphere(2, "selected_pregrasp", frame_id_, pp, 0.030, color(0.0f, 0.8f, 1.0f, 0.95f)));
    arr.markers.push_back(make_arrow(3, "selected_approach", frame_id_, pp, gp, 0.012, 0.030, color(0.0f, 0.2f, 1.0f, 0.95f)));
    arr.markers.push_back(make_arrow(4, "selected_gripper_top", frame_id_, gp, gp + top_world * 0.10, 0.010, 0.025, color(1.0f, 1.0f, 0.0f, 0.95f)));

    std::ostringstream ss;
    ss << "BEST "
       << best.candidate.strategy
       << " roll=" << std::fixed << std::setprecision(0) << best.candidate.roll_deg
       << " dist=" << std::setprecision(2) << best.candidate.pregrasp_dist
       << " motion=" << std::setprecision(3) << best.motion_cost
       << " top_err=" << std::setprecision(1) << best.top_error_deg
       << " cart=" << std::setprecision(2) << best.cartesian_fraction
       << " mode=" << (best.used_cartesian ? "cart" : "regular")
       << " ok=" << planned_success_count
       << "/" << checked_plan_count;

    arr.markers.push_back(make_text(
      5,
      "selected_text",
      frame_id_,
      gp + Eigen::Vector3d(0.0, 0.0, 0.09),
      ss.str(),
      color(1.0f, 1.0f, 1.0f, 0.95f)));

    for (auto & m : arr.markers) {
      m.header.stamp = now;
    }

    marker_pub_->publish(arr);

    moveit_msgs::msg::RobotState start_msg;
    moveit::core::robotStateToRobotStateMsg(seed_state, start_msg);

    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = robot_model_->getName();
    display.trajectory_start = start_msg;
    display.trajectory.push_back(best.plan1.trajectory_);
    display.trajectory.push_back(best.segment2);

    display_pub_->publish(display);

    RCLCPP_INFO(
      node_->get_logger(),
      "BEST SELECTED id=%d strategy=%s roll=%.0f dist=%.2f motion=%.4f top_error=%.2f score=%.4f mode=%s cart_fraction=%.3f total=%d ik_ok=%d checked_plan=%d success=%d",
      best.candidate.id,
      best.candidate.strategy.c_str(),
      best.candidate.roll_deg,
      best.candidate.pregrasp_dist,
      best.motion_cost,
      best.top_error_deg,
      best.score,
      best.used_cartesian ? "cartesian" : "regular",
      best.cartesian_fraction,
      total_candidates,
      ik_ok_count,
      checked_plan_count,
      planned_success_count);
  }

  void publish_failed(
    const Eigen::Vector3d & p,
    int total_candidates,
    int ik_ok_count,
    int checked_plan_count,
    int planned_success_count)
  {
    auto now = node_->now();

    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(del);

    arr.markers.push_back(make_sphere(
      1,
      "failed_object",
      frame_id_,
      p,
      0.04,
      color(1.0f, 0.0f, 0.0f, 0.95f)));

    std::ostringstream ss;
    ss << "NO PLAN total=" << total_candidates
       << " ik_ok=" << ik_ok_count
       << " checked=" << checked_plan_count
       << " success=" << planned_success_count;

    arr.markers.push_back(make_text(
      2,
      "failed_text",
      frame_id_,
      p + Eigen::Vector3d(0.0, 0.0, 0.08),
      ss.str(),
      color(1.0f, 0.2f, 0.2f, 0.95f)));

    for (auto & m : arr.markers) {
      m.header.stamp = now;
    }

    marker_pub_->publish(arr);
  }

  void on_point(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (planned_once_ && plan_once_) {
      return;
    }

    Eigen::Vector3d object_p(
      msg->point.x,
      msg->point.y,
      msg->point.z);

    RCLCPP_INFO(
      node_->get_logger(),
      "received object point in %s: x=%.3f y=%.3f z=%.3f",
      frame_id_.c_str(),
      object_p.x(),
      object_p.y(),
      object_p.z());

    moveit::core::RobotState seed_state(robot_model_);
    if (!get_seed_state(seed_state)) {
      planned_once_ = false;
      publish_failed(object_p, 0, 0, 0, 0);
      return;
    }

    planned_once_ = true;

    std::vector<Candidate> candidates = generate_candidates(object_p);

    int total_candidates = static_cast<int>(candidates.size());
    int ik_ok_count = 0;
    int checked_plan_count = 0;
    int planned_success_count = 0;

    bool has_best = false;
    CandidatePlan best;

    RCLCPP_INFO(node_->get_logger(), "generated %d grasp candidates. evaluating all candidates...", total_candidates);

    for (const auto & c : candidates) {
      bool pre_ik = check_ik(c.pregrasp, seed_state);
      bool grasp_ik = check_ik(c.grasp, seed_state);

      if (!pre_ik || !grasp_ik) {
        continue;
      }

      ik_ok_count++;

      if (checked_plan_count >= max_candidates_to_plan_) {
        continue;
      }

      checked_plan_count++;

      moveit::planning_interface::MoveGroupInterface::Plan plan1;
      moveit_msgs::msg::RobotTrajectory segment2;
      bool used_cartesian = false;
      double fraction = 0.0;

      bool ok = plan_candidate(c, seed_state, plan1, segment2, used_cartesian, fraction);

      if (!ok) {
        continue;
      }

      planned_success_count++;

      CandidatePlan result;
      result.candidate = c;
      result.plan1 = plan1;
      result.segment2 = segment2;
      result.used_cartesian = used_cartesian;
      result.cartesian_fraction = fraction;
      result.motion_cost = trajectory_motion_cost(seed_state, plan1.trajectory_, segment2);
      result.top_error_deg = gripper_top_error_deg(c);
      result.score = compute_score(result.motion_cost, result.top_error_deg, result.used_cartesian, result.cartesian_fraction);

      RCLCPP_INFO(
        node_->get_logger(),
        "SUCCESS candidate id=%d strategy=%s roll=%.0f dist=%.2f motion=%.4f top_error=%.2f score=%.4f mode=%s cart=%.3f",
        c.id,
        c.strategy.c_str(),
        c.roll_deg,
        c.pregrasp_dist,
        result.motion_cost,
        result.top_error_deg,
        result.score,
        used_cartesian ? "cartesian" : "regular",
        fraction);

      if (!has_best || is_better(result, best)) {
        best = result;
        has_best = true;
      }
    }

    RCLCPP_INFO(
      node_->get_logger(),
      "candidate search finished: total=%d ik_ok=%d checked_plan=%d success=%d",
      total_candidates,
      ik_ok_count,
      checked_plan_count,
      planned_success_count);

    if (!has_best) {
      RCLCPP_ERROR(node_->get_logger(), "NO GRASP PLAN FOUND");
      publish_failed(object_p, total_candidates, ik_ok_count, checked_plan_count, planned_success_count);
      return;
    }

    publish_selected(best, seed_state, total_candidates, ik_ok_count, planned_success_count, checked_plan_count);
  }

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::string planning_group_;
  std::string tip_link_;
  std::string frame_id_;

  std::string input_point_topic_;
  std::string joint_state_topic_;
  std::string selected_grasp_topic_;
  std::string selected_pregrasp_topic_;
  std::string marker_topic_;
  std::string display_topic_;

  double planning_time_;
  int planning_attempts_;
  double goal_position_tolerance_;
  double goal_orientation_tolerance_;
  double velocity_scale_;
  double acceleration_scale_;

  double ik_timeout_;
  int ik_attempts_;

  double eef_step_;
  double jump_threshold_;
  double min_cartesian_fraction_;
  bool allow_regular_grasp_fallback_;

  int max_candidates_to_plan_;
  bool plan_once_;
  bool avoid_collisions_;
  bool require_current_state_;

  double low_z_threshold_;
  double mid_z_threshold_;

  double side_base_qx_;
  double side_base_qy_;
  double side_base_qz_;
  double side_base_qw_;
  Eigen::Quaterniond side_base_q_ = Eigen::Quaterniond::Identity();

  double gripper_top_local_x_;
  double gripper_top_local_y_;
  double gripper_top_local_z_;
  Eigen::Vector3d gripper_top_local_ = Eigen::Vector3d(0.0, 0.0, -1.0);

  double joint4_weight_;
  double joint5_weight_;
  double joint6_weight_;
  double other_joint_weight_;

  double motion_score_weight_;
  double top_score_weight_;
  double regular_fallback_penalty_;
  double cartesian_fraction_penalty_;

  bool planned_once_ = false;

  moveit::core::RobotModelConstPtr robot_model_;
  const moveit::core::JointModelGroup * jmg_ = nullptr;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selected_grasp_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selected_pregrasp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;

  sensor_msgs::msg::JointState latest_joint_state_;
  bool have_joint_state_ = false;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_volatile_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_transient_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  options.automatically_declare_parameters_from_overrides(true);

  auto node = std::make_shared<rclcpp::Node>(
    "moveit_grasp_candidate_plan_cpp_node",
    options);

  std::string planning_group = get_param<std::string>(node, "planning_group", "arm");

  auto move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(
      node,
      planning_group);

  try {
    auto planner = std::make_shared<GraspCandidatePlanner>(node, move_group);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}
