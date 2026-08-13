// Copyright 2026 Gautham. Apache-2.0.
//
// See fleet_trajectory_layer.hpp for what this layer is and why it is a layer
// rather than a central planner.

#include "amr_costmap_plugins/fleet_trajectory_layer.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/LinearMath/Transform.h"
#include "tf2/LinearMath/Vector3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"

namespace amr_costmap_plugins
{

FleetTrajectoryLayer::FleetTrajectoryLayer() {}

void FleetTrajectoryLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"FleetTrajectoryLayer: owning node is gone"};
  }
  logger_ = node->get_logger();
  clock_ = node->get_clock();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  // Rendered per robot by amr_navigation.params as "every robot in fleet.yaml
  // except me". The exclusion lives in config generation, which is what keeps
  // this file free of any robot name (ENGINEERING_NOTES rule 5).
  declareParameter("trajectory_topics", rclcpp::ParameterValue(std::vector<std::string>{}));
  declareParameter("horizon_s", rclcpp::ParameterValue(6.0));
  declareParameter("decay_tau_s", rclcpp::ParameterValue(3.0));
  declareParameter("radius_m", rclcpp::ParameterValue(0.45));
  declareParameter("max_cost", rclcpp::ParameterValue(240));
  declareParameter("stale_after_s", rclcpp::ParameterValue(2.0));
  declareParameter("transform_tolerance_s", rclcpp::ParameterValue(0.2));

  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("trajectory_topics"), trajectory_topics_);
  node->get_parameter(getFullName("horizon_s"), horizon_s_);
  node->get_parameter(getFullName("decay_tau_s"), decay_tau_s_);
  node->get_parameter(getFullName("radius_m"), radius_m_);
  node->get_parameter(getFullName("stale_after_s"), stale_after_s_);
  node->get_parameter(getFullName("transform_tolerance_s"), transform_tolerance_s_);

  int max_cost = 240;
  node->get_parameter(getFullName("max_cost"), max_cost);
  // RAISE, DO NOT CLIP. A configuration that asks for a lethal trajectory cost is
  // asking for two robots to deadlock on each other's intentions; silently
  // clamping it would hide the mistake until a demo. amr_navigation.ramp_mask
  // takes the same position on filter mask values, for the same reason.
  if (max_cost < 1 || max_cost > static_cast<int>(kMaxAllowedCost)) {
    throw std::runtime_error{
            "FleetTrajectoryLayer: max_cost must be in [1, " +
            std::to_string(static_cast<int>(kMaxAllowedCost)) + "]; got " +
            std::to_string(max_cost) +
            ". 253 is INSCRIBED_INFLATED_OBSTACLE and 254 is LETHAL_OBSTACLE, both of "
            "which every footprint collision checker treats as a collision. A peer's "
            "predicted path must be expensive, not impassable."};
  }
  max_cost_ = static_cast<unsigned char>(max_cost);

  if (decay_tau_s_ <= 0.0) {
    throw std::runtime_error{"FleetTrajectoryLayer: decay_tau_s must be > 0"};
  }

  rclcpp::SubscriptionOptions options;
  // Costmap2DROS runs a dedicated executor over this callback group. A
  // subscription created without it is never spun and the layer sees nothing,
  // silently.
  options.callback_group = callback_group_;

  for (const auto & topic : trajectory_topics_) {
    subscriptions_.push_back(
      node->create_subscription<nav_msgs::msg::Path>(
        topic, rclcpp::SystemDefaultsQoS(),
        [this, topic](const nav_msgs::msg::Path::SharedPtr msg) {
          this->onTrajectory(msg, topic);
        },
        options));
  }

  RCLCPP_INFO(
    logger_,
    "FleetTrajectoryLayer '%s': %zu peer trajectory topic(s), horizon %.1f s, "
    "tau %.1f s, radius %.2f m, max_cost %d",
    name_.c_str(), trajectory_topics_.size(), horizon_s_, decay_tau_s_, radius_m_,
    static_cast<int>(max_cost_));
  if (trajectory_topics_.empty()) {
    RCLCPP_WARN(
      logger_,
      "FleetTrajectoryLayer '%s' has no peer topics and will contribute nothing. "
      "For a single-robot run that is correct; for a fleet run it means "
      "trajectory_topics was not rendered.",
      name_.c_str());
  }

  current_ = true;
}

void FleetTrajectoryLayer::onTrajectory(
  const nav_msgs::msg::Path::SharedPtr msg, const std::string & topic)
{
  std::lock_guard<std::mutex> lock(mutex_);
  trajectories_[topic] = *msg;
}

unsigned char FleetTrajectoryLayer::decayedCost(double dt) const
{
  if (dt > horizon_s_) {
    return 0;
  }
  // A pose whose predicted time has already passed is treated as "now" rather
  // than discarded: the peer is at worst slightly behind its own prediction, and
  // the cell it is standing in is the last one we want to cheapen.
  const double clamped = std::max(0.0, dt);
  const double scaled = static_cast<double>(max_cost_) * std::exp(-clamped / decay_tau_s_);
  return static_cast<unsigned char>(std::lround(std::min(scaled, static_cast<double>(max_cost_))));
}

void FleetTrajectoryLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<std::mutex> lock(mutex_);
  stamps_.clear();
  if (!enabled_) {
    current_ = true;
    return;
  }

  const rclcpp::Time now = clock_->now();
  const std::string target_frame = layered_costmap_->getGlobalFrameID();
  bool warned_this_cycle = false;

  for (const auto & entry : trajectories_) {
    const nav_msgs::msg::Path & path = entry.second;
    if (path.poses.empty()) {
      continue;
    }
    // A peer that has stopped publishing must stop costing. Without this a robot
    // that dies mid-aisle leaves a permanent phantom corridor its peers refuse
    // to enter -- the same failure the obstacle layer's raytrace clearing exists
    // to prevent, one level up.
    const double age = (now - rclcpp::Time(path.header.stamp)).seconds();
    if (age > stale_after_s_) {
      continue;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      // tf2::TimePointZero -- LATEST AVAILABLE, deliberately, and this is the
      // subtle part. Every pose stamp in this message is a predicted ARRIVAL
      // TIME in the future; asking TF to resolve a frame at a future time either
      // blocks until timeout or extrapolates. The transform we want is the
      // current relationship between the fleet frame and this costmap's frame,
      // which is what TimePointZero returns.
      tf = tf_->lookupTransform(target_frame, path.header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      if (!transform_warned_) {
        RCLCPP_WARN(
          logger_, "FleetTrajectoryLayer '%s': no transform %s -> %s (%s). "
          "Peer trajectories are not being costed.",
          name_.c_str(), path.header.frame_id.c_str(), target_frame.c_str(), ex.what());
        transform_warned_ = true;
      }
      warned_this_cycle = true;
      continue;
    }

    tf2::Transform transform;
    tf2::fromMsg(tf.transform, transform);

    for (const auto & pose : path.poses) {
      const double dt = (rclcpp::Time(pose.header.stamp) - now).seconds();
      const unsigned char cost = decayedCost(dt);
      if (cost == 0) {
        continue;
      }
      const tf2::Vector3 point = transform * tf2::Vector3(
        pose.pose.position.x, pose.pose.position.y, 0.0);

      stamps_.push_back(TrajectoryStamp{point.x(), point.y(), dt, cost});

      *min_x = std::min(*min_x, point.x() - radius_m_);
      *min_y = std::min(*min_y, point.y() - radius_m_);
      *max_x = std::max(*max_x, point.x() + radius_m_);
      *max_y = std::max(*max_y, point.y() + radius_m_);
    }
  }

  if (!warned_this_cycle) {
    transform_warned_ = false;
  }
  current_ = true;
}

void FleetTrajectoryLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (stamps_.empty()) {
    return;
  }

  unsigned char * master = master_grid.getCharMap();
  const double resolution = master_grid.getResolution();
  const int cell_radius = static_cast<int>(std::ceil(radius_m_ / resolution));
  const int radius_sq = cell_radius * cell_radius;

  for (const auto & stamp : stamps_) {
    unsigned int centre_x = 0;
    unsigned int centre_y = 0;
    if (!master_grid.worldToMap(stamp.x, stamp.y, centre_x, centre_y)) {
      continue;  // Outside this rolling window; the peer is simply too far away.
    }
    for (int dj = -cell_radius; dj <= cell_radius; ++dj) {
      for (int di = -cell_radius; di <= cell_radius; ++di) {
        if (di * di + dj * dj > radius_sq) {
          continue;
        }
        const int i = static_cast<int>(centre_x) + di;
        const int j = static_cast<int>(centre_y) + dj;
        if (i < min_i || i >= max_i || j < min_j || j >= max_j) {
          continue;
        }
        const unsigned int index = master_grid.getIndex(
          static_cast<unsigned int>(i), static_cast<unsigned int>(j));
        const unsigned char existing = master[index];
        if (existing == nav2_costmap_2d::NO_INFORMATION) {
          // std::max would keep 255 and the deposit would vanish on unknown
          // cells. Overwriting instead is the same rule nav2's KeepoutFilter
          // applies (`data > old_data || old_data == NO_INFORMATION`).
          master[index] = stamp.cost;
        } else {
          master[index] = std::max(existing, stamp.cost);
        }
      }
    }
  }
}

void FleetTrajectoryLayer::reset()
{
  std::lock_guard<std::mutex> lock(mutex_);
  trajectories_.clear();
  stamps_.clear();
  current_ = false;
}

}  // namespace amr_costmap_plugins

PLUGINLIB_EXPORT_CLASS(amr_costmap_plugins::FleetTrajectoryLayer, nav2_costmap_2d::Layer)
