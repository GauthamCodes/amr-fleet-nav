// Copyright 2026 Gautham. Apache-2.0.
//
// SafetyGate: the low-latency safety override.
//
// WHY THIS IS A SERIAL GATE AND NOT A twist_mux INPUT (ENGINEERING_NOTES rule 1)
//   A mux arbitrates between sources. If the arbiter dies, the highest-priority
//   source that is still alive wins - which means a dead safety node lets ordinary
//   navigation commands straight through. That is failing OPEN, and it is the wrong
//   failure mode for the one component whose entire job is to stop the robot. This
//   node is instead the last link in the chain: nothing else publishes the topic the
//   plant listens to, so a command can only reach the wheels by passing through here.
//
//   Simulation caveat, stated rather than implied: gz-sim 8.11.0's DiffDrive has no
//   command timeout, so if this process is SIGKILLed the plant keeps rolling on the
//   last velocity it latched. Real hardware does not behave that way - a motor
//   controller stops when its command stream stops - so amr_gazebo's drive_watchdog
//   models exactly that, sitting on the plant side of the boundary. The clean-shutdown
//   zero below is best effort and is not what the fail-closed claim rests on.
//
// WHY THE VELOCITY COMES FROM ODOMETRY (rule 3)
//   Commanded velocity is what we WANT; the braking distance depends on what the
//   robot is actually DOING. On a ramp, under a 60 kg payload, or during the ramp-up
//   after a release, those differ - and they differ most exactly when the gate matters.
//
// WHY BLOCKING ALSO WRITES A NAV2 PARAMETER (rule 2)
//   Zeroing cmd_vel without telling Nav2 makes the robot look stuck to the progress
//   checker, which fires recovery behaviours - spin-in-place, in front of whatever we
//   just stopped for. On entry the gate raises
//   controller_server's progress_checker.movement_time_allowance and restores it on
//   release, after the robot has actually moved.
//
// LATENCY
//   Every decision carries the scan stamp it was derived from, so the reported
//   figure is sensor-stamp-to-zero-command-published and not an internal timer.
//   Both a ROS-clock end-to-end figure and a steady-clock compute figure are
//   published; under use_sim_time the former is quantised by Gazebo's /clock step
//   and the latter is not. "Low-latency safety override" - not "hard real-time".

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "amr_safety/footprint.hpp"
#include "amr_safety/safety_model.hpp"

namespace amr_safety
{

namespace
{
/// Below this commanded speed there is no translation to stop, in m/s.
constexpr double kCommandEpsilon = 1e-3;
/// The Nav2 parameter that decides how long a stationary robot is tolerated.
constexpr char kAllowanceParameter[] = "progress_checker.movement_time_allowance";
}  // namespace

/// The serial, fail-closed gate that is the last link before the robot's cmd_vel.
class SafetyGate : public rclcpp::Node
{
public:
  /// Declares every parameter, wires the streams, and starts the watchdog timer.
  SafetyGate()
  : rclcpp::Node("safety_gate"),
    latch_(declare_parameter<double>("release_margin", 0.15),
      declare_parameter<double>("min_hold_s", 0.30))
  {
    k_ = declare_parameter<double>("k", 1.875);
    d_min_ = declare_parameter<double>("d_min", 0.30);
    sector_half_angle_ = declare_parameter<double>("sector_half_angle", M_PI / 2.0);
    self_return_margin_ = declare_parameter<double>("self_return_margin", 0.01);
    cmd_timeout_s_ = declare_parameter<double>("cmd_timeout_s", 0.5);
    scan_timeout_s_ = declare_parameter<double>("scan_timeout_s", 0.5);
    laser_x_ = declare_parameter<double>("laser_x", 0.0);
    laser_y_ = declare_parameter<double>("laser_y", 0.0);
    laser_yaw_ = declare_parameter<double>("laser_yaw", 0.0);

    suppress_recovery_ = declare_parameter<bool>("suppress_recovery", true);
    recovery_allowance_s_ = declare_parameter<double>("recovery_allowance_s", 1.0e6);
    restore_grace_s_ = declare_parameter<double>("restore_grace_s", 2.0);
    restore_movement_radius_ =
      declare_parameter<double>("restore_movement_radius", 0.5);

    const auto footprint_x = declare_parameter<std::vector<double>>(
      "footprint_x", std::vector<double>{0.49, 0.49, -0.21, -0.21});
    const auto footprint_y = declare_parameter<std::vector<double>>(
      "footprint_y", std::vector<double>{0.225, -0.225, -0.225, 0.225});
    if (footprint_x.size() != footprint_y.size() || footprint_x.size() < 3) {
      throw std::runtime_error("footprint_x and footprint_y must pair into >=3 vertices");
    }
    for (size_t i = 0; i < footprint_x.size(); ++i) {
      footprint_.push_back(Point{footprint_x[i], footprint_y[i]});
    }

    const auto scan_topic = declare_parameter<std::string>("scan_topic", "validated/scan");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    const auto cmd_in_topic = declare_parameter<std::string>("cmd_in_topic", "cmd_vel_in");
    const auto cmd_out_topic = declare_parameter<std::string>("cmd_out_topic", "cmd_vel");
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 20.0);

    // Depth 1 on both command topics: a stale command is worthless to a gate, and
    // queueing them would let a backlog outlive the condition that produced it.
    command_publisher_ = create_publisher<geometry_msgs::msg::Twist>(cmd_out_topic, 1);
    diagnostics_publisher_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "safety_gate/diagnostics", 10);

    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, rclcpp::QoS(5).reliable(),
      std::bind(&SafetyGate::OnScan, this, std::placeholders::_1));
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::QoS(10).reliable(),
      std::bind(&SafetyGate::OnOdometry, this, std::placeholders::_1));
    command_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_in_topic, 1, std::bind(&SafetyGate::OnCommand, this, std::placeholders::_1));

    max_vel_x_ = declare_parameter<double>("max_vel_x", 0.6);

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SafetyGate::OnTimer, this));

    auto controller_name = declare_parameter<std::string>("controller_server_name", "");
    if (controller_name.empty()) {
      const std::string ns = get_namespace();
      controller_name = (ns == "/") ? "/controller_server" : ns + "/controller_server";
    }
    controller_name_ = controller_name;
    if (suppress_recovery_) {
      recovery_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
        this, controller_name_);
    }

    RCLCPP_INFO(
      get_logger(),
      "SafetyGate up: k=%.4f s^2/m, d_min=%.3f m, d_safe(max_vel)=%.3f m, "
      "release margin %.3f m, %s -> %s, recovery suppression %s via %s",
      k_, d_min_, DSafe(k_, d_min_, max_vel_x_),
      latch_.release_margin(), cmd_in_topic.c_str(), cmd_out_topic.c_str(),
      suppress_recovery_ ? "ON" : "OFF", controller_name_.c_str());
  }

  /// Publishes a single zero command. Best effort, for clean shutdown only.
  void PublishStop()
  {
    if (command_publisher_) {
      command_publisher_->publish(geometry_msgs::msg::Twist());
    }
  }

private:
  void OnOdometry(nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    // MEASURED, never commanded. See this file's header.
    speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    odom_x_ = msg->pose.pose.position.x;
    odom_y_ = msg->pose.pose.position.y;
    have_odometry_ = true;
  }

  void OnCommand(geometry_msgs::msg::Twist::ConstSharedPtr msg)
  {
    command_ = *msg;
    last_command_time_ = now();
    have_command_ = true;
    const bool blocked = Evaluate();
    PublishDecision(blocked);
  }

  void OnScan(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
  {
    const auto compute_started = std::chrono::steady_clock::now();
    MeasureClearances(*msg);
    last_scan_time_ = now();
    last_scan_stamp_ = rclcpp::Time(msg->header.stamp);
    have_scan_ = true;

    const bool was_blocked = latch_.blocked();
    const bool blocked = Evaluate();
    transition_ = blocked && !was_blocked;

    // The low-latency path the metric measures: the zero goes out on the scan that
    // caused it, not on the next command tick.
    double sensor_to_zero_ms = -1.0;
    if (blocked) {
      PublishDecision(true);
      sensor_to_zero_ms = (now() - last_scan_stamp_).seconds() * 1000.0;
    }
    const auto compute_ended = std::chrono::steady_clock::now();
    const double compute_us =
      std::chrono::duration<double, std::micro>(compute_ended - compute_started).count();

    if (transition_) {
      RCLCPP_WARN(
        get_logger(),
        "HALT: clearance %.3f m <= d_safe %.3f m at measured %.3f m/s "
        "(sensor->zero %.1f ms)",
        UsedClearance(), DSafe(k_, d_min_, speed_), speed_, sensor_to_zero_ms);
    } else if (!blocked && released_) {
      // latch_clearance_, NOT UsedClearance(). They differ exactly when the
      // release was caused by the upstream ceasing to command translation, in
      // which case the latch saw infinity while the sector still held a near
      // return - and printing the sector value there produced a log line that
      // read "released because clearance 0.121 m > 0.717 m". Report the number
      // the decision was actually made on, and name the other case.
      RCLCPP_INFO(
        get_logger(),
        "RELEASE: latch clearance %.3f m > d_release %.3f m (sector %.3f m, "
        "commanded vx %.3f m/s)",
        latch_clearance_,
        ReleaseDistance(DSafe(k_, d_min_, speed_), latch_.release_margin()),
        UsedClearance(), command_.linear.x);
    }
    PublishDiagnostics(sensor_to_zero_ms, compute_us);
    released_ = false;
  }

  void OnTimer()
  {
    const bool blocked = Evaluate();
    const bool command_stale =
      !have_command_ || (now() - last_command_time_).seconds() > cmd_timeout_s_;
    // A gate with nothing upstream must not leave the plant on a latched command.
    if (blocked || command_stale) {
      PublishDecision(true);
    }
    ServiceRecoveryRestore();
    CaptureOriginalAllowance();
  }

  /// Projects the scan into the base frame and reduces it to two sector minima.
  void MeasureClearances(const sensor_msgs::msg::LaserScan & scan)
  {
    forward_clearance_ = std::numeric_limits<double>::infinity();
    rear_clearance_ = std::numeric_limits<double>::infinity();
    self_returns_ = 0;
    const double cos_yaw = std::cos(laser_yaw_);
    const double sin_yaw = std::sin(laser_yaw_);
    const double rear_threshold = M_PI - sector_half_angle_;

    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
        continue;
      }
      const double angle = scan.angle_min + static_cast<double>(i) * scan.angle_increment;
      const double lx = range * std::cos(angle);
      const double ly = range * std::sin(angle);
      const double bx = laser_x_ + lx * cos_yaw - ly * sin_yaw;
      const double by = laser_y_ + lx * sin_yaw + ly * cos_yaw;

      const double distance = DistanceToPolygon(footprint_, bx, by);
      if (distance < self_return_margin_) {
        ++self_returns_;
        continue;
      }
      const double bearing = std::abs(std::atan2(by, bx));
      if (bearing <= sector_half_angle_) {
        forward_clearance_ = std::min(forward_clearance_, distance);
      }
      if (bearing >= rear_threshold) {
        rear_clearance_ = std::min(rear_clearance_, distance);
      }
    }
  }

  /// Returns the clearance in the sector the CURRENT command would drive into.
  double UsedClearance() const
  {
    return command_.linear.x < 0.0 ? rear_clearance_ : forward_clearance_;
  }

  /// Advances the latch and keeps the recovery suppression in step with it.
  ///
  /// The latch is clocked on ``now()`` and never on the scan stamp. The two differ
  /// by the transport delay, and mixing them would feed the dwell timer a clock
  /// that steps backwards whenever a command tick lands between two scans. The
  /// scan stamp is used for exactly one thing - the latency metric.
  bool Evaluate()
  {
    const double stop = DSafe(k_, d_min_, speed_);
    const double seconds = now().seconds();
    const bool scan_stale =
      !have_scan_ || (now() - last_scan_time_).seconds() > scan_timeout_s_;

    const bool was_blocked = latch_.blocked();
    bool blocked;
    if (scan_stale) {
      // Fail-closed: an unknown world is an occupied one. This is the behaviour a
      // mux cannot provide, and the reason the gate is serial.
      latch_.ForceBlocked(seconds);
      blocked = true;
      stale_ = true;
    } else {
      stale_ = false;
      const bool translating = std::abs(command_.linear.x) > kCommandEpsilon;
      // With no translation commanded there is nothing to stop, and latching would
      // suppress Nav2 recovery for a robot that is merely parked.
      const double clearance =
        translating ? UsedClearance() : std::numeric_limits<double>::infinity();
      latch_clearance_ = clearance;
      blocked = latch_.Update(clearance, stop, seconds);
    }
    if (was_blocked && !blocked) {
      released_ = true;
    }
    ApplyRecoverySuppression(blocked);
    return blocked;
  }

  void PublishDecision(bool blocked)
  {
    geometry_msgs::msg::Twist out;
    const bool command_stale =
      !have_command_ || (now() - last_command_time_).seconds() > cmd_timeout_s_;
    if (!blocked && !command_stale) {
      out = command_;
    }
    // THE INVARIANT, CHECKED WHERE IT CAN BE CHECKED EXACTLY.
    //
    // "no motion command left the gate while it was latched" is the single claim
    // this node exists to support, and an external observer cannot verify it: it
    // sees the latch state only through diagnostics, which are published once per
    // scan, so a command passed legitimately in the ~100 ms after a release looks
    // to it like a leak. Counting here compares the twist actually being published
    // against the latch state at that instant, with no window in between.
    if (latch_.blocked() && std::abs(out.linear.x) > kCommandEpsilon) {
      ++leaked_commands_;
      RCLCPP_ERROR(
        get_logger(), "GATE LEAK: published vx %.3f m/s while latched", out.linear.x);
    }
    command_publisher_->publish(out);
  }

  // ------------------------------------------------------------------ recovery

  void CaptureOriginalAllowance()
  {
    if (!recovery_client_ || allowance_captured_ || allowance_requested_) {
      return;
    }
    if (!recovery_client_->service_is_ready()) {
      return;
    }
    allowance_requested_ = true;
    recovery_client_->get_parameters(
      {kAllowanceParameter},
      [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        const auto values = future.get();
        if (!values.empty() &&
        values[0].get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
        {
          original_allowance_ = values[0].as_double();
          allowance_captured_ = true;
          RCLCPP_INFO(
            get_logger(), "captured %s = %.1f s from %s",
            kAllowanceParameter, original_allowance_, controller_name_.c_str());
        } else {
          // Declared explicitly in nav2_params.yaml precisely so this cannot happen;
          // saying so out loud beats silently running without recovery suppression.
          allowance_requested_ = false;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 10000,
            "%s is not a double on %s - recovery suppression is INACTIVE",
            kAllowanceParameter, controller_name_.c_str());
        }
      });
  }

  void ApplyRecoverySuppression(bool blocked)
  {
    if (!recovery_client_ || !allowance_captured_) {
      return;
    }
    if (blocked) {
      pending_restore_ = false;
      if (!recovery_suppressed_) {
        WriteAllowance(recovery_allowance_s_);
        recovery_suppressed_ = true;
        RCLCPP_INFO(
          get_logger(), "recovery suppressed: %s -> %.0f s",
          kAllowanceParameter, recovery_allowance_s_);
      }
    } else if (recovery_suppressed_ && !pending_restore_) {
      // Do NOT restore yet. The robot is still stationary at this instant, and
      // handing back a 10 s allowance before it has started moving fires the very
      // recovery this suppression exists to prevent.
      pending_restore_ = true;
      restore_deadline_ = now() + rclcpp::Duration::from_seconds(restore_grace_s_);
      restore_origin_x_ = odom_x_;
      restore_origin_y_ = odom_y_;
    }
  }

  void ServiceRecoveryRestore()
  {
    if (!pending_restore_ || !recovery_suppressed_) {
      return;
    }
    const double moved =
      have_odometry_ ? std::hypot(odom_x_ - restore_origin_x_, odom_y_ - restore_origin_y_)
      : 0.0;
    const bool moved_enough = moved >= restore_movement_radius_;
    if (moved_enough || now() >= restore_deadline_) {
      WriteAllowance(original_allowance_);
      recovery_suppressed_ = false;
      pending_restore_ = false;
      RCLCPP_INFO(
        get_logger(), "recovery restored: %s -> %.1f s after %.3f m of motion",
        kAllowanceParameter, original_allowance_, moved);
    }
  }

  void WriteAllowance(double value)
  {
    recovery_client_->set_parameters(
      {rclcpp::Parameter(kAllowanceParameter, value)},
      [this](std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>
      future) {
        const auto results = future.get();
        if (!results.empty() && !results[0].successful) {
          RCLCPP_ERROR(
            get_logger(), "failed to write %s: %s",
            kAllowanceParameter, results[0].reason.c_str());
        }
      });
  }

  // --------------------------------------------------------------- diagnostics

  void PublishDiagnostics(double sensor_to_zero_ms, double compute_us)
  {
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(get_namespace()) + " SafetyGate";
    status.hardware_id = get_namespace();
    const double stop = DSafe(k_, d_min_, speed_);
    if (stale_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "BLOCKED (sensor stale, fail-closed)";
    } else if (latch_.blocked()) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      status.message = "BLOCKED";
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "CLEAR";
    }

    const auto add = [&status](const std::string & key, double value) {
        diagnostic_msgs::msg::KeyValue pair;
        pair.key = key;
        pair.value = std::to_string(value);
        status.values.push_back(pair);
      };
    add("blocked", latch_.blocked() ? 1.0 : 0.0);
    add("transition", transition_ ? 1.0 : 0.0);
    add("stale", stale_ ? 1.0 : 0.0);
    add("clearance", UsedClearance());
    add("clearance_latched", latch_clearance_);
    add("clearance_forward", forward_clearance_);
    add("clearance_rear", rear_clearance_);
    add("speed_measured", speed_);
    add("d_stop", stop);
    add("d_release", ReleaseDistance(stop, latch_.release_margin()));
    add("commanded_vx", command_.linear.x);
    add("self_returns", static_cast<double>(self_returns_));
    add("leaked_commands", static_cast<double>(leaked_commands_));
    add("sensor_to_zero_ms", sensor_to_zero_ms);
    add("compute_us", compute_us);
    add("scan_stamp", last_scan_stamp_.seconds());
    add("recovery_suppressed", recovery_suppressed_ ? 1.0 : 0.0);
    add("k", k_);
    add("d_min", d_min_);

    array.status.push_back(std::move(status));
    diagnostics_publisher_->publish(array);
  }

  // ------------------------------------------------------------------- members

  double k_ {1.875};
  double d_min_ {0.30};
  double sector_half_angle_ {M_PI / 2.0};
  double self_return_margin_ {0.01};
  double cmd_timeout_s_ {0.5};
  double scan_timeout_s_ {0.5};
  double laser_x_ {0.0};
  double laser_y_ {0.0};
  double laser_yaw_ {0.0};
  double max_vel_x_ {0.6};
  std::vector<Point> footprint_;

  HysteresisLatch latch_;
  double speed_ {0.0};
  double odom_x_ {0.0};
  double odom_y_ {0.0};
  bool have_odometry_ {false};
  bool have_scan_ {false};
  bool have_command_ {false};
  bool stale_ {true};
  bool transition_ {false};
  bool released_ {false};
  double latch_clearance_ {std::numeric_limits<double>::infinity()};
  double forward_clearance_ {std::numeric_limits<double>::infinity()};
  double rear_clearance_ {std::numeric_limits<double>::infinity()};
  int self_returns_ {0};
  long leaked_commands_ {0};

  geometry_msgs::msg::Twist command_;
  rclcpp::Time last_command_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_time_ {0, 0, RCL_ROS_TIME};
  rclcpp::Time last_scan_stamp_ {0, 0, RCL_ROS_TIME};

  bool suppress_recovery_ {true};
  double recovery_allowance_s_ {1.0e6};
  double restore_grace_s_ {2.0};
  double restore_movement_radius_ {0.5};
  std::string controller_name_;
  std::shared_ptr<rclcpp::AsyncParametersClient> recovery_client_;
  bool allowance_requested_ {false};
  bool allowance_captured_ {false};
  double original_allowance_ {10.0};
  bool recovery_suppressed_ {false};
  bool pending_restore_ {false};
  rclcpp::Time restore_deadline_ {0, 0, RCL_ROS_TIME};
  double restore_origin_x_ {0.0};
  double restore_origin_y_ {0.0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
    diagnostics_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace amr_safety

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<amr_safety::SafetyGate>();
  // Best effort only. The guaranteed stop is the plant-side drive_watchdog; see
  // this file's header for why a serial gate cannot stop a latched simulator.
  rclcpp::on_shutdown([node]() {node->PublishStop();});
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
