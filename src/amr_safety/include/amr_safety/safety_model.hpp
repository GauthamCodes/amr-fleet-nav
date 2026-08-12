// Copyright 2026 Gautham. Apache-2.0.
//
// d_safe = k*v^2 + d_min, and the hysteresis latch that consumes it.
//
// No ROS types, so the decision arithmetic is separable from the node that runs
// it. The mirror image of this file is amr_safety/safety_model.py, which derives
// `k` from fleet.yaml and is what tests/test_safety_distance.py exercises; the
// launch file passes the Python-derived constants into the node, so there is one
// source for the numbers and two implementations of the same two lines.

#ifndef AMR_SAFETY__SAFETY_MODEL_HPP_
#define AMR_SAFETY__SAFETY_MODEL_HPP_

namespace amr_safety
{

/// Returns the safe following distance at a measured speed, in metres.
///
/// `speed` is MEASURED, from odometry, never commanded (docs/ENGINEERING_NOTES.md rule 3). Its
/// sign is irrelevant because it is squared, so a reversing robot gets the same
/// envelope as a forward one. [k] = s^2/m, so k*v^2 yields metres.
inline double DSafe(double k, double d_min, double speed)
{
  return k * speed * speed + d_min;
}

/// Returns the clearance at which a blocked gate is allowed to release.
inline double ReleaseDistance(double stop_distance, double release_margin)
{
  return stop_distance + release_margin;
}

/// The blocked/clear state machine, with hysteresis on release.
///
/// Entry is on `clearance <= stop`. Release needs BOTH a larger clearance and a
/// minimum dwell, because either alone chatters: a bare threshold chatters on
/// range noise, and a bare dwell chatters on anything slower than the dwell.
class HysteresisLatch
{
public:
  /// Builds a latch in the clear state.
  HysteresisLatch(double release_margin, double min_hold_s)
  : release_margin_(release_margin), min_hold_s_(min_hold_s) {}

  /// Advances the latch by one observation and returns whether it is blocked.
  bool Update(double clearance, double stop_distance, double now)
  {
    if (blocked_) {
      const bool clear_enough =
        clearance > ReleaseDistance(stop_distance, release_margin_);
      if (clear_enough && (now - entered_at_) >= min_hold_s_) {
        blocked_ = false;
      }
    } else if (clearance <= stop_distance) {
      blocked_ = true;
      entered_at_ = now;
    }
    return blocked_;
  }

  /// Forces the blocked state, for fail-closed paths such as a stale sensor.
  void ForceBlocked(double now)
  {
    if (!blocked_) {
      blocked_ = true;
      entered_at_ = now;
    }
  }

  bool blocked() const {return blocked_;}
  double release_margin() const {return release_margin_;}

private:
  double release_margin_;
  double min_hold_s_;
  bool blocked_ {false};
  double entered_at_ {0.0};
};

}  // namespace amr_safety

#endif  // AMR_SAFETY__SAFETY_MODEL_HPP_
