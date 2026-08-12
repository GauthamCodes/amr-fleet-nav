// Copyright 2026 Gautham. Apache-2.0.
//
// Distance from a point to the robot's footprint polygon.
//
// A direct port of amr_navigation/clearance.py, which is the tested reference.
// Keeping the two in step is not left to inspection: scripts/verify_clearance.py
// re-derives the gate's reported clearance offline from the recorded scan using the
// Python functions and reports the worst disagreement, so a divergence between
// these two implementations shows up as a number in the evidence report rather
// than as a silent difference between what was tested and what ran.
//
// WHY CLEARANCE IS MEASURED FROM THE POLYGON AND NOT FROM THE BASE FRAME
//   This chassis' origin is its drive axle and the body is offset forward of it, so
//   the footprint reaches 0.49 m ahead of the origin and 0.21 m behind. A clearance
//   quoted from the origin overstates the gap in front by nearly half a metre - in
//   the direction of travel, which is the only direction that matters here.

#ifndef AMR_SAFETY__FOOTPRINT_HPP_
#define AMR_SAFETY__FOOTPRINT_HPP_

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace amr_safety
{

/// A point in the robot's base frame, in metres.
struct Point
{
  double x;
  double y;
};

/// Returns the distance from a point to a line segment.
inline double SegmentDistance(
  double px, double py, double ax, double ay, double bx, double by)
{
  const double dx = bx - ax;
  const double dy = by - ay;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 0.0) {
    return std::hypot(px - ax, py - ay);
  }
  double t = ((px - ax) * dx + (py - ay) * dy) / length_squared;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
}

/// Returns whether a point lies inside a polygon. Ray casting, any winding.
inline bool PointInPolygon(const std::vector<Point> & polygon, double px, double py)
{
  bool inside = false;
  const size_t count = polygon.size();
  for (size_t i = 0; i < count; ++i) {
    const Point & a = polygon[i];
    const Point & b = polygon[(i + 1) % count];
    if ((a.y > py) != (b.y > py)) {
      const double crossing = a.x + (py - a.y) * (b.x - a.x) / (b.y - a.y);
      if (px < crossing) {
        inside = !inside;
      }
    }
  }
  return inside;
}

/// Returns the distance from a point to a polygon's boundary, NEGATIVE inside.
///
/// The sign is what lets the caller tell "just touching" from "this return came
/// off the robot's own body and must be discarded".
inline double DistanceToPolygon(
  const std::vector<Point> & polygon, double px, double py)
{
  const size_t count = polygon.size();
  double best = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < count; ++i) {
    const Point & a = polygon[i];
    const Point & b = polygon[(i + 1) % count];
    best = std::min(best, SegmentDistance(px, py, a.x, a.y, b.x, b.y));
  }
  return PointInPolygon(polygon, px, py) ? -best : best;
}

}  // namespace amr_safety

#endif  // AMR_SAFETY__FOOTPRINT_HPP_
