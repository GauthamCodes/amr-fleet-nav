#!/usr/bin/env python3
"""PHASE 1 - scripted survey drive for the SLAM map artifact.

Drives a closed rectangular circuit of the main aisle so slam_toolbox has
something to map, then stops. The route and the number of laps are parameters;
the defaults are the ones the committed map artifact was produced with.

WHY A CLOSED CIRCUIT DRIVEN TWICE
    A survey that ends where it started, having gone round twice, gives
    slam_toolbox a genuine revisit to close a loop against. An out-and-back would
    revisit the same cells with the same heading and the same viewpoint, which
    scan matching finds trivially and which therefore proves nothing about the
    map's global consistency. The second lap is where any accumulated drift shows
    up as a doubled rack face.

WHY IT DRIVES ON WHEEL ODOMETRY
    The controller closes the loop on ``/<robot>/odom``, not on ground truth and
    not on the SLAM pose. Ground truth is not available to a real robot; the SLAM
    pose steps whenever a loop closes, and a step in the feedback of a waypoint
    controller becomes a lurch in the trajectory that ends up in the map. Wheel
    odometry over a 60 m circuit is more than accurate enough to hit 0.25 m
    waypoint tolerances in a 5.5 m aisle, and the drive is not the measurement -
    the map is.

WHY NOT NAV2
    Nav2 needs a map to plan in and this run exists to produce one. Phase 1's
    goal-to-goal run (``nav_goal_run.py``) is where the navigation stack drives.
"""

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def yaw_of(q):
    """Return the yaw angle of a geometry_msgs quaternion."""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def wrap(angle):
    """Wrap an angle into [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


#: Default circuit, in WORLD coordinates, one lap.
#:
#: Rack rows occupy |y| in [2.75, 3.65] for x in [-13.2, 3.6] and the ramp toe is at
#: x = 2.44, so the drivable aisle is |y| < 2.75, x < 2.44. The circuit runs up one
#: side of that aisle and back down the other at |y| = 1.5, which keeps roughly
#: 1.25 m of clearance to the rack faces on the near side while leaving both rows
#: inside the 12 m LiDAR range from either leg.
#:
#: The east end stops at x = 0.8. Further east the robot would begin to pitch onto
#: the ramp, and a pitched scan plane injects the phantom ground return smoke test 2
#: measured (1.78 m at 8 deg) into the map as a wall that is not there. Phase 2's
#: PitchGate is the prerequisite for mapping the ramp; Phase 1 does not map it.
DEFAULT_ROUTE_X = [0.8, 0.8, -13.0, -13.0]
DEFAULT_ROUTE_Y = [-1.5, 1.5, 1.5, -1.5]


class SurveyDrive(Node):
    """Drive a fixed circuit of waypoints and stop."""

    def __init__(self):
        """Declare the route and control gains, and subscribe to odometry."""
        super().__init__("survey_drive")

        self.declare_parameter("robot_name", "amr1")
        self.declare_parameter("route_x", DEFAULT_ROUTE_X)
        self.declare_parameter("route_y", DEFAULT_ROUTE_Y)
        self.declare_parameter("laps", 2)
        # Well below the 0.60 m/s the robot is capable of. slam_toolbox processes
        # scans at 10 Hz; at survey speed that is a scan every 45 mm of travel.
        self.declare_parameter("cruise_speed", 0.45)
        self.declare_parameter("turn_speed", 0.7)
        self.declare_parameter("heading_gain", 1.6)
        # Beyond this heading error the robot turns in place instead of arcing.
        # An arc that wide would carry it into a rack face before it converged.
        self.declare_parameter("turn_in_place_threshold", 0.5)
        self.declare_parameter("waypoint_tolerance", 0.25)
        # /odom starts at the spawn pose, so these convert odometry into the world
        # coordinates the route is written in. Supplied by the launch file from
        # fleet.yaml - this node does not read the fleet list itself.
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("settle_s", 3.0)
        self.declare_parameter("timeout_s", 900.0)

        get = self.get_parameter
        self.robot_name = get("robot_name").value
        route_x = list(get("route_x").value)
        route_y = list(get("route_y").value)
        if len(route_x) != len(route_y) or not route_x:
            raise ValueError("route_x and route_y must be equal-length and non-empty")
        self.route = list(zip(route_x, route_y)) * int(get("laps").value)
        self.cruise = float(get("cruise_speed").value)
        self.turn_speed = float(get("turn_speed").value)
        self.heading_gain = float(get("heading_gain").value)
        self.turn_threshold = float(get("turn_in_place_threshold").value)
        self.tolerance = float(get("waypoint_tolerance").value)
        self.spawn = (
            float(get("spawn_x").value),
            float(get("spawn_y").value),
            float(get("spawn_yaw").value),
        )
        self.settle_s = float(get("settle_s").value)
        self.timeout_s = float(get("timeout_s").value)

        self.publisher = self.create_publisher(Twist, f"/{self.robot_name}/cmd_vel", 10)
        self.create_subscription(
            Odometry, f"/{self.robot_name}/odom", self._on_odom, 10
        )

        # SLAM_TOOLBOX'S MAP PUBLICATION IS SUBSCRIBER-GATED, AND THAT COSTS A RUN.
        # It builds and publishes the occupancy grid only while something is
        # subscribed. With Nav2 absent - which is exactly this run's configuration -
        # nothing subscribes, so nothing is ever published, and its TRANSIENT_LOCAL
        # publisher has no sample in its history to hand a late joiner. The map
        # saver that runs after this node exits therefore subscribes to a live
        # topic, waits out its whole timeout and exits 1, having never received a
        # map that slam_toolbox was perfectly capable of producing:
        #
        #     [map_saver]: Saving map from '/amr1/map' topic to '.../phase1_map'
        #     [map_saver]: Failed to spin map subscription
        #
        # No QoS warning, no error from slam_toolbox, and a three minute drive
        # thrown away. The same launch WITH Nav2 saves fine, because the global
        # costmap's static layer holds the subscription open. Holding one here
        # makes the artifact independent of who else happens to be listening.
        # (nav2_costmap_2d shows the same pattern - its OccupancyGrid publisher is
        # subscriber-gated too.)
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            OccupancyGrid, f"/{self.robot_name}/map", self._on_map, map_qos
        )
        self.map_cells = None

        self.pose = None
        self.index = 0
        self.started = None
        self.travelled = 0.0
        self.previous_xy = None
        self.finished = False
        self.create_timer(0.05, self._tick)
        self.get_logger().info(
            f"survey armed: {len(self.route)} waypoints "
            f"({int(get('laps').value)} laps of {len(route_x)})"
        )

    def _on_odom(self, msg):
        """Convert odometry into world coordinates and accumulate distance."""
        p = msg.pose.pose.position
        cos_yaw, sin_yaw = math.cos(self.spawn[2]), math.sin(self.spawn[2])
        x = self.spawn[0] + p.x * cos_yaw - p.y * sin_yaw
        y = self.spawn[1] + p.x * sin_yaw + p.y * cos_yaw
        self.pose = (x, y, wrap(self.spawn[2] + yaw_of(msg.pose.pose.orientation)))

        if self.previous_xy is not None:
            self.travelled += math.hypot(
                x - self.previous_xy[0], y - self.previous_xy[1]
            )
        self.previous_xy = (x, y)

    def _on_map(self, msg):
        """Record what the map contains, so the drive reports its own product."""
        known = sum(1 for value in msg.data if value >= 0)
        self.map_cells = (known, len(msg.data), msg.info.resolution)

    def _elapsed(self):
        if self.started is None:
            return 0.0
        return (self.get_clock().now() - self.started).nanoseconds / 1e9

    def _publish(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.publisher.publish(twist)

    def _finish(self, reason):
        self.finished = True
        self._publish(0.0, 0.0)
        self.get_logger().info(
            f"SURVEY {reason}: {self.index}/{len(self.route)} waypoints, "
            f"{self.travelled:.1f} m driven in {self._elapsed():.0f} s of sim time"
        )
        if self.map_cells is None:
            # The saver is about to fail; say why now rather than leaving the
            # reader to work it out from a timeout twenty seconds later.
            self.get_logger().error(
                "no map received on /%s/map - slam_toolbox published nothing, so "
                "the map save that follows will have nothing to write" % self.robot_name
            )
        else:
            known, total, resolution = self.map_cells
            area = known * resolution * resolution
            self.get_logger().info(
                f"map at save time: {known}/{total} cells known ({area:.1f} m^2)"
            )
        # Exiting is part of the contract: the survey launch file saves the map on
        # this process's exit, so the whole artifact is produced by one command.
        rclpy.shutdown()

    def _tick(self):
        if self.pose is None or self.finished:
            return
        if self.started is None:
            self.started = self.get_clock().now()

        # Hold still briefly so slam_toolbox has a stationary first scan to anchor
        # its map on. A first scan taken mid-acceleration biases the whole graph.
        if self._elapsed() < self.settle_s:
            self._publish(0.0, 0.0)
            return

        if self._elapsed() > self.timeout_s:
            self._finish("TIMED OUT")
            return

        target = self.route[self.index]
        x, y, yaw = self.pose
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)

        if distance < self.tolerance:
            self.index += 1
            self.get_logger().info(
                f"waypoint {self.index}/{len(self.route)} reached "
                f"({target[0]:+.1f}, {target[1]:+.1f})"
            )
            if self.index >= len(self.route):
                self._finish("COMPLETE")
            return

        error = wrap(math.atan2(dy, dx) - yaw)
        if abs(error) > self.turn_threshold:
            self._publish(0.0, math.copysign(self.turn_speed, error))
            return

        angular = max(-self.turn_speed, min(self.turn_speed, self.heading_gain * error))
        # Slow into the last half metre so the robot settles on the waypoint
        # instead of orbiting it, which would put a knot in the scan graph.
        speed = self.cruise * min(1.0, max(0.35, distance / 0.5))
        self._publish(speed, angular)


def main():
    rclpy.init()
    node = SurveyDrive()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
