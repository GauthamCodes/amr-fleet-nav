"""What the BSP validators accept, and - more importantly - what they refuse to.

The interesting half of a validator is its FALSE POSITIVES. A check that rejects a
healthy sensor is worse than no check at all, because it removes the data the stack
needs while reading as diligence. Three tests here exist only to pin that boundary:
a stationary IMU reads 1 g and must pass, an open aisle returns a scan that is
almost entirely ``inf`` and must pass, and a handful of NaN beams warns rather than
discarding the scan around them.

Everything is a hand-rolled stub, so this file runs with no ROS graph, no simulator
and no rclpy - the same property that makes tests/test_clearance.py cheap.
"""

import math

from amr_bsp.validators import (
    CAMERA_ENCODINGS,
    OK,
    REJECT,
    validate_camera,
    validate_imu,
    validate_lidar,
    WARN,
)

#: Bounds the BSP node derives per robot; fixed here so the tests state their own.
MAX_ANGULAR_RATE = 4.0
MAX_LINEAR_ACCEL = 29.42
GRAVITY = 9.80665

CAMERA_WIDTH = 160
CAMERA_HEIGHT = 120


class FakeVector:
    """Minimal stand-in for geometry_msgs/Vector3."""

    def __init__(self, x=0.0, y=0.0, z=0.0):
        """Store the three components."""
        self.x, self.y, self.z = x, y, z


class FakeQuaternion:
    """Minimal stand-in for geometry_msgs/Quaternion, identity by default."""

    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        """Store the four components."""
        self.x, self.y, self.z, self.w = x, y, z, w


class FakeImu:
    """Minimal stand-in for sensor_msgs/Imu, at rest and level by default."""

    def __init__(
        self, angular_velocity=None, linear_acceleration=None, orientation=None
    ):
        """Build an IMU reading, defaulting to a healthy stationary sensor."""
        self.angular_velocity = angular_velocity or FakeVector()
        self.linear_acceleration = linear_acceleration or FakeVector(z=GRAVITY)
        self.orientation = orientation or FakeQuaternion()


class FakeScan:
    """Minimal stand-in for sensor_msgs/LaserScan, matching the fleet's geometry."""

    def __init__(self, ranges=None, count=360, angle_increment=None):
        """Build a full-circle scan with the fields the validator reads."""
        self.angle_min = -math.pi
        self.angle_max = math.pi
        self.angle_increment = (
            2.0 * math.pi / (count - 1) if angle_increment is None else angle_increment
        )
        self.range_min = 0.12
        self.range_max = 12.0
        self.ranges = [4.0] * count if ranges is None else ranges


class FakeImage:
    """Minimal stand-in for sensor_msgs/Image at the configured camera resolution."""

    def __init__(
        self,
        width=CAMERA_WIDTH,
        height=CAMERA_HEIGHT,
        encoding="rgb8",
        step=None,
        data=None,
    ):
        """Build an image whose buffer is self-consistent unless told otherwise."""
        self.width = width
        self.height = height
        self.encoding = encoding
        self.step = width * CAMERA_ENCODINGS.get(encoding, 3) if step is None else step
        self.data = bytes(self.step * height) if data is None else data


def imu(**kwargs):
    return validate_imu(
        FakeImu(**kwargs.pop("fields", {})),
        MAX_ANGULAR_RATE,
        MAX_LINEAR_ACCEL,
        **kwargs,
    )


# --------------------------------------------------------------------------- IMU


def test_a_healthy_stationary_imu_passes():
    """1 g at rest is correct, not a fault: specific force includes gravity."""
    assert imu().level == OK


def test_a_plausible_turn_passes():
    fields = {"angular_velocity": FakeVector(z=0.9)}
    assert imu(fields=fields).level == OK


def test_an_implausible_angular_velocity_is_rejected():
    """The primary demonstrated check, per PLAN.md section 3."""
    fields = {"angular_velocity": FakeVector(z=50.0)}
    verdict = imu(fields=fields)
    assert verdict.level == REJECT
    assert not verdict.accepted
    assert "angular velocity" in verdict.reason
    assert "50.000" in verdict.reason, "the reason must name the offending value"


def test_an_implausible_rate_on_any_axis_is_rejected():
    for axis in ("x", "y", "z"):
        fields = {"angular_velocity": FakeVector(**{axis: 12.0})}
        verdict = imu(fields=fields)
        assert verdict.level == REJECT, f"axis {axis} went unchecked"
        assert f"|w_{axis}|" in verdict.reason


def test_a_nan_angular_velocity_is_rejected():
    fields = {"angular_velocity": FakeVector(y=float("nan"))}
    assert imu(fields=fields).level == REJECT


def test_an_infinite_acceleration_is_rejected():
    fields = {"linear_acceleration": FakeVector(z=float("inf"))}
    assert imu(fields=fields).level == REJECT


def test_an_implausible_acceleration_is_rejected():
    fields = {"linear_acceleration": FakeVector(x=40.0, z=GRAVITY)}
    verdict = imu(fields=fields)
    assert verdict.level == REJECT
    assert "linear acceleration" in verdict.reason


def test_an_unnormalised_quaternion_is_rejected():
    fields = {"orientation": FakeQuaternion(w=0.5)}
    verdict = imu(fields=fields)
    assert verdict.level == REJECT
    assert "unit quaternion" in verdict.reason


def test_a_stale_imu_sample_is_rejected():
    assert imu(age=0.4, max_age=0.5).level == OK
    assert imu(age=0.6, max_age=0.5).level == REJECT


def test_a_stamp_that_does_not_advance_is_rejected():
    assert imu(since_previous=0.01).level == OK
    assert imu(since_previous=0.0).level == REJECT
    assert imu(since_previous=-0.01).level == REJECT


# ------------------------------------------------------------------------- LiDAR


def test_a_nominal_scan_passes():
    assert validate_lidar(FakeScan()).level == OK


def test_an_open_aisle_is_not_a_fault():
    """An all-inf scan is legitimate: inf is the sensor saying 'nothing in range'."""
    assert validate_lidar(FakeScan(ranges=[float("inf")] * 360)).level == OK


def test_a_few_nan_beams_warn_but_are_still_republished():
    ranges = [4.0] * 360
    for index in range(5):
        ranges[index] = float("nan")
    verdict = validate_lidar(FakeScan(ranges=ranges))
    assert verdict.level == WARN
    assert verdict.accepted, "dropping the scan around 5 bad beams starves Nav2"


def test_too_many_nan_beams_reject_the_scan():
    ranges = [float("nan")] * 200 + [4.0] * 160
    verdict = validate_lidar(FakeScan(ranges=ranges), max_nan_fraction=0.25)
    assert verdict.level == REJECT
    assert "55.6%" in verdict.reason


def test_an_entirely_nan_scan_is_rejected():
    verdict = validate_lidar(FakeScan(ranges=[float("nan")] * 360))
    assert verdict.level == REJECT
    assert "every one of 360 beams" in verdict.reason


def test_an_empty_scan_is_rejected():
    assert validate_lidar(FakeScan(ranges=[], count=0)).level == REJECT


def test_a_zero_angle_increment_is_rejected():
    assert validate_lidar(FakeScan(angle_increment=0.0)).level == REJECT


def test_a_beam_count_inconsistent_with_the_angles_is_rejected():
    """The failure that silently rotates every obstacle in the costmap."""
    verdict = validate_lidar(FakeScan(angle_increment=2.0 * math.pi / 180))
    assert verdict.level == REJECT
    assert "inconsistent" in verdict.reason


def test_both_full_circle_indexing_conventions_are_accepted():
    for increment in (2.0 * math.pi / 359, 2.0 * math.pi / 360):
        assert validate_lidar(FakeScan(angle_increment=increment)).level == OK


def test_an_inverted_range_band_is_rejected():
    scan = FakeScan()
    scan.range_min, scan.range_max = 12.0, 0.12
    assert validate_lidar(scan).level == REJECT


def test_a_stale_scan_is_rejected():
    assert validate_lidar(FakeScan(), age=0.6, max_age=0.5).level == REJECT


# ------------------------------------------------------------------------ Camera


def test_a_nominal_image_passes():
    assert validate_camera(FakeImage(), CAMERA_WIDTH, CAMERA_HEIGHT).level == OK


def test_a_renegotiated_resolution_is_rejected():
    """Silent resolution changes turn every downstream pixel into a wrong metre."""
    verdict = validate_camera(
        FakeImage(width=320, height=240), CAMERA_WIDTH, CAMERA_HEIGHT
    )
    assert verdict.level == REJECT
    assert "320x240" in verdict.reason


def test_a_degenerate_image_is_rejected():
    assert validate_camera(FakeImage(width=0), CAMERA_WIDTH, CAMERA_HEIGHT).level == (
        REJECT
    )


def test_an_unsupported_encoding_is_rejected():
    image = FakeImage()
    image.encoding = "yuv422"
    verdict = validate_camera(image, CAMERA_WIDTH, CAMERA_HEIGHT)
    assert verdict.level == REJECT
    assert "yuv422" in verdict.reason


def test_mono8_is_accepted_with_its_own_stride():
    image = FakeImage(encoding="mono8")
    assert image.step == CAMERA_WIDTH
    assert validate_camera(image, CAMERA_WIDTH, CAMERA_HEIGHT).level == OK


def test_a_truncated_buffer_is_rejected():
    image = FakeImage()
    image.data = image.data[:-1]
    verdict = validate_camera(image, CAMERA_WIDTH, CAMERA_HEIGHT)
    assert verdict.level == REJECT
    assert "truncated" in verdict.reason


def test_a_stride_too_small_for_the_width_is_rejected():
    image = FakeImage(step=CAMERA_WIDTH)
    verdict = validate_camera(image, CAMERA_WIDTH, CAMERA_HEIGHT)
    assert verdict.level == REJECT
    assert "step" in verdict.reason


def test_a_stale_image_is_rejected():
    verdict = validate_camera(
        FakeImage(), CAMERA_WIDTH, CAMERA_HEIGHT, age=0.9, max_age=0.5
    )
    assert verdict.level == REJECT


def test_a_verdict_is_truthy_exactly_when_it_may_be_republished():
    assert validate_lidar(FakeScan())
    assert not validate_imu(
        FakeImu(angular_velocity=FakeVector(z=50.0)), MAX_ANGULAR_RATE, MAX_LINEAR_ACCEL
    )


def test_pytest_approx_is_not_needed_for_the_encoding_table():
    """Guard the guard: the encoding table must not silently lose an entry."""
    assert set(CAMERA_ENCODINGS) >= {"rgb8", "bgr8", "mono8"}
    assert CAMERA_ENCODINGS["rgb8"] == 3
    assert CAMERA_ENCODINGS["mono8"] == 1
