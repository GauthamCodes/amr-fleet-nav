#!/usr/bin/env python3
"""Point the Gazebo GUI camera down the warehouse aisle once the GUI is up.

WHY THIS EXISTS

Gazebo's default camera looks at the world origin, and in this warehouse the origin
is the empty floor in front of the ramp. Both robots spawn at x = -11, behind the
camera and out of frame, so the first thing anyone sees when they run a demo with
``headless:=false`` is an empty aisle - which reads as "the robots did not spawn".
They did; the camera is simply pointed elsewhere.

There is no way to say "start the camera here" in the world SDF without replacing
Gazebo's whole default GUI configuration (the ``<gui>`` element is a replacement,
not an overlay), which would mean vendoring a copy of the distribution's gui.config
and having it drift. Instead this asks the running GUI to move, over the same
transport everything else uses.

FAILURE IS NOT AN ERROR. If the service never appears - headless, a Gazebo build
without the camera service, a GUI that is slow to start - this gives up quietly and
exits 0. A demo must never fail because a camera did not move.
"""

import argparse
import math
import subprocess
import sys
import time

#: Looking from just outside the near end of the aisle towards the ramp. Chosen so
#: that both spawn poses (x = -11, y = +/-1.5), the rack rows either side (y = +/-3.2,
#: x from -12 to +2.4) and the ramp beyond them are all in frame at once.
DEFAULT_POSE = (-19.0, 0.0, 3.2)
DEFAULT_PITCH = 0.35
DEFAULT_YAW = 0.0

SERVICE = "/gui/move_to/pose"


def quaternion(roll, pitch, yaw):
    """Return ``(x, y, z, w)`` for an intrinsic Z-Y-X rotation, as Gazebo wants it."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def request(position, orientation):
    """Return the gz.msgs.GUICamera request text for a pose."""
    x, y, z = position
    qx, qy, qz, qw = orientation
    return (
        "pose: {{position: {{x: {:.4f}, y: {:.4f}, z: {:.4f}}}, "
        "orientation: {{x: {:.6f}, y: {:.6f}, z: {:.6f}, w: {:.6f}}}}}"
    ).format(x, y, z, qx, qy, qz, qw)


def move(req, timeout_s):
    """Call the move-to-pose service until it answers or the deadline passes."""
    deadline = time.monotonic() + timeout_s
    attempt = 0
    while time.monotonic() < deadline:
        attempt += 1
        try:
            done = subprocess.run(
                [
                    "gz",
                    "service",
                    "-s",
                    SERVICE,
                    "--reqtype",
                    "gz.msgs.GUICamera",
                    "--reptype",
                    "gz.msgs.Boolean",
                    "--timeout",
                    "2000",
                    "--req",
                    req,
                ],
                capture_output=True,
                text=True,
                timeout=10,
            )
        except (OSError, subprocess.TimeoutExpired):
            time.sleep(1.0)
            continue
        if "true" in done.stdout:
            print(f"gui_camera: framed the aisle after {attempt} attempt(s)")
            return True
        time.sleep(1.0)
    print(
        f"gui_camera: {SERVICE} did not answer within {timeout_s:.0f} s - "
        "leaving the camera where Gazebo put it",
        file=sys.stderr,
    )
    return False


def main(argv=None):
    """Parse arguments and move the camera."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, default=DEFAULT_POSE[0])
    parser.add_argument("--y", type=float, default=DEFAULT_POSE[1])
    parser.add_argument("--z", type=float, default=DEFAULT_POSE[2])
    parser.add_argument("--pitch", type=float, default=DEFAULT_PITCH)
    parser.add_argument("--yaw", type=float, default=DEFAULT_YAW)
    parser.add_argument(
        "--timeout",
        type=float,
        default=45.0,
        help="Seconds to keep retrying before giving up quietly.",
    )
    args = parser.parse_args(argv)
    move(
        request((args.x, args.y, args.z), quaternion(0.0, args.pitch, args.yaw)),
        args.timeout,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
