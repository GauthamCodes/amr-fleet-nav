"""Payload-scaled acceleration and jerk limiting, as pure functions.

No ROS imports, so the arithmetic that shapes every command the robot executes
is testable without a simulator - the same reason ``amr_safety.safety_model``
mirrors the C++ gate in Python.

UNITS, STATED BECAUSE THEY ARE THE ARGUMENT

    [v]     m/s        commanded velocity
    [a]     m/s^2      acceleration, the rate of change of v
    [j]     m/s^3      JERK, the rate of change of a

    The rotational triple is rad/s, rad/s^2, rad/s^3. PLAN.md section 6 asks for
    ``d_safe``'s units to be stated for the same reason: a limit whose dimension
    is not written down is a number somebody will later tune by feel.

WHY JERK AT ALL, GIVEN THE STOCK SMOOTHER ALREADY LIMITS ACCELERATION

    ``nav2_velocity_smoother`` bounds the FIRST derivative. A command that steps
    from +a_max to -a_max still inverts the acceleration in one control period,
    and on a loaded AMR that is what tips a pallet, not the acceleration itself:
    the load sees an impulsive change in force. Bounding the second derivative is
    the thing the stock node does not do, and it is the whole reason this module
    exists rather than a second copy of the smoother (ENGINEERING_NOTES rule 6).

THE PAYLOAD MODEL, AND WHICH DIRECTION IT SCALES

    Traction force is what a chassis actually has a fixed budget of. With F
    bounded, a = F/m, so acceleration available falls as mass rises:

        a_eff = a_unloaded * m_unloaded / (m_unloaded + payload)

    Jerk is scaled by the same ratio: it is the rate at which that same bounded
    force can be redistributed.

    The limits in fleet.yaml are the UNLOADED envelope, which is also what the
    Gazebo DiffDrive plugin enforces as a hard kinematic ceiling. So this scaling
    only ever moves DOWNWARD from a bound the plant already applies. That
    direction is deliberate: an adapter that could raise a limit above the
    plant's would produce a trace showing the plant's clamp and credit it to
    this node.
"""

import math

#: Below this the vehicle is treated as unloaded. Guards against a payload topic
#: publishing a tiny negative value from a load-cell zero drift.
MIN_PAYLOAD_KG = 0.0


def payload_scale(base_mass_kg, payload_kg):
    """Return the factor limits are multiplied by at this payload.

    1.0 unloaded, falling towards 0 as payload grows. See the module docstring
    for the force-limited derivation.
    """
    if base_mass_kg <= 0.0:
        raise ValueError(f"base_mass_kg must be > 0, got {base_mass_kg}")
    payload = max(float(payload_kg), MIN_PAYLOAD_KG)
    return base_mass_kg / (base_mass_kg + payload)


def scaled_limits(limits, base_mass_kg, payload_kg):
    """Return ``limits`` with every bound scaled for the current payload.

    ``limits`` is a mapping with keys ``max_accel_x``, ``max_decel_x``,
    ``max_accel_theta``, ``max_jerk_x`` and ``max_jerk_theta``. Signs are
    preserved, so ``max_decel_x`` stays negative.
    """
    scale = payload_scale(base_mass_kg, payload_kg)
    return {key: value * scale for key, value in limits.items()}


def _clamp(value, low, high):
    """Return ``value`` confined to ``[low, high]``."""
    return max(low, min(high, value))


def limit_axis(target, previous_v, previous_a, dt, accel_max, decel_max, jerk_max):
    """Shape one axis of a command and return ``(velocity, acceleration)``.

    ``accel_max`` is positive, ``decel_max`` negative.

    WHY THIS ANTICIPATES INSTEAD OF JUST CLAMPING

        The obvious implementation - take the acceleration the error implies,
        clamp its rate of change to ``jerk_max``, clamp its value to the
        acceleration limits - is wrong, and wrong in a way that reaches the
        plant. Bounded jerk means the acceleration cannot be removed instantly
        either: from ``a`` it takes ``a / jerk_max`` seconds to reach zero, and
        the velocity keeps climbing throughout. At a = 1.0 m/s^2 and
        j = 2.0 m/s^3 that is 0.25 m/s of OVERSHOOT past the commanded speed.

        This node sits DOWNSTREAM of the stock smoother, so its velocity clamp
        cannot catch that; the overshoot would go straight through twist_mux
        and SafetyGate to the wheels, and the vehicle would exceed the
        ``max_vel_x`` every other component believes it is holding to.

        So the target acceleration is the one that can still be ramped to zero
        exactly as the velocity error closes - the standard jerk-limited
        (S-curve) approach phase, in the DISCRETE form derived below. Far from
        the target it saturates at the acceleration limit and the profile is
        trapezoidal; near it, the acceleration is already on its way down.
    """
    if dt <= 0.0:
        return previous_v, previous_a

    error = target - previous_v

    # The acceleration whose own ramp-down lands on the target, in DISCRETE
    # time. The continuous-time answer is sqrt(2*j*|e|); this loop integrates
    # with rectangles, so shedding acceleration at rate j over n steps covers
    # a^2/(2j) + a*dt/2 rather than a^2/(2j). Solving that for a gives the
    # half-step correction below.
    #
    # The difference is not academic. With the continuous form the profile
    # arrives with acceleration still on the books, the velocity clamp below
    # then absorbs the residual in a single step, and the emitted jerk spikes
    # to 2.9-3.5x jerk_max at the instant the command reaches its target -
    # measured, in exactly the transient a jerk limiter exists to remove. The
    # discrete form brings the worst case to ~1.1x.
    #
    # The second term never asks for more acceleration than lands on the target
    # in one step, which is what keeps the clamp below from engaging at all in
    # the ordinary case.
    if jerk_max > 0.0:
        half_step = 0.5 * jerk_max * dt
        ramp = -half_step + math.sqrt(
            half_step * half_step + 2.0 * jerk_max * abs(error)
        )
    else:
        ramp = 0.0
    a_desired = math.copysign(min(ramp, abs(error) / dt), error)

    # Acceleration bounds. decel_max applies when the command is shrinking the
    # speed and accel_max when it is growing it - a statement about the SIGN OF
    # THE ACCELERATION, not about driving forwards or backwards.
    a_desired = _clamp(a_desired, decel_max, accel_max)

    # Jerk: how far the acceleration may move from where it actually was.
    limited_a = _clamp(
        a_desired, previous_a - jerk_max * dt, previous_a + jerk_max * dt
    )
    limited_a = _clamp(limited_a, decel_max, accel_max)

    velocity = previous_v + limited_a * dt

    # Final guard, and it earns its place: the jerk bound above can still force
    # an acceleration larger than the remaining error when the PREVIOUS
    # acceleration was large - the vehicle is committed and physically cannot
    # shed it faster. Never exceeding the commanded velocity is the property
    # worth keeping, so the velocity is clipped and the reported acceleration
    # recomputed to match what was actually emitted. A returned acceleration
    # that disagreed with the returned velocity would corrupt the next step's
    # jerk bound and every number the trace derives.
    if error >= 0.0:
        velocity = min(velocity, target)
    if error <= 0.0:
        velocity = max(velocity, target)
    limited_a = (velocity - previous_v) / dt

    return velocity, limited_a


def limit_twist(target, previous, dt, limits):
    """Shape a full 2-D command.

    ``target`` and ``previous`` are ``(v, w)`` pairs; ``previous_accel`` is
    carried by the caller. Returns ``((v, w), (a, alpha))``.
    """
    target_v, target_w = target
    (prev_v, prev_w), (prev_a, prev_alpha) = previous

    v, a = limit_axis(
        target_v,
        prev_v,
        prev_a,
        dt,
        limits["max_accel_x"],
        limits["max_decel_x"],
        limits["max_jerk_x"],
    )
    # Rotation is symmetric: there is no "reverse" about the yaw axis, so the
    # same magnitude bounds the acceleration in both directions.
    w, alpha = limit_axis(
        target_w,
        prev_w,
        prev_alpha,
        dt,
        limits["max_accel_theta"],
        -limits["max_accel_theta"],
        limits["max_jerk_theta"],
    )
    return (v, w), (a, alpha)


def limits_from_robot(robot):
    """Return the unloaded limit mapping for a ``fleet.yaml`` entry."""
    return {
        "max_accel_x": float(robot["max_accel_x"]),
        "max_decel_x": -abs(float(robot["max_decel_x"])),
        "max_accel_theta": float(robot["max_accel_theta"]),
        "max_jerk_x": float(robot["max_jerk_x"]),
        "max_jerk_theta": float(robot["max_jerk_theta"]),
    }
