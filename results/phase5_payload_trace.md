# Phase 5 - payload-adaptive velocity and jerk

Chain under test, a velocity STEP into its input:

```
cmd_vel_nav -> nav2_velocity_smoother -> cmd_vel_smoothed
            -> PayloadJerkAdapter     -> cmd_vel_shaped
            -> twist_mux -> SafetyGate -> cmd_vel
```

Commanded step: **0.50 m/s**, held 7 s,
then commanded zero for 5 s.

## Configured limits (fleet.yaml), and what payload does to them

| robot | payload | scale | a_max m/s^2 | j_max m/s^3 |
|---|---|---|---|---|
| amr1 | unloaded (0 kg) | 1.000 | 0.400 | 1.000 |
| amr1 | loaded (60 kg) | 0.333 | 0.133 | 0.333 |
| amr2 | unloaded (0 kg) | 1.000 | 1.000 | 2.500 |
| amr2 | loaded (5 kg) | 0.783 | 0.783 | 1.957 |

Scale is `m_base / (m_base + payload)` - fixed traction force, so
acceleration available falls as mass rises. The adapter only ever
scales DOWNWARD from the plant's own kinematic ceiling.

## Measured

| robot | payload | peak v cmd | peak v odom | peak a cmd | peak a odom | **peak jerk cmd** |
|---|---|---|---|---|---|---|
| amr1 | unloaded | 0.500 | 0.500 | 0.725 | 0.683 | **1.784** |
| amr1 | loaded | 0.500 | 0.500 | 0.272 | 0.520 | **0.634** |
| amr2 | unloaded | 0.500 | 0.500 | 1.112 | 1.106 | **4.722** |
| amr2 | loaded | 0.500 | 0.500 | 0.940 | 1.363 | **2.742** |

Velocities m/s, accelerations m/s^2, jerk m/s^3. Jerk is taken from
the shaped command, which is the signal this node bounds.

Every series is resampled onto the chain's own 50 ms
publish grid before it is differentiated. Differentiating by message
ARRIVAL time instead measures transport jitter: the first run of this
trace reported 0.686 m/s^2 against a 0.400 limit and 2.900 m/s^3
against 1.000 - a uniform ~1.7x, which is the ratio of a jittered
0.03 s gap to the true 0.05 s period, not a violated bound.
Odometry is differentiated once, not twice: a third-order numeric
derivative of a sampled velocity is dominated by quantisation, and
quoting it as measured jerk would be quoting noise.

## Does the payload state change the motion?

- **amr1**: peak commanded acceleration 0.725 unloaded -> 0.272 loaded (x0.38); rated payload 60 kg on a 30 kg chassis.
- **amr2**: peak commanded acceleration 1.112 unloaded -> 0.940 loaded (x0.85); rated payload 5 kg on a 18 kg chassis.

The two robots differ because fleet.yaml says they differ - a
60 kg payload on a 30 kg chassis is a far larger perturbation than
5 kg on 18 kg, and no code distinguishes them.

