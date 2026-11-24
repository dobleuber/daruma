"""Scripted gait (no RL) with inverse kinematics of the legs.

Works for Mini13 (5 motors per leg) and Mini16 (6, with ankle roll).

Idea:
  1. The body sways from side to side (hip roll) to load the weight onto one foot.
     If there is ankle roll, it compensates so the feet stay flat.
  2. The free foot is lifted (knee) and moved forward (hip pitch); the support foot
     moves backwards, pushing the body forward.
  3. The ankle pitch keeps the feet parallel to the body.
  4. To turn, it takes shorter steps on one side and rotates the hip yaw.
  5. The head compensates the body's tilt (like birds do) and looks around.

Actuator order (same as in the XMLs):
  left leg, right leg (hip_yaw, hip_roll, hip_pitch, knee, ankle[, ankle_roll]),
  then head (neck_pitch, head_pitch, head_yaw[, head_roll]).
"""

from dataclasses import dataclass, fields, replace

import numpy as np

import robots


@dataclass
class GaitParams:
    """Fields left as None take the robot's default value (robots.py)."""

    robot: str = None  # "mini13" / "mini16" (None = robots.DEFAULT)
    freq: float = None  # cycles per second (each cycle = one step with each foot)
    step_len: float = None  # m, half of the foot's forward/backward travel
    lift: float = None  # m, height the foot is lifted to
    sway: float = None  # m, lateral sway of the body
    sway_lead: float = None  # rad, phase lead of the sway relative to the step
    hip_height: float = None  # m, hip height above the sole (knees bent)
    foot_x: float = None  # m, foot position relative to the hip
    turn: float = 0.0  # turn (+ = left). With steer: requested turn rate in rad/s
    steer: bool = True  # corrects the heading with the IMU (otherwise the turn is open loop)
    steer_gain: float = None  # heading control gain (per rad of error)
    steer_damping: float = None  # damping (per rad/s of turn rate, gyroscope)
    turn_yaw: float = None  # rad of hip yaw per step when turn = 1
    stride_bias: float = None  # corrects drift when walking straight (unequal steps)
    balance: float = None  # rad of ankle pitch per rad of torso tilt (IMU); 0 = no correction
    ramp_time: float = 1.5  # s, soft start
    head: bool = True  # animate the head

    def __post_init__(self):
        rb = robots.get(self.robot)
        self.robot = rb.name
        for f in fields(self):
            if getattr(self, f.name) is None:
                setattr(self, f.name, rb.gait.get(f.name, 0.0))

    @property
    def rb(self):
        return robots.get(self.robot)


def leg_ik(x: float, y: float, z: float, robot=None) -> np.ndarray:
    """Angles (hip_roll, hip_pitch, knee, ankle) that bring the ankle to (x, y, z).

    (x, y, z) is measured from the hip pitch axis with roll = 0, in the torso frame.
    The knee bends backwards (like Microduck's) and the ankle is chosen so that
    the foot stays parallel to the torso in pitch.
    """
    rb = robots.get(robot)
    L1, L2, rp = rb.thigh, rb.shin, rb.roll_to_pitch
    pz = z - rp  # relative to the roll axis
    roll = np.arctan2(y, -pz)
    h = np.hypot(y, pz) - rp  # downward distance in the plane of the leg
    r = min(np.hypot(x, h), L1 + L2 - 1e-4)
    cos_knee = (L1**2 + L2**2 - r**2) / (2 * L1 * L2)
    knee = -(np.pi - np.arccos(np.clip(cos_knee, -1, 1)))
    alpha = np.arctan2(x, h)
    beta = np.arccos(np.clip((L1**2 + r**2 - L2**2) / (2 * L1 * r), -1, 1))
    hip_pitch = -alpha + beta
    ankle = -(hip_pitch + knee)
    return np.array([roll, hip_pitch, knee, ankle])


def leg_targets(t: float, p: GaitParams):
    """Ankle position of each foot (relative to its hip) and hip yaw."""
    ramp = min(1.0, t / p.ramp_time) if p.ramp_time > 0 else 1.0
    phi = 2 * np.pi * p.freq * t
    z0 = -(p.hip_height - p.rb.ankle_to_sole)

    # Left foot in the air when sin(phi) > 0, right foot when sin(phi) < 0.
    swing_l = max(0.0, np.sin(phi)) ** 2  # smooth profile (no impacts)
    swing_r = max(0.0, -np.sin(phi)) ** 2
    # Turning: shorter steps on the side it turns towards (like a tank).
    # Limit ±0.4: beyond that the servos cannot reach the required speed.
    k = np.clip(p.turn + p.stride_bias, -0.4, 0.4)
    x_l = p.foot_x - p.step_len * (1 - k) * ramp * np.cos(phi)
    x_r = p.foot_x + p.step_len * (1 + k) * ramp * np.cos(phi)
    # Sway: the body moves towards the support foot (the feet move the opposite way).
    sway = p.sway * ramp * np.sin(phi + p.sway_lead)
    z_l = z0 + p.lift * ramp * swing_l
    z_r = z0 + p.lift * ramp * swing_r
    # Turn: the hip yaw rotates during the step, just like x.
    yaw_l = -p.turn * p.turn_yaw * ramp * np.cos(phi) / 2
    yaw_r = p.turn * p.turn_yaw * ramp * np.cos(phi) / 2
    return (x_l, sway, z_l, yaw_l), (x_r, sway, z_r, yaw_r)


def leg_command(yaw, x, y, z, rb) -> list:
    roll, pitch, knee, ankle = leg_ik(x, y, z, rb)
    q = [yaw, roll, pitch, knee, ankle]
    if rb.ankle_roll:
        q.append(-roll)  # ankle roll = -hip roll: flat foot
    return q


def targets(t: float, p: GaitParams, torso_pitch: float = 0.0, torso_roll: float = 0.0) -> np.ndarray:
    """Target angles of all motors at time t."""
    rb = p.rb
    (xl, yl, zl, yawl), (xr, yr, zr, yawr) = leg_targets(t, p)
    q = np.zeros(rb.n_act)
    q[: rb.per_leg] = leg_command(yawl, xl, yl, zl, rb)
    q[rb.per_leg : rb.n_leg] = leg_command(yawr, xr, yr, zr, rb)
    if p.balance:
        # Ankle balance: if the torso tilts, the ankles push it back.
        ankle = rb.leg_joints.index("ankle")
        q[ankle] += p.balance * torso_pitch
        q[rb.per_leg + ankle] += p.balance * torso_pitch
    if p.head:
        q[rb.n_leg :] = head_targets(t, torso_pitch, torso_roll, rb.n_head)
    return q


def head_targets(t: float, torso_pitch: float, torso_roll: float, n_head: int = 3) -> np.ndarray:
    """Head: compensates the body's tilt and looks to the sides."""
    neck_pitch = 0.15 * np.sin(2 * np.pi * 0.3 * t)
    head_pitch = -torso_pitch - neck_pitch  # keeps the gaze horizontal
    head_yaw = 0.6 * np.sin(2 * np.pi * 0.15 * t)
    q = [neck_pitch, head_pitch, head_yaw, -torso_roll]  # head roll compensates the sway
    return np.array(q[:n_head])


class Steering:
    """Heading control with the IMU (the BNO055 gives the absolute heading).

    Keeps a reference heading that advances at the requested turn rate
    (params.turn, rad/s) and adjusts the gait's turn (unequal steps +
    hip yaw) to follow it. With turn = 0 it keeps the initial direction.
    """

    MAX_TURN = 0.35

    def __init__(self, params: GaitParams):
        self.params = params
        self.yaw_ref = 0.0
        self.prev_yaw = None
        self.rate = 0.0

    def update(self, yaw_measured: float, dt: float) -> GaitParams:
        p = self.params
        if not p.steer:
            return p
        if self.prev_yaw is not None:  # filtered turn rate (on the robot: gyroscope)
            self.rate += 0.2 * ((yaw_measured - self.prev_yaw) / dt - self.rate)
        self.prev_yaw = yaw_measured
        self.yaw_ref += p.turn * dt
        err = self.yaw_ref - yaw_measured
        u = p.steer_gain * err - p.steer_damping * (self.rate - p.turn) + 0.3 * p.turn
        turn_cmd = float(np.clip(u, -self.MAX_TURN, self.MAX_TURN))
        return replace(p, turn=turn_cmd, steer=False)
