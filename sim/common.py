"""Shared utilities: load the model and simulate the gait."""

from pathlib import Path

import mujoco
import numpy as np

import robots
from gait import GaitParams, Steering, targets

CONTROL_HZ = 50  # same as Microduck / Open Duck


def load(robot=None):
    model = mujoco.MjModel.from_xml_path(str(robots.get(robot).scene))
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    mujoco.mj_forward(model, data)
    return model, data


def torso_tilt(data) -> float:
    """Angle (rad) between the torso's vertical axis and the world vertical."""
    up_z = data.xmat[1].reshape(3, 3)[2, 2]
    return float(np.arccos(np.clip(up_z, -1, 1)))


def torso_pitch_roll(data):
    """Torso pitch and roll (rad), as the IMU would measure them."""
    R = data.xmat[1].reshape(3, 3)
    pitch = float(np.arcsin(np.clip(-R[2, 0], -1, 1)))
    roll = float(np.arctan2(R[2, 1], R[2, 2]))
    return pitch, roll


def fallen(data) -> bool:
    return data.qpos[2] < 0.05 or torso_tilt(data) > 0.8


def yaw(data) -> float:
    R = data.xmat[1].reshape(3, 3)
    return float(np.arctan2(R[1, 0], R[0, 0]))


def simulate(params: GaitParams, seconds: float, on_frame=None, load_fn=None):
    """Runs the gait. Returns metrics. on_frame(model, data) is called at 30 fps.

    load_fn(robot) -> (model, data) allows changing the terrain or the model.
    """
    N_LEG = params.rb.n_leg
    model, data = (load_fn or load)(params.robot)
    steps_per_ctrl = int(round(1 / (CONTROL_HZ * model.opt.timestep)))
    frame_every = int(round(1 / (30 * model.opt.timestep)))
    n = int(seconds / model.opt.timestep)
    max_tilt = 0.0
    max_torque = 0.0
    torques, errors = [], []
    prev_ctrl, max_cmd_vel = None, 0.0
    yaw_total, prev_yaw = 0.0, yaw(data)
    steering = Steering(params)
    fell_at = None
    for i in range(n):
        if i % steps_per_ctrl == 0:
            pitch, roll = torso_pitch_roll(data)
            p_now = steering.update(yaw_total, 1 / CONTROL_HZ)
            data.ctrl[:] = targets(data.time, p_now, pitch, roll)
            if prev_ctrl is not None and data.time > 0.5:
                vel = np.abs(data.ctrl[:N_LEG] - prev_ctrl) * CONTROL_HZ
                max_cmd_vel = max(max_cmd_vel, float(vel.max()))
            prev_ctrl = data.ctrl[:N_LEG].copy()
            y = yaw(data)
            yaw_total += (y - prev_yaw + np.pi) % (2 * np.pi) - np.pi  # accumulated turn
            prev_yaw = y
        mujoco.mj_step(model, data)
        max_tilt = max(max_tilt, torso_tilt(data))
        max_torque = max(max_torque, float(np.abs(data.actuator_force[:N_LEG]).max()))
        if data.time > 2.0 and i % 5 == 0:
            torques.append(np.abs(data.actuator_force[:N_LEG]))
            errors.append(np.abs(data.ctrl[:N_LEG] - data.qpos[7 : 7 + N_LEG]))
        if on_frame is not None and i % frame_every == 0:
            on_frame(model, data)
        if fell_at is None and fallen(data):
            fell_at = data.time
            if on_frame is None:
                break
    return {
        "x": float(data.qpos[0]),
        "y": float(data.qpos[1]),
        "yaw_deg": float(np.degrees(yaw_total)),
        "fell_at": fell_at,
        "max_tilt_deg": float(np.degrees(max_tilt)),
        "max_leg_torque": max_torque,
        # 99th percentile: leg torque (N·m) and tracking error (degrees)
        "p99_leg_torque": float(np.percentile(torques, 99)) if torques else 0.0,
        "p99_track_err_deg": float(np.degrees(np.percentile(errors, 99))) if errors else 0.0,
        # Maximum speed commanded to a leg servo (rad/s); STS3215 ~4.7 rad/s with no load
        "max_cmd_vel": max_cmd_vel,
        "time": float(data.time),
    }
