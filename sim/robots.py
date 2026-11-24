"""Available robots and their geometric data.

Choose the robot: environment variable MINI_ROBOT (mini16 by default) or pass `robot=`
to the functions. E.g.: MINI_ROBOT=mini13 python run_demo.py
"""

import os
from dataclasses import dataclass, field
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent.parent / "model"


@dataclass(frozen=True)
class Robot:
    name: str
    scene: Path
    # Leg geometry (must match the XML)
    roll_to_pitch: float  # m, from the roll axis to the hip pitch axis
    thigh: float  # m
    shin: float  # m
    ankle_to_sole: float  # m
    ankle_roll: bool  # True if it has ankle roll (6 motors per leg)
    n_head: int  # neck/head motors
    gait: dict = field(default_factory=dict)  # default gait parameters
    target_speed: float = 0.06  # m/s, target speed for RL training
    rl_log_std: float = -1.5  # initial RL exploration noise (log of the standard deviation)
    servo_speed: float = 4.7  # rad/s, maximum speed of the leg servo (STS3215 with no load)
    rl_head: bool = False  # True: the RL policy also moves the neck and head (counterweight)

    @property
    def leg_joints(self):
        j = ["hip_yaw", "hip_roll", "hip_pitch", "knee", "ankle"]
        return j + ["ankle_roll"] if self.ankle_roll else j

    @property
    def per_leg(self):
        return len(self.leg_joints)

    @property
    def n_leg(self):
        return 2 * self.per_leg

    @property
    def n_act(self):
        return self.n_leg + self.n_head


ROBOTS = {
    "mini13": Robot(
        name="mini13",
        scene=MODEL_DIR / "scene_mini13.xml",
        roll_to_pitch=0.014,
        thigh=0.055,
        shin=0.06,
        ankle_to_sole=0.025,
        ankle_roll=False,
        n_head=3,
        target_speed=0.06,
        rl_log_std=-2.5,  # without ankle roll it is less stable: explore with less noise
        gait=dict(freq=1.6, step_len=0.016, lift=0.008, sway=0.002, sway_lead=0.0, hip_height=0.115,
                  foot_x=0.0, turn_yaw=0.0, stride_bias=0.0, steer_gain=0.2, steer_damping=0.0),
    ),
    "mini16": Robot(
        # Geometry generated from the CAD (cad/lib/params.scad, cad/build_mjcf.py)
        name="mini16",
        scene=MODEL_DIR / "scene_mini16.xml",
        roll_to_pitch=0.02822,
        thigh=0.03333,
        shin=0.03333,
        ankle_to_sole=0.03333 + 0.0345,  # ankle pitch -> ankle roll -> sole
        ankle_roll=True,
        n_head=4,
        gait=dict(freq=1.5, step_len=0.008, lift=0.006, sway=0.014, sway_lead=0.0, hip_height=0.12,
                  foot_x=-0.02, turn_yaw=0.4, stride_bias=0.0, steer_gain=0.3, steer_damping=0.0),
    ),
    "mini14": Robot(
        # New design with what we learned from Microduck (model/build_mini14.py, LESSONS.md)
        name="mini14",
        scene=MODEL_DIR / "scene_mini14.xml",
        roll_to_pitch=0.024,
        thigh=0.045,
        shin=0.045,
        ankle_to_sole=0.022,
        ankle_roll=False,
        n_head=4,
        target_speed=0.08,
        rl_log_std=-2.0,
        servo_speed=10.0,  # XL330-M288 at 7.4 V (~16 rad/s with no load; margin for load)
        rl_head=True,
        # Soft servos: the reference corrects the tilt with the ankle (balance) and barely
        # moves forward on its own; moving forward is what the RL policy learns.
        gait=dict(freq=2.0, step_len=0.008, lift=0.009, sway=0.0, sway_lead=0.0, hip_height=0.1,
                  foot_x=0.0, turn_yaw=0.3, stride_bias=0.0, steer_gain=0.2, steer_damping=0.0,
                  balance=0.6),
    ),
}

DEFAULT = os.environ.get("MINI_ROBOT", "mini16")


def get(name=None) -> Robot:
    if isinstance(name, Robot):
        return name
    return ROBOTS[name or DEFAULT]
