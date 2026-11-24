"""Demo of Mini13 / Mini16 walking.

Usage:
  python run_demo.py                      # simulates 12 s and prints results (Mini16)
  python run_demo.py --robot mini13       # the 13-motor one (no ankle roll)
  python run_demo.py --video robot.mp4    # also records a video (no display needed)
  python run_demo.py --viewer             # opens the interactive MuJoCo viewer
  python run_demo.py --turn 0.2           # turns left at 0.2 rad/s (negative = right)
"""

import argparse
import os

from gait import GaitParams


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--robot", default=None, help="mini16 (default) or mini13")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--freq", type=float, help="step cycles per second")
    ap.add_argument("--step", type=float, help="half-step length (m)")
    ap.add_argument("--lift", type=float, help="foot lift height (m)")
    ap.add_argument("--turn", type=float, default=0.0, help="turn rate in rad/s (+ left), e.g. 0.2")
    ap.add_argument("--video", help=".mp4 or .gif path to record to")
    ap.add_argument("--viewer", action="store_true", help="interactive viewer")
    args = ap.parse_args()

    params = GaitParams(robot=args.robot, freq=args.freq, step_len=args.step, lift=args.lift, turn=args.turn)

    if args.viewer:
        run_viewer(params)
        return

    on_frame, finish = (None, None)
    if args.video:
        on_frame, finish = make_recorder(args.video)

    from common import simulate

    out = simulate(params, args.seconds, on_frame)
    if finish:
        finish()

    print(f"Moved forward {out['x'] * 100:.0f} cm (lateral drift {out['y'] * 100:.0f} cm) in {out['time']:.0f} s")
    print(f"Maximum torso tilt: {out['max_tilt_deg']:.1f}°, total turn: {out['yaw_deg']:.0f}°")
    print(f"Leg torque (p99): {out['p99_leg_torque']:.2f} N·m out of 1.9 N·m")
    print("Fell at %.1f s" % out["fell_at"] if out["fell_at"] else "Did not fall")


def make_recorder(path):
    os.environ.setdefault("MUJOCO_GL", "egl")
    import imageio.v2 as imageio
    import mujoco

    frames = []
    state = {}

    def on_frame(model, data):
        if "r" not in state:
            state["r"] = mujoco.Renderer(model, 480, 640)
            cam = mujoco.MjvCamera()
            cam.distance, cam.azimuth, cam.elevation = 0.75, 150, -12
            state["cam"] = cam
        cam = state["cam"]
        cam.lookat[:] = [data.qpos[0], data.qpos[1], data.qpos[2] + 0.025]
        state["r"].update_scene(data, camera=cam)
        frames.append(state["r"].render())

    def finish():
        if path.endswith(".gif"):
            imageio.mimsave(path, frames[::2], duration=1 / 15, loop=0)
        else:
            imageio.mimsave(path, frames, fps=30)
        print(f"Video saved to {path}")

    return on_frame, finish


def run_viewer(params):
    import time

    import mujoco
    import mujoco.viewer

    import numpy as np

    from common import CONTROL_HZ, load, torso_pitch_roll, yaw
    from gait import Steering, targets

    model, data = load(params.robot)
    steering = Steering(params)
    yaw_total, prev_yaw = 0.0, yaw(data)
    every = int(round(1 / (CONTROL_HZ * model.opt.timestep)))
    with mujoco.viewer.launch_passive(model, data) as v:
        i = 0
        while v.is_running():
            start = time.time()
            if i % every == 0:
                pitch, roll = torso_pitch_roll(data)
                y = yaw(data)
                yaw_total += (y - prev_yaw + np.pi) % (2 * np.pi) - np.pi
                prev_yaw = y
                p_now = steering.update(yaw_total, 1 / CONTROL_HZ)
                data.ctrl[:] = targets(data.time, p_now, pitch, roll)
            mujoco.mj_step(model, data)
            i += 1
            if i % 10 == 0:
                v.sync()
            time.sleep(max(0, model.opt.timestep - (time.time() - start)))


if __name__ == "__main__":
    main()
