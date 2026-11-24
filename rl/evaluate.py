"""Compares the scripted gait against the trained policy on each terrain.

Usage:
  python evaluate.py --policy runs/mix/policy.zip                  # results table
  python evaluate.py --policy runs/mix/policy.zip --video out_dir  # + side-by-side videos
  python evaluate.py --policy mini16_policy.onnx                   # also accepts ONNX

Success = does not fall and moves forward at least 30 cm (on steps: climbs at least the first one).
Each terrain is tested with several seeds, with random friction/mass/servos and pushes.
"""

import argparse
from pathlib import Path

import numpy as np

from env import TERRAINS, MiniEnv

SUCCESS_PROGRESS = 0.3


ROBOT = None  # set with --robot


def n_leg():
    """Number of motors moved by the RL policy (legs, plus head if the robot has rl_head)."""
    import robots

    rb = robots.get(ROBOT)
    return rb.n_act if rb.rl_head else rb.n_leg


def load_policy(path):
    if path is None:
        return lambda obs: np.zeros(n_leg(), np.float32)  # scripted gait alone
    if str(path).endswith(".onnx"):
        import onnxruntime as ort

        sess = ort.InferenceSession(str(path))
        name = sess.get_inputs()[0].name
        return lambda obs: sess.run(None, {name: obs[None].astype(np.float32)})[0][0]
    from stable_baselines3 import PPO

    model = PPO.load(path, device="cpu")
    return lambda obs: model.predict(obs, deterministic=True)[0]


def run_episode(policy, terrain, seed, frames=None, seconds=10.0):
    env = MiniEnv(terrain=[terrain], robot=ROBOT, episode_seconds=seconds, seed=seed)
    obs, _ = env.reset(seed=seed)
    fell = False
    while True:
        obs, _, term, trunc, info = env.step(policy(obs))
        if frames is not None and env.t % 2 == 0:  # 25 fps
            frames.append(env.render())
        if term:
            fell = True
            if frames is None:
                break
            # keep recording a little to show the fall
            for _ in range(25):
                env.step(np.zeros(env.n_ctrl))
                frames.append(env.render())
            break
        if trunc:
            break
    return {"progress": info["progress"], "fell": fell, "ok": (not fell) and info["progress"] >= SUCCESS_PROGRESS}


def evaluate(policy, terrains, seeds):
    rows = {}
    for t in terrains:
        res = [run_episode(policy, t, s) for s in seeds]
        rows[t] = (np.mean([r["ok"] for r in res]), np.mean([r["progress"] for r in res]))
    return rows


def label(t):
    kind, v = t
    return {"flat": "Flat", "ramp": f"Ramp {v}°", "step": f"Steps {v * 100:g} cm"}[kind]


def side_by_side_video(policies, names, terrain, path, seed=0):
    import imageio.v2 as iio
    from PIL import Image, ImageDraw, ImageFont

    font = lambda s: ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", s)  # noqa: E731
    cols, outs = [], []
    for pol in policies:
        fr = []
        outs.append(run_episode(pol, terrain, seed, frames=fr))
        cols.append(fr)
    n = max(len(c) for c in cols)
    W, H = 480, 360
    video = []
    for i in range(n):
        im = Image.new("RGB", (W * len(cols), H + 96), (15, 15, 18))
        dr = ImageDraw.Draw(im)
        dr.text((12, 8), label(terrain), fill=(255, 255, 255), font=font(30))
        for j, c in enumerate(cols):
            im.paste(Image.fromarray(c[min(i, len(c) - 1)]), (j * W, 96))
            dr.text((j * W + 10, 52), names[j], fill=(230, 230, 230), font=font(22))
            if i >= len(c) - 1 or i > n - 40:
                o = outs[j]
                txt, col = (("fell", (230, 80, 80)) if o["fell"] else
                            (("made it", (90, 200, 90)) if o["ok"] else ("stuck", (240, 170, 60))))
                dr.text((j * W + 330, 52), txt, fill=col, font=font(22))
        video.append(np.array(im))
    iio.mimsave(path, video, fps=25, macro_block_size=1)
    return outs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--policy", help="policy.zip or .onnx (without it: only the scripted gait)")
    ap.add_argument("--robot", default=None, help="mini16 (default), mini13 or mini14")
    ap.add_argument("--terrain", default="mix", choices=list(TERRAINS), help="terrains to evaluate")
    ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--video", help="folder to save comparison videos to")
    args = ap.parse_args()
    global ROBOT
    ROBOT = args.robot

    terrains = TERRAINS[args.terrain]
    seeds = list(range(100, 100 + args.seeds))
    base = evaluate(load_policy(None), terrains, seeds)
    pol = load_policy(args.policy) if args.policy else None
    trained = evaluate(pol, terrains, seeds) if pol else None

    print(f"{'Terrain':<18}{'Scripted':>22}{'With RL':>22}")
    for t in terrains:
        b = f"{base[t][0] * 100:3.0f}% ({base[t][1] * 100:4.0f} cm)"
        r = f"{trained[t][0] * 100:3.0f}% ({trained[t][1] * 100:4.0f} cm)" if trained else "-"
        print(f"{label(t):<18}{b:>22}{r:>22}")

    if args.video and pol:
        out = Path(args.video)
        out.mkdir(parents=True, exist_ok=True)
        vids = [("ramp", 15), ("ramp", 20), ("step", 0.01), ("step", 0.015)]
        for t in [v for v in vids if v in terrains] or terrains:
            name = f"{t[0]}_{t[1]}".replace(".", "_")
            side_by_side_video([load_policy(None), pol], ["Scripted gait", "With RL"], t,
                               str(out / f"{name}.mp4"))
            print("video:", out / f"{name}.mp4")


if __name__ == "__main__":
    main()
