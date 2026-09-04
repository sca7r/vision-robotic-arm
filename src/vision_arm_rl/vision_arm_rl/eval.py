"""Measure grasp success, with a policy or without one.

With no checkpoint this is phase 0, the gate: how often does the SCRIPTED grasp
work over randomised placements? If it already works nearly every time there is
nothing for a residual to learn and the feature is decoration.

With a checkpoint it is phase 4: the same placements, same seed, same code path,
with the policy in the loop. Comparing the two numbers is the only evidence that
the policy earns its place.

    python3 -m vision_arm_rl.eval --trials 60 --out baseline.csv
    python3 -m vision_arm_rl.eval --trials 60 --model models/residual.zip --out policy.csv
"""
import argparse
import csv
import math
import sys

import numpy as np

from .residual_env import ResidualGraspEnv


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--trials", type=int, default=60)
    ap.add_argument("--out", default="eval.csv")
    ap.add_argument("--seed", type=int, default=0,
                    help="same seed gives the same placements, which is what "
                         "makes baseline and policy runs comparable")
    ap.add_argument("--model", default=None, help="checkpoint; omit for the baseline")
    ap.add_argument("--radius", type=float, nargs=2, metavar=("LO", "HI"),
                    help="sample this distance band from the base instead of the "
                         "default working envelope")
    args = ap.parse_args(argv)

    policy = None
    if args.model:
        from stable_baselines3 import SAC
        policy = SAC.load(args.model)

    env = ResidualGraspEnv(seed=args.seed, radius=args.radius)
    rows, wins, skipped, consecutive = [], 0, 0, 0
    for i in range(args.trials):
        try:
            obs, _ = env.reset()
            consecutive = 0
        except RuntimeError as exc:
            # One trial the rig could not set up is not a robot failure and must
            # not be scored as one, but it must not end the run either. Only a
            # run of them means the stack is actually down.
            skipped += 1
            consecutive += 1
            print(f"trial {i}: {exc}, skipping", flush=True)
            if consecutive >= 5:
                print("five setups failed in a row, stopping")
                break
            continue
        action = (np.zeros(3, dtype=np.float32) if policy is None
                  else policy.predict(obs, deterministic=True)[0])
        _, reward, _, _, info = env.step(action)

        wins += info["success"]
        x, y = info["start"][:2]
        rows.append({
            "trial": i, "x": round(x, 4), "y": round(y, 4),
            "radius": round(math.hypot(x, y), 4),
            "dx_mm": round(1000 * info["residual_m"][0], 1),
            "dy_mm": round(1000 * info["residual_m"][1], 1),
            "dz_mm": round(1000 * info["residual_m"][2], 1),
            "success": int(info["success"]),
            "progress": round(info["progress"], 3),
            "reward": round(reward, 3),
            "result": info["verdict"],
        })
        print(f"trial {i:3d}  ({x:+.3f}, {y:+.3f})  r={math.hypot(x, y):.3f}  "
              f"d=({rows[-1]['dx_mm']:+5.1f},{rows[-1]['dy_mm']:+5.1f},"
              f"{rows[-1]['dz_mm']:+5.1f})mm  "
              f"{'ok  ' if info['success'] else 'FAIL'}  {wins}/{i + 1}", flush=True)

    if rows:
        with open(args.out, "w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)

    n = len(rows)
    label = "policy" if policy else "scripted baseline"
    if skipped:
        print(f"({skipped} trials skipped before they started, not scored)")
    print(f"\n{label} success {wins}/{n}" + (f" = {100 * wins / n:.0f}%" if n else ""))
    print(f"rows written to {args.out}")
    if policy is None and n and wins / n >= 0.95:
        print("\nGATE: the scripted grasp is already near perfect on this "
              "distribution.\nA residual has nothing to learn from it. Inject "
              "calibration error (phase 1)\nbefore training anything.")
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
