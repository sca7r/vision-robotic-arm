"""Train the grasp residual.

Soft actor critic with an episode length of one. SAC is chosen for sample
efficiency, which is the only thing that matters when a sample costs a minute of
simulated execution rather than a millisecond.

    python3 -m vision_arm_rl.train --steps 2000 --out models/residual.zip

There is no GPU in this: at one action per grasp the wall clock goes on the
simulator, not on gradients. Expect roughly a minute per step.
"""
import argparse
import sys

from stable_baselines3 import SAC

from .residual_env import ResidualGraspEnv


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=2000)
    ap.add_argument("--out", default="models/residual.zip")
    ap.add_argument("--seed", type=int, default=1,
                    help="keep this away from the seed eval.py uses, so the "
                         "policy is not graded on the placements it trained on")
    ap.add_argument("--tensorboard", default=None)
    ap.add_argument("--save-every", type=int, default=100)
    args = ap.parse_args(argv)

    env = ResidualGraspEnv(seed=args.seed)
    model = SAC(
        "MlpPolicy", env,
        learning_starts=50,      # a few dozen random corrections before learning
        batch_size=64,
        train_freq=1,
        gradient_steps=8,        # samples are expensive, so squeeze each one
        policy_kwargs={"net_arch": [64, 64]},
        seed=args.seed,
        tensorboard_log=args.tensorboard,
        verbose=1,
    )

    # Checkpoint often. A run of a few thousand attempts is hours long and the
    # stack underneath it is a simulator that can wedge; losing all of it to a
    # crash at step 1900 would be avoidable and infuriating.
    from stable_baselines3.common.callbacks import CheckpointCallback
    callback = CheckpointCallback(
        save_freq=args.save_every, save_path="models/checkpoints",
        name_prefix="residual")

    model.learn(total_timesteps=args.steps, callback=callback, log_interval=10)
    model.save(args.out)
    print(f"saved {args.out}")
    env.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
