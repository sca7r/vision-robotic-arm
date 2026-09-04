# Dev tools (not a ROS package, not built)

Diagnostic scripts. Run with the workspace sourced and, where noted, with the
simulation already running.

| Script | Needs | What it does |
|---|---|---|
| `fake_detections.py` | sim | Publishes `/detections` and `/ground_truth` from Gazebo's pose stream, so pick and place can be tested without the camera (which costs about 45 s per frame on a box with no GPU). `--bias-seed N --noise M` makes it lie the way a miscalibrated camera does. This one is load bearing: the RL bench and evaluation both run against it. |
| `check_accuracy.py [seconds]` | sim + perception | Compares `/detections` against Gazebo ground truth, transformed through the live base pose from `gz model -m vision_arm -p`. |
| `find_look_pose.py <urdf>` | sim + `move_group` | Searches joint space for a look pose that frames two or more cubes, is collision free (`/check_state_validity`), and can actually reach them (`/compute_ik`). This is what chose the home pose in `poses.yaml`. Re-run it if the collision geometry changes. |
| `weld_check.py [q1 or q1,..,q6]` | sim | Commands one trajectory and prints what each joint actually reached plus the base pose from `gz model`. Catches the silent failure where the action reports SUCCEEDED with a joint pinned at 0. |
| `reach_map.py <urdf> [n]` | nothing | Samples joint space and reports, per mount height, where a top-down grasp is geometrically possible. Forward kinematics only, no self-collision check, so its areas are upper bounds. This is what chose the flat bolt-to-the-bench layout. |

Get the expanded URDF with:

    xacro src/vision_arm_description/urdf/vision_arm.urdf.xacro camera:=true > /tmp/arm.urdf

**Read this before trusting a look pose:** a pose that frames the cubes can still be
self colliding. The pose `[-1.213, 0.1599, 0.0012, -2.4671, -1.5579, 3.012]` frames green
and blue and looks fine in the camera image, but MoveIt reports `link4_forearm` 62 mm
inside `gripper`, so every IK query from it fails with `avoid_collisions=True` (error -31)
and nothing can be planned. Always run the `/check_state_validity` filter.
