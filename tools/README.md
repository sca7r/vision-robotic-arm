# Dev tools (not a ROS package, not built)

Throwaway-but-useful scripts from Phase 5 bring-up. Run with the workspace sourced
and, where noted, with the sim already running.

| Script | Needs | What it does |
|---|---|---|
| `find_look_pose.py <urdf>` | sim + `move_group` | Searches joint space for a look pose that frames 2+ cubes, **is collision-free** (`/check_state_validity`) and can actually reach them (`/compute_ik`). The collision filter is the one that was missing - see below. |
| `look_poses.py <urdf>` | nothing | The FK + pinhole-projection half alone, no ROS. Fast, but does NOT check self-collision, so its answers are not directly usable. |
| `spike_ik.py [colour] [standoff]` | sim + `move_group` + perception | Asks `/compute_ik` whether a grasp above a detected cube is solvable, across several yaws. |
| `reach_map.py <urdf> [n]` | nothing | Samples joint space and reports, per mount height, where a top-down grasp is geometrically possible. FK only, no self-collision check, so its areas are upper bounds. This is what chose the flat bolt-to-the-bench layout. |
| `weld_check.py [q1 or q1,..,q6]` | sim | Commands one trajectory and prints what each joint actually reached plus the base pose from `gz model`. Catches the silent failure where the action reports SUCCEEDED with a joint pinned at 0. |
| `check_accuracy.py [seconds]` | sim + perception | Compares `/detections` against Gazebo ground truth, transformed through the live base pose from `gz model -m vision_arm -p`. |
| `sweep.py` | sim + perception | Sweeps joint1 and shows the world model accumulating cubes across views. |

Get the expanded URDF with:

    xacro src/vision_arm_description/urdf/vision_arm.urdf.xacro camera:=true > /tmp/arm.urdf

**Read this before trusting a look pose:** a pose that frames cubes can still be
self-colliding. The pose `[-1.213, 0.1599, 0.0012, -2.4671, -1.5579, 3.012]` frames
green and blue and looks fine in the camera image, but MoveIt reports
`link4_forearm` 62 mm inside `gripper` - so every IK query from it fails with
`avoid_collisions=True` (error -31) and nothing can be planned. Always run the
`/check_state_validity` filter.
