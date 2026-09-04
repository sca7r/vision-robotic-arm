# vision-arm-ros2

A vision guided pick and place system for a 6 DOF robotic arm, built on ROS 2 Jazzy and
Gazebo Harmonic. You send it a short command such as `red left`, and the arm looks at the
bench, finds the red cube, picks it up, puts it down where you asked, and returns to its
starting pose.

![Arm and gripper model in RViz](docs/images/visionarm.png)

The arm is a 3D printed stepper driven design with a parallel two jaw gripper and a wrist
mounted RGB-D camera. It had no ROS support of any kind, so this repository builds the
whole stack from the URDF upwards: model, simulation, control, motion planning, perception,
and task sequencing. Every layer is written so the same code drives real hardware once a
`ros2_control` hardware interface is added underneath it.

On top of that stack sits a reinforcement learning layer. A small policy learns a bounded
correction to the grasp pose the motion planner is asked to reach, so the arm can
compensate for the camera to base calibration error that real hardware always has.

## Contents

- [Status](#status)
- [Quick start](#quick-start)
- [Telling the robot what to do](#telling-the-robot-what-to-do)
- [How the pick and place cycle works](#how-the-pick-and-place-cycle-works)
- [Learning the grasp correction](#learning-the-grasp-correction)
- [System architecture](#system-architecture)
- [Packages](#packages)
- [Configuration](#configuration)
- [Engineering notes](#engineering-notes)
- [Command reference](#command-reference)
- [Testing and CI](#testing-and-ci)
- [Repository layout](#repository-layout)
- [Known limitations](#known-limitations)
- [Roadmap](#roadmap)

## Status

| Capability | State |
|---|---|
| URDF model, joint limits, inertials | Working, 9 tests |
| Gazebo simulation with `ros2_control` | Working, runs at roughly realtime |
| Arm bolted rigidly to a workbench | Working, zero drift measured |
| MoveIt 2 planning and collision checking | Working |
| RGB-D cube detection in `base_link` | Working, accuracy measured at 1.5 to 2.4 mm |
| Pick and place cycle | Working, placement accuracy 1 to 5 mm |
| Grasp success, 60 randomised placements | 65% full range, 64% working envelope, 62% with a 6.4 mm detection bias |
| Camera to grasp in one loop | Not yet run end to end, see Known Limitations |
| Learned grasp residual | Built and measured against, not yet trained |
| Natural language commands | Not started |
| Real hardware | Not started |

63 tests across six packages, all passing: description 9, gazebo 3, moveit_config 4,
perception 4, rl 10, tasks 33.

## Quick start

### Prerequisites

Ubuntu 24.04 with ROS 2 Jazzy:

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-moveit \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-vision-msgs \
  python3-colcon-common-extensions
```

For the learning layer only:

```bash
pip install gymnasium stable-baselines3
```

### Build

```bash
git clone <this-repo> vision-robotic-arm
cd vision-robotic-arm
colcon build --symlink-install
source install/setup.bash
```

### Watch it pick and place

Four terminals, each with `source install/setup.bash` first.

```bash
# 1. simulation, controllers and the sensor bridge
ros2 launch vision_arm_gazebo spawn.launch.py

# 2. MoveIt planning, once the first terminal reports
#    "Configured and activated gripper_controller"
ros2 launch vision_arm_moveit_config move_group.launch.py

# 3. object positions. Either the real detector:
ros2 launch vision_arm_perception perception.launch.py
#    or, on a machine without a GPU, the ground truth stand in:
python3 tools/fake_detections.py

# 4. the task node
ros2 run vision_arm_tasks task_server.py
```

Then send it work:

```bash
ros2 topic pub --once /task/command std_msgs/String "data: 'red left'"
ros2 topic pub --once /task/command std_msgs/String "data: 'blue 0.05 -0.55'"
ros2 topic echo /task/result
```

A full cycle takes about 60 seconds of simulated time. The arm starts at home, moves 10 cm
above the cube, descends, closes the jaws, lifts, carries, lowers, releases, and returns
home.

## Telling the robot what to do

### Anatomy of the command

```
ros2 topic pub  --once  /task/command  std_msgs/String  "data: 'blue 0.05 -0.55'"
```

| Part | What it is |
|---|---|
| `ros2 topic pub` | Publishes a message onto a topic, which is ROS 2's broadcast channel |
| `--once` | Send a single message and exit. Without it the command republishes at 1 Hz forever and the arm repeats the same job |
| `/task/command` | The topic `task_server.py` is listening on |
| `std_msgs/String` | The message type. It has to match what the node expects |
| `"data: 'blue 0.05 -0.55'"` | The message itself, written as YAML. `String` has one field called `data`. The outer quotes are for your shell, the inner ones keep the value as a single string |

Only the text inside the inner quotes changes from one command to the next.

### The two command forms

```
'<colour> <place>'      e.g.  'red left'          a named destination
'<colour> <x> <y>'      e.g.  'red 0.15 -0.40'    explicit coordinates
```

Colours are `red`, `green` and `blue`. Named places live in
`src/vision_arm_tasks/config/poses.yaml` and are currently `left`, `right` and `front`.

### Coordinates

`x` and `y` are metres in `base_link`, the frame at the arm's base.

- **x** runs left and right across the bench. Negative is left, positive is right, 0 is
  straight ahead.
- **y** is distance out from the robot. It is always negative, because the workspace is in
  front of the base. Something at `-0.30` is close in, something at `-0.55` is far out.
- There is no **z**. Objects sit on the bench, at the height they were picked from.

Put objects between 0.40 and 0.56 m from the base. That is the measured working envelope,
and the reason it is smaller than the geometric reach is in
[Engineering notes](#the-usable-workspace-is-smaller-than-the-geometry-suggests).

### The bench, to scale

One character is 2.5 cm. `R`, `G` and `B` are the three cubes in their starting positions,
and `o` marks each named place.

```
  +---------------------------------------------------------+  y = +0.15   near edge of the bench
  |                    [ROBOT]                              |  y =  0.00   base_link origin, x = 0, y = 0
  |                      #######                            |              base plate footprint
  |                                                         |
  |          ................................               |  y = -0.19   near edge of reach
  |          .                              .               |
  |          .   o  R    G    B  o          .               |  y = -0.42   cubes R G B, places 'left' and 'right'
  |          .                              .               |
  |          .           o                  .               |  y = -0.55   place 'front'
  |          ................................               |  y = -0.63   far edge of reach
  |                                                         |
  +---------------------------------------------------------+  y = -0.85   far edge of the bench
         |       |       |       |       |       |       |
        -0.4    -0.2    +0.0    +0.2    +0.4    +0.6    +0.8   x in metres
```

The dotted rectangle is where a straight down grasp is geometrically possible. Outside it
the command comes back as a failure without the arm moving.

### Adding your own named places

Edit `src/vision_arm_tasks/config/poses.yaml`:

```yaml
places:
  left: [-0.20, -0.42]
  right: [0.20, -0.42]
  front: [0.00, -0.55]
  bin: [0.30, -0.50]        # a new one
```

Rebuild with `colcon build --symlink-install`, restart the task server, and `'red bin'`
works. A misspelled name is rejected along with the list of valid ones, rather than being
turned into a motion.

To point the node at a different set of poses without touching the installed file:

```bash
ros2 run vision_arm_tasks task_server.py --ros-args -p poses_file:=/path/to/other.yaml
```

### Reading the reply

```bash
ros2 topic echo /task/result
```

Three kinds of reply:

```
picking red at (-0.120, -0.425, 0.041) -> (-0.200, -0.420)
done: red is at (-0.198, -0.422), 3 mm from target
failed: unknown place 'lft', known: ['front', 'left', 'right']
```

The `done` line reports where the cube actually ended up, re-detected from the home pose
after the move, not where the arm was asked to put it. If those disagree by more than
`place_tolerance` you get a `failed` line instead, which is how a grasp that closed on
nothing gets caught.

## How the pick and place cycle works

### The cycle

The system is one repeating loop, anchored on a single home pose:

```
   home  ->  detect  ->  plan  ->  approach  ->  grasp
     ^                                             |
     |                                             v
   home  <-  release  <-  lower  <-  carry  <-  lift
```

The home pose is six joint angles chosen so that every object on the bench is inside the
wrist camera's frame, the arm is not in collision with itself, and every object is
reachable from there. Because the arm returns to it after every job, three properties fall
out:

- Every job starts from an identical, known state, so results are comparable run to run.
- The scene is re-detected at the start of every job, so objects moved by hand between jobs
  are picked up automatically. This is observable: during testing, carrying one cube nudged
  its neighbour 17 mm sideways, and the next command grasped that neighbour at its new
  position without being told.
- Changing the environment means editing six numbers in a YAML file, not editing code.

The pose in use is `[-1.1215, 0.047, 0.7946, 1.026, 0.6052, -1.0986]`. It was not chosen by
hand. `tools/find_look_pose.py` searched joint space for a configuration passing three
filters in increasing order of cost: forward kinematics plus a pinhole camera projection to
check the cubes are in frame, `/check_state_validity` to check the arm is not folded into
itself, and `/compute_ik` to check each cube can actually be reached from there.

### Data flow

```
  Gazebo Harmonic
      |  RGB image, depth image, camera info      (ros_gz bridge)
      v
  cube_detector  (vision_arm_perception)
      |  HSV segmentation -> blob centroid -> depth patch median
      |  -> deprojection through camera intrinsics -> TF into base_link
      |
      |  /detections   (vision_msgs/Detection3DArray, frame base_link)
      v
  task_server  (vision_arm_tasks)
      |  colour -> cube position, destination -> place position
      |  /grasp_residual  optional learned correction, clipped on arrival
      |  /compute_ik      Cartesian waypoint -> joint angles
      |  /move_action     plan and execute, collision aware
      |  /gripper_controller/commands   jaw positions
      v
  ros2_control -> joint_trajectory_controller -> Gazebo (or real hardware)
```

The detector holds a world model, a map from colour to the last known position and the
timestamp it was seen at. Objects therefore stay in `/detections` after they leave the
camera frame, which is what makes a single look pose sufficient.

### Motion

MoveIt is driven from plain `rclpy`. There is no `moveit_py` and no MoveIt Task
Constructor, because neither is installed on the target platform and MTC on Jazzy needs a
patched fork. A pick is five moves, so the services are called directly:

- `/compute_ik` turns a Cartesian target into joint angles.
- `/check_state_validity` reports named contact pairs, which is what makes collision
  problems debuggable.
- `/move_action` plans a collision free path to a joint configuration and executes it.
- The gripper is not in MoveIt. Jaw positions are published as `[x, -x]` on
  `/gripper_controller/commands`.

Two details in the IK path exist because of failures found by running the system, not by
reading it:

1. **Every waypoint is solved with the jaw position it will be executed with.** Closing the
   jaws on a cube swings the jaw meshes about 3 cm, and they then contact the forearm's
   collision volume. Validating with open jaws and executing with closed ones made the
   place descent fail while the arm hung in the air holding a cube.
2. **IK is retried across seeds and jaw yaws.** KDL is a local numeric solver, so it
   succeeds or fails depending on where it starts, and `/compute_ik` seeds from wherever
   the arm happens to be standing. The same grasp point solved after one approach and
   failed after another. The solver now tries the current state, the home pose, six
   perturbations of home, and four yaw angles. A cube is square, so which way the jaws
   close is free.

All waypoints are solved before the arm leaves home. A destination that turns out to be
unreachable therefore costs nothing, instead of stranding the arm mid cycle holding an
object.

The result is verified rather than assumed. A gripper that closes on nothing, or an object
that slips during the lift, produces exactly the same sequence of successful moves as a
real pick. After returning home the node re-detects the object and compares its position
against the requested destination. This is not hypothetical: the first full run reported
three successes when one cube had never left its starting position.

## Learning the grasp correction

### The problem it solves

The arm computes where to reach from where the camera says the cube is. That calculation
inherits every millimetre of error in the camera to base transform, and on real hardware
that transform is never exact. A fixed 5 mm offset is the difference between closing the
jaws on a cube and knocking it across the bench, and no amount of care in the motion
planner helps, because the planner is faithfully reaching for the wrong point.

The error cannot be calibrated away once and forgotten either. It drifts when the camera is
bumped, when the mount flexes, when the arm is reassembled.

### What is learned

Three numbers: an offset in x, y and z applied to the grasp point, each clipped to 15 mm.
The policy is a small multilayer perceptron that reads the detected cube position and
outputs that offset. Nothing else in the pipeline changes.

One grasp attempt is one action followed immediately by its reward, with no state carried
between attempts, so this is a contextual bandit rather than a policy with a planning
horizon, and it is trained as one. That is the smallest formulation that fits: an attempt
costs the better part of a minute of simulated execution, which affords a few thousand
samples and not the hundreds of thousands a step by step controller would need. Soft actor
critic is used for its sample efficiency, which is the only property that matters at that
price per sample.

The reward grades near misses rather than scoring pass or fail:

| Term | Value |
|---|---|
| Fraction of the distance to the target the cube actually travelled | 0 to +1 |
| Cycle succeeded, cube placed within tolerance | +1 |
| Inverse kinematics or planning failure on the corrected pose | -0.25 |
| Mean magnitude of the correction | -0.05 scaled |

Progress is measured from Gazebo ground truth, start position against final position
relative to the target. It is what separates "the jaws closed on nothing", which scores
zero, from "the grasp slipped near the end", which scores most of the way. That is what
carries a gradient on attempts that fail. The magnitude penalty keeps the correction near
zero where the scripted grasp is already right, so the policy acts only where the script is
biased.

### Why this is safe

The correction arrives on `/grasp_residual` as three floats, and `task_server` clips them
to the envelope before use. The clip lives there rather than in whatever publishes, because
a policy that is undertrained, mid-training, or simply wrong is exactly the case it has to
hold for. A message that is not three finite numbers is dropped rather than partially
applied. MoveIt still plans and collision checks the corrected pose. Within 15 mm the worst
a bad correction can do is miss the cube, which is a failure the scripted grasp already
has.

Nothing publishes to that topic unless the policy node is running, so the default behaviour
of the system is the scripted grasp, unchanged. The same topic is what the training
environment drives, so training and deployment exercise one code path rather than two that
can drift apart.

### Injecting the error there is something to learn from

In simulation the detections come from ground truth, so the aim is already perfect and a
residual has nothing to correct. `tools/fake_detections.py` therefore lies on purpose:

```bash
python3 tools/fake_detections.py --cubes red --bias-seed 3 --noise 0.002
```

`--bias-seed` draws a fixed offset once and applies it to every detection for the life of
the process, which is what a calibration error actually looks like: not noise, a constant
the arm cannot see and cannot average away. `--noise` adds per-frame jitter on top. Two
topics come out. `/detections` is what the robot is allowed to believe and carries both.
`/ground_truth` carries neither, and exists so the bench can grade an attempt without
asking Gazebo separately.

**The injected error has to be big enough to hurt, and seed 3 is not.** That seed draws
(-6.3, +1.1, 0.0) mm, and scoring 60 randomised placements through it gives 37/60, against
64% with no bias at all. The standard error at n=60 is 6.3 points, so those two numbers are
the same number. The reason is the gripper: the jaws open to 10.4 cm around a 5 cm cube, so
there is about 2.7 cm of clearance per side and a 6 mm aiming error is absorbed by it.

The bias has to sit near the top of the correctable range to be worth training against: big
enough that the scripted grasp measurably loses success, still inside the 15 mm the
residual is allowed to move. `--bias-mag` sets the half-width of the draw, so

    python3 tools/fake_detections.py --cubes red --bias-seed 3 --bias-mag 0.014 --noise 0.002

is the shape of it. Re-measure the baseline through whatever bias is chosen before
training, because that baseline, not the unbiased one, is the number the policy has to
beat.

### The workflow

```bash
# 1. Measure what the scripted grasp already does. This is the gate: if it rarely
#    fails there is nothing to learn, and eval.py says so.
python3 -m vision_arm_rl.eval --trials 60 --out baseline.csv

# 2. Give it something to learn: the calibration error real hardware has.
python3 tools/fake_detections.py --cubes red --bias-seed 3 --noise 0.002

# 3. Train. No GPU: at one action per grasp the wall clock goes on the simulator,
#    not on gradients.
python3 -m vision_arm_rl.train --steps 2000 --out models/residual.zip

# 4. Score it against the baseline, same seed so the placements are identical.
python3 -m vision_arm_rl.eval --trials 60 --model models/residual.zip --out policy.csv

# 5. Deploy, if and only if step 4 beat step 1.
ros2 run vision_arm_rl policy_node --ros-args -p model:=models/residual.zip
```

Steps 1 and 4 run the same code with and without a checkpoint, so the before and after
numbers are comparable by construction rather than by hope.

### Where it stands

Built, tested, and measured against. Not yet trained. See
[Known limitations](#known-limitations) for the honest ceiling on what training can buy.

Full design in
[`docs/superpowers/specs/2026-09-03-residual-rl-grasp-design.md`](docs/superpowers/specs/2026-09-03-residual-rl-grasp-design.md).

## System architecture

One ROS 2 package per concern. Layers communicate only through standard ROS 2 interfaces,
so each can be developed, tested, and replaced independently.

```
+---------------------------------------------------------------------+
|  vision_arm_rl                                                      |
|  learned residual on the grasp pose, trained against the simulator  |
+---------------------------------------------------------------------+
|  vision_arm_tasks                                                   |
|  home -> detect -> grasp -> place -> home, driven by string commands|
+---------------------------------------------------------------------+
|  vision_arm_moveit_config                                           |
|  MoveIt 2: KDL inverse kinematics, OMPL planning, collision matrix  |
+---------------------------------------------------------------------+
|  vision_arm_perception                                              |
|  RGB-D -> HSV segmentation -> 3D pose in base_link -> world model   |
+---------------------------------------------------------------------+
|  vision_arm_gazebo                                                  |
|  Gazebo Harmonic world, ros2_control controllers, sensor bridge     |
+---------------------------------------------------------------------+
|  vision_arm_description                                             |
|  URDF and xacro: arm, gripper, camera, workbench, inertials, limits |
+---------------------------------------------------------------------+
```

### Why it is built this way

- **Separation of concerns.** Perception knows nothing about motion planning. The task
  layer never writes arm joint commands directly, it goes through MoveIt.
- **Interfaces at the boundaries.** The perception contract is
  `vision_msgs/Detection3DArray` in `base_link`. Any detector honouring that contract, HSV
  today or a learned model later, is a drop in replacement. `tools/fake_detections.py`
  exploits exactly this to test motion without a camera.
- **Simulation and hardware parity.** Controllers are defined through `ros2_control`, so
  the Gazebo plugin and a future hardware interface expose identical command and state
  interfaces.
- **Configuration over code.** Home pose, named destinations, gripper positions,
  tolerances, HSV bands and controller gains all live in YAML.
- **Learning is additive, not a replacement.** The residual is bounded and clipped, and
  MoveIt still plans and collision checks the corrected pose. Switch the policy off, or let
  its correction fail IK, and the system falls back to the scripted grasp. The worst case
  of an untrained policy is the behaviour the system already has.
- **Verify, do not assume.** Every number in this README was measured against Gazebo ground
  truth, and the scripts that measured it are in `tools/` and `src/vision_arm_rl`.

## Packages

### vision_arm_description

The single source of truth for the robot model. A modular xacro builds the arm, mounts the
gripper on the flange, attaches the camera, and welds the whole thing to a workbench.

- Six revolute joints with position limits derived from the stepper firmware's step count
  bounds multiplied by each joint's gear ratio, then recentred on the pose the homing
  routine actually leaves the arm in. The model can only reach what the hardware can reach.
- Full inertial parameters per link. One of them, the base plate, had a CAD exported centre
  of mass lying outside its own mesh and outside its support polygon, which made the model
  tip over as soon as any mass was added. A test now asserts every link's centre of mass
  lies inside that link's mesh.
- Collision geometry is the mesh bounding box rather than the mesh itself. The full meshes
  are 278k triangles and grinding them against the ground plane every millisecond held the
  simulation at a real time factor of 0.006. The jaws keep their real meshes, because
  grasping needs real geometry.
- A `world` link and a fixed `world_joint` bolt `base_link` to the bench at `mount_z`.
- The workbench is a link in the URDF, not a model in the world SDF, so Gazebo and MoveIt
  share one definition of the surface the arm stands on. A planner that cannot see the
  bench will drive the arm straight through it.
- The wrist camera is off by default. Enable it with `camera:=true`.

### vision_arm_gazebo

Everything needed to run the robot in Gazebo Harmonic.

- The world: a ground plane, lighting, and three 5 cm cubes in a row on the bench. The
  bench itself comes from the URDF.
- Robot spawning through `gz_ros2_control`, so the simulated arm is driven by the same
  controller stack as real hardware.
- Controllers: `joint_state_broadcaster`, a `joint_trajectory_controller` for the arm, and
  a `JointGroupPositionController` for the gripper. Spawners are chained one at a time on
  process exit events, because activating them concurrently raced the controller manager's
  switch lock and killed each other.
- A `ros_gz` bridge exposing clock, RGB, depth, point cloud and camera info to ROS.

### vision_arm_moveit_config

MoveIt 2 configuration connecting planning to execution.

- Planning groups `arm` and `gripper`, with named states `home`, `open` and `closed`.
- The `arm` group is declared as a chain from `base_link` to `tcp`, not as a joint list.
  The joint list form ended the group at the flange, so IK solved for a point 140 mm short
  of where the gripper actually grasps, and every Cartesian goal was wrong by that much.
- KDL inverse kinematics and OMPL planning.
- A collision matrix that disables structurally adjacent pairs and three pairs measured to
  collide in 100 percent of configurations. See [Engineering
  notes](#three-link-pairs-made-every-state-invalid).

### vision_arm_perception

One node, `cube_detector.py`, turning RGB-D frames into object poses.

- HSV segmentation rather than a learned detector. The scene is three saturated primary
  colours on a desaturated background, which thresholding solves exactly and a network
  solves approximately.
- Blob centroid, median depth over a small patch to reject edge speckle, deprojection
  through the camera intrinsics, then a TF transform into `base_link`.
- A world model retaining each colour's last known position and the timestamp it was
  observed at. Each detection carries the stamp it was actually seen at, while the array
  header carries the current stamp.
- Publishes `/detections` and an annotated `/detections/debug` image.
- Accuracy measured against Gazebo ground truth at 1.5 to 2.4 mm.

### vision_arm_tasks

One node, `task_server.py`, running the pick and place cycle.

- Subscribes to `/task/command` as a `std_msgs/String` and replies on `/task/result`.
- Subscribes to `/grasp_residual` as three floats, clips them to 15 mm per axis, applies
  them to the pick waypoints only, and expires them after one cycle so a stale correction
  cannot bias the next command. It never loads a checkpoint and never imports torch.
- There is deliberately no custom action or interfaces package. Nothing needs cancellation
  or progress feedback yet, and a string is exactly what a language layer would publish.
  Swapping in an action later touches one node.
- All configuration lives in `config/poses.yaml`, selected with `-p poses_file:=...`.

### vision_arm_rl

The learning layer, and the only package here that trains a neural network. It does not
replace MoveIt, it corrects the pose MoveIt is asked to reach.

It also owns the measurement rig, which matters more than it sounds: `eval.py` scores the
scripted grasp and the learned one through exactly the same code, so the before and after
numbers are comparable by construction.

| Piece | What it is |
|---|---|
| `bench.py` | The ROS 2 side: place a cube, reset the arm, run one attempt, grade the result. Knows nothing about learning |
| `residual_env.py` | Gymnasium environment over `bench.py`, one grasp per step |
| `train.py` | Soft actor critic from Stable Baselines3, horizon 1 |
| `eval.py` | Success rate, with a checkpoint or without one |
| `policy_node.py` | Publishes corrections from a checkpoint at run time |
| `models/` | Trained checkpoints, not tracked in git |

Two details in `bench.py` were found the expensive way and are load bearing. Trials reset
the arm to home by commanding the controller directly, because a failed cycle strands the
arm and MoveIt refuses to plan out of a colliding start state, so the next trial would
record a failure it did not earn. And the bench waits until the detector reports the cube
where it was just placed rather than waiting a fixed interval, because a fixed wait
straddles a detector tick and the task server then grasps at the previous trial's location.

## Configuration

### `vision_arm_tasks/config/poses.yaml`

Everything that changes when the environment changes.

```yaml
home: [-1.1215, 0.047, 0.7946, 1.026, 0.6052, -1.0986]   # six joint angles
places:
  left: [-0.20, -0.42]      # x, y in base_link
  right: [0.20, -0.42]
  front: [0.00, -0.55]
open: 0.0                   # jaw position, 0 is fully open (10.4 cm)
grip: -0.035                # closes to 10.4 - 2*|grip| cm
grip_tilted: -0.040         # firmer squeeze when the approach is off vertical
place_tolerance: 0.03       # how far a cube may land before the job is failed
approach: 0.10              # standoff above an object for approach and retreat
```

### `vision_arm_perception/config/cube_detector.yaml`

```yaml
target_frame: base_link
min_area: 60                # pixels, a floor against speckle
depth_patch: 2              # median over a 5x5 patch
cube_size: 0.05             # used to offset from the front face to the centre
hsv:                        # h_lo s_lo v_lo h_hi s_hi v_hi, OpenCV 0-179 hue
  red: [0, 120, 70, 10, 255, 255, 170, 120, 70, 179, 255, 255]
  green: [40, 120, 70, 80, 255, 255]
  blue: [100, 120, 70, 130, 255, 255]
```

Red gets two bands because its hue wraps around zero.

## Engineering notes

The decisions that were expensive to arrive at and are cheap to undo by accident.

### The arm is bolted down, and this is load bearing

The model used to be free floating. Joint 1 only turned because the base counter rotated
underneath it, absorbing roughly half of every commanded angle: commanding minus 0.9 rad
yawed the base plus 0.46 rad. That is survivable for perception, which measures objects in
`base_link` and rides the base along with them, but it is fatal for pick and place, because
objects are bolted to the world rather than to the base. An object measured at a `base_link`
coordinate is no longer at that coordinate once the base has yawed 26 degrees mid motion.

The arm is now welded to the world with a fixed joint. Verified with `tools/weld_check.py`:
a six joint move tracks every joint to within 0.003 rad while `gz model -m vision_arm -p`
reports the model at exact world identity throughout.

### The layout was chosen by measurement, not by eye

`tools/reach_map.py` samples 400k joint configurations, keeps those within 15 degrees of
straight down, and reports where a top down grasp is geometrically possible for each
candidate mount height:

| Mount | Usable top down grasp area |
|---|---|
| Arm on the floor, objects on a 0.28 m table (the original layout) | none at all |
| Arm bolted flat to the work surface | about 2500 cm2, radius 0.21 to 0.65 m |
| On a 10 cm riser | about 1600 cm2 |
| On a 20 cm riser | about 1125 cm2 |
| On a 28 cm riser | about 675 cm2 |

So the arm is bolted flat and there is no pedestal, because every centimetre of riser
shrinks the reachable area. The long standing belief that top down grasps were unreachable
with this wrist was an artifact of the original layout.

### Three link pairs made every state invalid

Sampling 600 random configurations through `/check_state_validity` found that zero of them
were collision free. Three pairs collided in all 600:

```
link2_upper_arm <-> link4_forearm
link4_forearm   <-> link6_gripper_mount
link4_forearm   <-> gripper
```

All three are one link apart, and their collision geometry is the mesh bounding box. A long
forearm box necessarily overlaps its neighbours' boxes at every joint angle, so the check
was never meaningful. While it was enabled, no configuration in the robot's entire space
was valid, `/compute_ik` with `avoid_collisions` failed for every target including the
arm's own current pose, and no look pose could ever be found.

They are now disabled in the SRDF with `reason="Always"`, which is what MoveIt Setup
Assistant does with permanently colliding pairs. Collision free configurations went from
0 of 600 to 42 of 600, and a valid look pose was found on the first candidate checked.
Everything genuinely configuration dependent stays enabled, including `link4` against the
base at 49 percent, `link4` against `link1` at 71 percent, and every link against the
workbench. If collision geometry ever changes, re-run that survey before trusting anything.

### The usable workspace is smaller than the geometry suggests

`tools/reach_map.py` reports top down grasps as geometrically possible from radius 0.21 to
0.65 m. That ignores two things that decide it in practice: self collision, and whether KDL
actually finds the solution. Scoring 60 randomised placements across the geometric range
gave, by distance from the base:

| Radius | Success |
|---|---|
| 0.25 to 0.40 m | 2/6 |
| 0.40 to 0.48 m | 22/28 |
| 0.48 to 0.56 m | 13/23 |
| 0.56 to 0.65 m | 2/3 |

Close in is where it collapses, and the reason is in the next note. The measured working
envelope is 0.40 to 0.56 m, and that is what `vision_arm_rl/bench.py` samples.

### Straight down was a choice, not a limit, and tilting it did not pay

The IK path used to build the target orientation from a yaw angle alone, so the jaws always
pointed at the bench and the only freedom was the spin about the vertical. That was never a
wrist limit: joint5 has 213 degrees of travel, joint4 has 304, joint6 has 361. It was
simply the only thing ever asked for.

`plan_pick` now escalates to tilted approaches when straight down finds nothing, backing
the standoff off along the tool axis rather than straight up. Straight down is still tried
first and with the most seeds, so ordinary grasps are unchanged and cost the same.

It does not rescue the close in band, and the reason is worth recording so nobody spends
another afternoon on it. Measured over the 0.30 to 0.40 m band:

| Approach | Success |
|---|---|
| Straight down only | 2/6 |
| Tilts of 20, 35, 50 degrees | 3/6 |
| Tilts of 90, 60, 30 degrees, squeezing harder when tilted | 3/6 |

All three are the same number inside the noise. Two things came out of the attempt:

**A true side grasp is geometrically unavailable here.** The jaws close along the tool Y
axis and pitch turns about that same axis, so the jaw opening direction stays horizontal at
every tilt: straight down and straight sideways both put the flat plates on two opposite
vertical faces of a cube, and the angles between are the awkward ones. Sideways would be
the better grip, but the TCP sits 2.5 cm above the bench and the jaws hang 14 cm along the
tool axis, so a horizontal approach drives the lower finger through the workbench. MoveIt
has the bench in its collision model and rejects every 90 degree grasp. Only 60 is ever
selected.

**Tilt converts unreachable into reachable but dropped.** A 60 degree grasp plans and
flies, then reports that the jaws closed on nothing, because the finger geometry leans into
the bench and pushes the cube out as it closes. Squeezing from -0.035 to -0.040 did not fix
it.

Tilting is kept because it is free: straight down is tried first, so it costs nothing on
normal grasps and occasionally saves an otherwise impossible one. It is just not the answer
to the close in band. The answer there is not to put objects there.

### Run one simulator at a time

Two `gz sim` processes against the same world, or two `fake_detections.py` publishing
different biases onto `/detections`, do not announce themselves. The symptom is a joint
that sits pinned at its limit and ignores every trajectory while the other five track
normally, which reads as a broken URDF and is not. If the arm stops responding for no
reason, count the processes before debugging the model.

### Other notes

- **Joint limits come from firmware, not guesswork.** The original model declared every
  joint continuous and unlimited.
- **Controller startup is sequenced.** Spawners activate one at a time, chained on process
  exit events.
- **The camera needs an explicit sensor pose.** SDF cameras look along positive X, not
  negative Z. Getting this wrong put 41 percent of the deprojected point cloud below the
  floor.
- **Software rendering is forced only when headless.** On a machine without a GPU the
  simulation server needs `LIBGL_ALWAYS_SOFTWARE` and a surfaceless EGL platform to keep a
  usable real time factor, but those same variables make the Gazebo GUI exit within
  seconds, so they are applied only when `headless:=true`.

## Command reference

### Launch files

| Command | What it does |
|---|---|
| `ros2 launch vision_arm_description display.launch.py` | Model in RViz with joint sliders |
| `ros2 launch vision_arm_gazebo spawn.launch.py` | Simulation, controllers, sensor bridge |
| `ros2 launch vision_arm_gazebo spawn.launch.py headless:=true` | The same without the GUI, for scripted runs |
| `ros2 launch vision_arm_gazebo spawn.launch.py camera:=true` | With the wrist RGB-D camera enabled |
| `ros2 launch vision_arm_moveit_config move_group.launch.py` | MoveIt planning services |
| `ros2 launch vision_arm_moveit_config demo.launch.py` | MoveIt with the RViz planning plugin |
| `ros2 launch vision_arm_perception perception.launch.py` | The cube detector |
| `ros2 run vision_arm_tasks task_server.py` | The pick and place cycle |
| `ros2 run vision_arm_rl policy_node --ros-args -p model:=models/residual.zip` | Publish learned grasp corrections |

### Development tools

Not a ROS package and not built. Run with the workspace sourced. Details in
[`tools/README.md`](tools/README.md).

| Script | Needs | What it does |
|---|---|---|
| `tools/fake_detections.py` | sim | Publishes `/detections` and `/ground_truth` from Gazebo's pose stream, so motion can be tested without the camera. `--bias-seed N --noise M` makes it lie the way a miscalibrated camera does |
| `tools/check_accuracy.py [seconds]` | sim and perception | Compares `/detections` against ground truth |
| `tools/reach_map.py <urdf> [n] [cube]` | nothing | Where a top down grasp is possible, per mount height |
| `tools/find_look_pose.py <urdf>` | sim and `move_group` | Searches for a look pose passing all three filters |
| `tools/weld_check.py [q1 or q1,..,q6]` | sim | Commands one trajectory, prints what each joint reached and where the base ended up |

Get the expanded URDF that several of these want with:

```bash
xacro src/vision_arm_description/urdf/vision_arm.urdf.xacro camera:=true > /tmp/arm.urdf
```

## Testing and CI

```bash
colcon test && colcon test-result --verbose
# or, faster during development:
python3 -m pytest src -q
```

63 tests across six packages. They are deliberately weighted towards the things that fail
silently:

- the mount height and the bench top agree,
- every link's centre of mass lies inside its own mesh,
- the camera sensor pose matches the optical frame's claim,
- a malformed command is rejected rather than turned into a motion,
- a malformed or oversized `/grasp_residual` is dropped or clamped rather than applied,
- the tool axis the approach backs off along matches the one the grasp orientation implies,
- the residual limit in `bench.py` still matches the one `task_server.py` enforces.

None of them need the simulator. The environment tests inject a stub bench, so the
arithmetic is pinned down without spending wall clock on Gazebo.

`.github/workflows/ci.yml` builds every package and runs `colcon test` on push and pull
request, in a `ros:jazzy-ros-base` container. It installs `gymnasium` with pip because
rosdep does not package it. `stable_baselines3` and torch are deliberately left out:
nothing under test imports them, and pulling torch into CI costs minutes for no coverage.

## Repository layout

```
vision-robotic-arm/
  src/
    vision_arm_description/    URDF and xacro, meshes, RViz config, display launch
    vision_arm_gazebo/         world SDF, spawn launch, controller and bridge YAML
    vision_arm_moveit_config/  SRDF, kinematics, joint limits, MoveIt launches
    vision_arm_perception/     cube_detector node, HSV config, perception launch
    vision_arm_tasks/          task_server node, poses and places config
    vision_arm_rl/             residual policy env, training and evaluation scripts
  tools/                       development and measurement scripts
  docs/images/                 README media
  docs/superpowers/specs/      design documents
  .github/workflows/ci.yml     build and test on push
  LICENSE
```

## Known limitations

Stated plainly, because a README that only lists successes is not much use.

- **The camera to grasp loop has not been run end to end.** Perception is verified on its
  own, and the pick and place cycle is verified on its own using ground truth object
  positions from `tools/fake_detections.py`. They have not yet been run as one pipeline.
  The reason is practical: the development machine has no GPU, so software rendering takes
  about 45 seconds per RGB-D frame and stalls the physics step alongside it.
- **Perception accuracy was measured in the previous scene layout.** The 1.5 to 2.4 mm
  figure predates the move onto the workbench and the change to 5 cm objects. It has not
  been re-measured since.
- **Three link pairs are excluded from collision checking**, as described above. MoveIt
  cannot catch the forearm genuinely striking the gripper at an extreme wrist angle. The
  upgrade path, if that ever matters, is convex hull collision meshes for those links
  instead of bounding boxes, at a cost in planning and simulation speed.
- **The scene is deliberately easy.** Three identical cubes, well separated, in a row,
  inside the measured reachable area. This is a starting point chosen so that complexity
  can be added one variable at a time.
- **Grasping is position controlled and open loop.** The jaws are commanded to a fixed
  position and the result is checked afterwards by looking at the object again. There is no
  force feedback and no grasp quality estimation.
- **The grasp residual is not trained yet, and the error injection is not yet calibrated
  to make training worthwhile.** Three baselines are measured, all 60 randomised
  placements: 65% across the full geometric range, 64% inside the working envelope, and
  62% inside the working envelope with a 6.4 mm detection bias injected. The last two are
  within one standard error of each other, which means the bias currently being injected is
  not what makes the grasp fail. Raise `--bias-mag` toward the 15 mm the residual can
  actually cancel, re-measure, and train against that. The policy stays disabled by default
  until a paired evaluation shows it beating the scripted grasp on the same seed.
- **What actually fails has changed.** In the 60 trial biased run, every one of the 23
  failures was the jaws closing on nothing or losing the cube. There were no IK failures
  and no planner failures at all, because sampling the measured working envelope and
  falling back to a tilted approach removed them. Failures are also not uniform across that
  envelope: 20/23 succeeded between 0.40 and 0.46 m, 10/24 between 0.46 and 0.52 m, and
  7/13 beyond 0.52 m. The outer half of the declared envelope is the weak part, and it is
  worth understanding why before assuming a residual is the fix.
- **The close in band is not usable.** Below about 0.40 m a straight down grasp has to fold
  the arm back over itself, and tilted approaches do not rescue it. Objects belong in the
  0.40 to 0.56 m envelope.
- **No real hardware.** Everything here is simulation.

## Roadmap

1. **Calibrate the injected error, then train the grasp residual.** The measurement rig
   and the error injection are both in place, but at `--bias-mag 0.012` the drawn bias does
   not measurably hurt the scripted grasp, so there is nothing yet to learn. Raise it toward
   the 15 mm the residual can cancel, re-measure the baseline through it, then train and
   evaluate on the same seed. It stays behind a parameter that is off until a paired
   evaluation shows it winning.
2. **Natural language commands.** A node turning "pick the red cube and put it on the left"
   into the strings the task server already accepts. A regular expression pass first, then
   a local Ollama model for anything the pattern misses. Both publish to `/task/command`,
   so neither touches the motion code.
3. **One end to end run on a GPU machine**, closing the camera to grasp loop and
   re-measuring perception accuracy in the current scene.
4. **A bringup package**, one top level launch composing simulation, perception, planning
   and tasks.
5. **Increasing scene complexity**, one variable at a time: objects closer together, then
   varied sizes, then orientations that require solving for the grasp angle.
6. **Real hardware.** A `ros2_control` hardware interface for the arm's serial stepper
   controller and the gripper, an RGB-D camera driver, and hand eye calibration. The
   planning and task layers above run unchanged.

## License

Apache License 2.0. See [LICENSE](LICENSE).
