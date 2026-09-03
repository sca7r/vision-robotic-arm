# vision-arm-ros2

A vision guided pick and place system for a 6 DOF robotic arm, built on ROS 2 Jazzy and
Gazebo Harmonic. You give it a short command such as `red left`, and the arm looks at the
bench, finds the red cube, picks it up, puts it down where you asked, and returns to its
starting pose.

![Arm and gripper model in RViz](docs/images/visionarm.png)

The arm is a 3D printed stepper driven design with a parallel two jaw gripper and a wrist
mounted RGB-D camera. It had no ROS support of any kind, so this repository builds the
whole stack from the URDF upwards: model, simulation, control, motion planning,
perception, and task sequencing. Every layer is written so that the same code drives the
real hardware once a `ros2_control` hardware interface is added underneath it.

A reinforcement learning layer sits on top of that stack. A small policy learns a bounded
correction to the grasp pose the motion planner is asked to reach, so that the arm
compensates for the camera to base calibration error that real hardware always has.

## Status

| Capability | State |
|---|---|
| URDF model, joint limits, inertials | Working, 9 tests |
| Gazebo simulation with `ros2_control` | Working, runs at roughly realtime |
| Arm bolted rigidly to a workbench | Working, zero drift measured |
| MoveIt 2 planning and collision checking | Working |
| RGB-D cube detection in `base_link` | Working, accuracy measured at 1.5 to 2.4 mm |
| Pick and place cycle | Working, placement accuracy 1 to 5 mm |
| Camera to grasp in one loop | Not yet run end to end, see Known Limitations |
| Learned grasp residual, reinforcement learning | Not yet trained |
| Natural language commands | Not started |
| Real hardware | Not started |

Test counts by package: description 9, gazebo 3, moveit_config 4, perception 4,
tasks 9. All 29 pass.

## Quick Start

### Prerequisites

Ubuntu 24.04 with ROS 2 Jazzy:

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-moveit \
  ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-vision-msgs \
  python3-colcon-common-extensions
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
above the cube, descends straight down, closes the jaws, lifts, carries, lowers, releases,
and returns home.

## Telling the Robot What to Do

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

Only the text inside the inner quotes changes from one command to the next. Everything
else stays identical.

### The two command forms

```
'<colour> <place>'      e.g.  'red left'          a named destination
'<colour> <x> <y>'      e.g.  'red 0.15 -0.40'    explicit coordinates
```

Colours are `red`, `green` and `blue`. Named places are defined in
`src/vision_arm_tasks/config/poses.yaml` and are currently `left`, `right` and `front`.

### Coordinates

`x` and `y` are metres in `base_link`, the frame sitting at the arm's base.

- **x** runs left and right across the bench. Negative is left, positive is right, and 0 is
  straight ahead.
- **y** is distance out from the robot. It is always **negative**, because the workspace is
  in front of the base. Something at `-0.30` is close in, something at `-0.55` is far out.
- There is no **z**. Objects are placed on the bench, at the height they were picked from.

Usable range, taken from the measured reach map rather than guessed: x from about -0.31 to
+0.48, y from about -0.19 to -0.63. Stay a little inside those edges, and do not ask for
anything closer than roughly y = -0.32, because that is where the arm's own base plate is.

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

The dotted rectangle is where a straight down grasp is geometrically possible. Outside it,
the arm either cannot reach or cannot get the gripper vertical, and the command comes back
as a failure without the arm moving.

### Adding your own named places

Easier than retyping coordinates. Edit `src/vision_arm_tasks/config/poses.yaml`:

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

To point the node at an entirely different set of poses without touching the installed
file:

```bash
ros2 run vision_arm_tasks task_server.py --ros-args -p poses_file:=/path/to/other.yaml
```

### Reading the reply

```bash
ros2 topic echo /task/result
```

There are three kinds of reply:

```
picking red at (-0.120, -0.425, 0.041) -> (-0.200, -0.420)
done: red is at (-0.198, -0.422), 3 mm from target
failed: unknown place 'lft', known: ['front', 'left', 'right']
```

The `done` line reports where the cube **actually** ended up, re-detected from the home
pose after the move, not where the arm was asked to put it. If those disagree by more than
`place_tolerance` you get a `failed` line instead, which is how a grasp that closed on
nothing gets caught.

## How It Works

### The cycle

The whole system is organised around one repeating loop, anchored on a single **home
pose**:

```
   home  ->  detect  ->  plan  ->  approach  ->  grasp
     ^                                             |
     |                                             v
   home  <-  release  <-  lower  <-  carry  <-  lift
```

The home pose is six joint angles chosen so that every object on the bench is inside the
wrist camera's frame, the arm is not in collision with itself, and every object is
reachable from there. Because the arm returns to it after every job, three useful
properties fall out:

- Every job starts from an identical, known state, so results are comparable run to run.
- The scene is re-detected at the start of every job, so objects moved by hand between
  jobs are picked up automatically. This is observable: during testing, carrying one cube
  nudged its neighbour 17 mm sideways, and the next command grasped that neighbour at its
  new position without being told.
- Changing the environment means editing six numbers in a YAML file, not editing code.

The home pose currently in use is `[-1.1215, 0.047, 0.7946, 1.026, 0.6052, -1.0986]`. It
was not chosen by hand. `tools/find_look_pose.py` searched joint space for a configuration
that passes three filters in increasing order of cost: forward kinematics plus a pinhole
camera projection to check the cubes are inside the frame, `/check_state_validity` to check
the arm is not folded into itself, and `/compute_ik` to check each cube can actually be
reached from there.

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
      |  /compute_ik      Cartesian waypoint -> joint angles
      |  /move_action     plan and execute, collision aware
      |  /gripper_controller/commands   jaw positions
      v
  ros2_control -> joint_trajectory_controller -> Gazebo (or real hardware)
```

The detector holds a **world model**, a map from colour to the last known position and the
timestamp it was seen at. Objects therefore stay in `/detections` after they leave the
camera frame, which is what makes a single look pose sufficient and what allows the arm to
sweep and accumulate views if the scene ever grows past one frame.

### Motion

MoveIt is driven from plain `rclpy`. There is no `moveit_py` and no MoveIt Task
Constructor, because neither is installed on the target platform and MTC on Jazzy requires
a patched fork. A pick is five moves, so the services are called directly:

- `/compute_ik` turns a Cartesian target into joint angles.
- `/check_state_validity` reports named contact pairs, which is invaluable for debugging.
- `/move_action` plans a collision free path to a joint configuration and executes it.
- The gripper is not in MoveIt. Jaw positions are published as `[x, -x]` on
  `/gripper_controller/commands`.

Two details in `solve_ik` exist because of failures found by running the system, not by
reading it:

1. **Every waypoint is solved with the jaw position it will be executed with.** Closing the
   jaws on a cube swings the jaw meshes about 3 cm, and they then contact the forearm's
   collision volume. Validating with open jaws and executing with closed ones caused the
   place descent to fail while the arm hung in the air holding a cube.
2. **IK is retried across seeds and jaw yaws.** KDL is a local numeric solver, so it
   succeeds or fails depending on where it starts, and `/compute_ik` seeds from wherever
   the arm happens to be standing. The same grasp point solved after one approach and
   failed after another. `solve_ik` now tries the current state, the home pose, six
   perturbations of home, and four yaw angles. A cube is square, so which way the jaws
   close is free.

All waypoints are solved **before** the arm leaves home. A destination that turns out to be
unreachable therefore costs nothing, instead of stranding the arm mid cycle holding an
object.

Finally, the result is **verified rather than assumed**. A gripper that closes on nothing,
or an object that slips during the lift, produces exactly the same sequence of successful
moves as a real pick. After returning home the node re-detects the object and compares its
position against the requested destination. This is not hypothetical: the first full run
reported three successes when one cube had never left its starting position.

## System Architecture

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

### Design principles

- **Separation of concerns.** Perception knows nothing about motion planning. The task
  layer never writes joint commands for the arm directly, it goes through MoveIt.
- **Interface driven boundaries.** The perception contract is
  `vision_msgs/Detection3DArray` in `base_link`. Any detector honouring that contract, HSV
  today or a learned model later, is a drop in replacement. `tools/fake_detections.py`
  exploits exactly this to test motion without a camera.
- **Simulation and hardware parity.** Controllers are defined through `ros2_control`, so
  the Gazebo plugin and a future hardware interface expose identical command and state
  interfaces.
- **Configuration over code.** Home pose, named destinations, gripper positions,
  tolerances, HSV bands and controller gains all live in YAML.
- **Learning is additive, not a replacement.** The learned grasp residual is bounded and
  clipped, and MoveIt still plans and collision checks the corrected pose. If the policy is
  switched off, or its correction fails inverse kinematics, the task layer falls back to the
  scripted grasp pose. The worst case of an untrained policy is the behaviour the system
  already has.
- **Verify, do not assume.** Every claim in this README that has a number attached to it
  was measured against Gazebo ground truth, and the tools that measured it are in `tools/`.

## Packages

### vision_arm_description

The single source of truth for the robot model. A modular xacro builds the arm, mounts the
gripper on the flange, attaches the camera, and welds the whole thing to a workbench.

- Six revolute joints with position limits derived from the stepper firmware's step count
  bounds multiplied by each joint's gear ratio, then recentred on the pose the homing
  routine actually leaves the arm in. The model can only reach what the hardware can reach.
- Full inertial parameters per link. One of them, the base plate, had a CAD exported centre
  of mass lying outside its own mesh and outside its support polygon, which made the model
  tip over as soon as any mass was added. A test now asserts that every link's centre of
  mass lies inside that link's mesh.
- Collision geometry is the mesh bounding box rather than the mesh itself. The full meshes
  are 278k triangles and grinding them against the ground plane every millisecond held the
  simulation at a real time factor of 0.006. The jaws keep their real meshes, because
  grasping needs real geometry.
- A `world` link and a fixed `world_joint` bolt `base_link` to the bench at `mount_z`.
- The workbench is a **link in the URDF**, not a model in the world SDF, so Gazebo and
  MoveIt share one definition of the surface the arm stands on. A planner that cannot see
  the bench will drive the arm straight through it.
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
- The `arm` group is declared as a **chain** from `base_link` to `tcp`, not as a joint list.
  The joint list form ended the group at the flange, so IK solved for a point 140 mm short
  of where the gripper actually grasps, and every Cartesian goal was wrong by that much.
- KDL inverse kinematics and OMPL planning.
- A collision matrix that disables structurally adjacent pairs and three pairs that were
  measured to collide in 100 percent of configurations. See Engineering Notes.

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
  Commands take either a named destination (`red left`) or explicit coordinates in
  `base_link` (`blue 0.05 -0.55`).
- There is deliberately no custom action or interfaces package. Nothing needs cancellation
  or progress feedback yet, and a string is exactly what the planned language layer will
  publish. Swapping in an action later touches one node.
- All configuration lives in `config/poses.yaml`, selected with `-p poses_file:=...`.

### vision_arm_rl

The learning layer, and the only package here that trains a neural network. It does not
replace MoveIt, it corrects the pose MoveIt is asked to reach.

The scripted grasp computes a nominal grasp pose from the detected object. On real hardware
that pose is systematically wrong, because camera to base calibration is never exact, and a
fixed offset of a few millimetres is enough to turn a grasp into a nudge. This package learns
that correction. A small multilayer perceptron reads the detected object pose and the nominal
grasp pose, and outputs a bounded offset in x, y, z and yaw which is added to the grasp pose
before it is sent to inverse kinematics.

One grasp attempt is one action followed immediately by its reward, so this is a contextual
bandit rather than a policy with a planning horizon, and it is trained as one. That is a
deliberate choice of the smallest formulation that fits the problem. An episode costs a few
seconds of simulated execution, which affords a few thousand attempts and not the hundreds of
thousands a step by step control policy would need.

| Piece | What it is |
|---|---|
| `residual_env.py` | Gymnasium environment wrapping the live stack, one grasp per step |
| `train.py` | Soft actor critic from Stable Baselines3, horizon 1 |
| `eval.py` | Paired success rate, learned residual against the scripted baseline |
| `models/` | Trained checkpoint |

The correction is clipped to 15 mm and 10 degrees, so the policy cannot ask for a pose far
from the one the scripted system would have used. Training runs headless against Gazebo on
CPU. No GPU is involved, because at one action per grasp the cost is simulator time rather
than gradient time.

Full design in
[`docs/superpowers/specs/2026-09-03-residual-rl-grasp-design.md`](docs/superpowers/specs/2026-09-03-residual-rl-grasp-design.md).

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

## Engineering Notes

These are the decisions that were expensive to arrive at and are cheap to undo by accident.

### The arm is bolted down, and this is load bearing

The model used to be free floating. Joint 1 only turned because the base counter rotated
underneath it, absorbing roughly half of every commanded angle: commanding minus 0.9 rad
yawed the base plus 0.46 rad. That is survivable for perception, which measures objects in
`base_link` and rides the base along with them, but it is fatal for pick and place, because
objects are bolted to the world rather than to the base. An object measured at a `base_link`
coordinate is no longer at that coordinate once the base has yawed 26 degrees mid motion.

The arm is now welded to the world with a fixed joint. Verified with
`tools/weld_check.py`: a six joint move tracks every joint to within 0.003 rad while
`gz model -m vision_arm -p` reports the model at exact world identity throughout.

### The layout was chosen by measurement, not by eye

`tools/reach_map.py` samples 400k joint configurations, keeps those within 15 degrees of
straight down, and reports where a top down grasp is geometrically possible for each
candidate mount height. The result decided the scene:

| Mount | Usable top down grasp area |
|---|---|
| Arm on the floor, objects on a 0.28 m table (the original layout) | **none at all** |
| Arm bolted flat to the work surface | about 2500 cm2, radius 0.21 to 0.65 m |
| On a 10 cm riser | about 1600 cm2 |
| On a 20 cm riser | about 1125 cm2 |
| On a 28 cm riser | about 675 cm2 |

So the arm is bolted flat and there is no pedestal, because every centimetre of riser
shrinks the reachable area. The long standing belief that "top down grasps are unreachable
with this wrist" was an artifact of the original layout.

### Three link pairs made every state invalid

Sampling 600 random configurations through `/check_state_validity` found that **zero** of
them were collision free. Three pairs collided in all 600:

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

## Command Reference

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

### Development tools

Not a ROS package and not built. Run with the workspace sourced.

| Script | Needs | What it does |
|---|---|---|
| `tools/reach_map.py <urdf> [n] [cube]` | nothing | Where a top down grasp is possible, per mount height |
| `tools/find_look_pose.py <urdf>` | sim and `move_group` | Searches for a look pose passing all three filters |
| `tools/weld_check.py [q1 or q1,..,q6]` | sim | Commands one trajectory, prints what each joint reached and where the base ended up |
| `tools/fake_detections.py` | sim | Publishes `/detections` from Gazebo ground truth, so motion can be tested without the camera |
| `tools/check_accuracy.py [seconds]` | sim and perception | Compares `/detections` against ground truth |
| `tools/sweep.py` | sim and perception | Sweeps joint 1 and shows the world model accumulating views |

Get the expanded URDF that several of these want with:

```bash
xacro src/vision_arm_description/urdf/vision_arm.urdf.xacro camera:=true > /tmp/arm.urdf
```

## Testing

```bash
colcon test && colcon test-result --verbose
# or, faster during development:
python3 -m pytest src -q
```

29 tests across five packages. They are deliberately weighted towards the things that fail
silently: that the mount height and the bench top agree, that every link's centre of mass
lies inside its own mesh, that the camera sensor pose matches the optical frame's claim,
and that a malformed command is rejected rather than turned into a motion.

## Repository Layout

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
  LICENSE
```

## Known Limitations

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
- **The grasp residual is not trained yet.** The baseline it has to beat has not been
  measured, so there is no evidence yet that the scripted grasp fails often enough in
  simulation for a learned correction to have anything to learn. Measuring that is the first
  phase of the work. The policy stays disabled by default until a paired evaluation shows it
  beating the scripted grasp.
- **No real hardware.** Everything here is simulation.

## Roadmap

1. **A learned grasp residual.** Measure the scripted grasp's baseline success rate over
   randomised object placements, inject the camera to base calibration error that real
   hardware will have, then train a bounded correction to the grasp pose against it. Kept
   behind a parameter that stays off until a paired evaluation shows it winning.
2. **Natural language commands.** A node turning "pick the red cube and put it on the left"
   into the strings the task server already accepts. A regular expression pass first, then
   a local Ollama model for anything the pattern misses. Both publish to `/task/command`,
   so neither touches the motion code.
3. **One end to end run on a GPU machine**, closing the camera to grasp loop and
   re-measuring perception accuracy in the current scene.
4. **A bringup package**, one top level launch composing simulation, perception, planning
   and tasks.
5. **Continuous integration**, building all packages and running `colcon test` on push.
6. **Increasing scene complexity**, one variable at a time: objects closer together, then
   varied sizes, then orientations that require solving for the grasp angle.
7. **Real hardware.** A `ros2_control` hardware interface for the arm's serial stepper
   controller and the gripper, an RGB-D camera driver, and hand eye calibration. The
   planning and task layers above run unchanged.

## License

Apache License 2.0. See [LICENSE](LICENSE).
