# UR5e static obstacle course

This is an additional mode of the existing demo. The original entrypoints remain
available. It uses MoveIt 2's `isaac_ros_cumotion` pipeline and the actual GPU
backend. Only the virtual UR controller is a GenericSystem mock.

One file, [`config/obstacle_course.yaml`](config/obstacle_course.yaml), defines the
boxes, their poses in `base_link`, the virtual initial joint configuration and
the target TCP pose (`tool0`, zero tool offset). Dimensions are metres, joint
positions are radians and quaternions are `[x,y,z,w]`. No intermediate waypoints
are specified. MoveIt IK selects collision-free goal configurations for the TCP
pose; the course tries nearby branches in cuMotion until one yields a valid path.
These are endpoint configurations, not operator waypoints.

## Run

Run from the demo's root directory after the existing platform and offline asset
setup. Close a previous course launcher before starting another one.

```bash
bash obstacle_course/scripts/launch.sh single_wall
bash obstacle_course/scripts/launch.sh corridor
bash obstacle_course/scripts/launch.sh gate_3d
```

Virtual mode owns a separate ROS domain **178**, mock controller and GPU
container. Closing Isaac Sim cleans up those processes. The original virtual
domain 172 and physical domain 174 retain their existing meanings. The launcher
refuses to take over an existing planner/controller in its domain.

In Isaac Sim, select `/World/TargetTCP`, move/rotate it and click **Plan**.
The current REAL state is the start. A successful plan runs the amber GHOST
preview. The amber polyline shows its TCP path; the grey straight line is only
a geometric reference and is never sent to a controller. **Execute MOCK** is
enabled after preview. To change the virtual start, edit the six
`initial_joint_positions` in YAML and relaunch. The physical start is always the
current robot pose, positioned by the operator using the existing UR controls.

All UI text is English. Status shows measured planning time, prepared duration,
point count and object count. Full start/target poses, pipeline/plugin, backend
goal UUID, validation and tracking metrics are in the run's JSON/CSV files.

## Physical mode

Use the existing calibrated UR driver and External Control program. Close the
old physical demo window/planner before starting the course in domain 174.

```bash
bash obstacle_course/scripts/launch.sh single_wall physical
```

This launcher does not start a robot program, reposition the robot or execute a
plan automatically. It checks calibrated FK and controller state, then opens
the existing Plan → GHOST preview → operator approval → FollowJointTrajectory
workflow with the course world added.

**Virtual boxes are not physical obstacles.** Position and measure the actual
obstacles to match YAML before approving a physical trajectory. The unchecked
confirmation says **Physical obstacles match this scene; work area is clear**.
Approval binds the exact previewed trajectory hash and geometry hash. The
existing executor also checks calibration, zero TCP, current start state,
unchanged collision scene, robot modes, speed scaling and inactive motion
controllers. Stop uses the existing cancellation/deactivation path. Physical
execution qualification requires an operator-approved run and is reported
separately from virtual qualification.

Collision avoidance is not a safety system and does not replace UR safety.

## Reproduce qualification

```bash
python3 obstacle_course/tests/test_world.py
bash obstacle_course/scripts/launch.sh single_wall qualify
bash obstacle_course/scripts/regression.sh
UR_COURSE_WORKFLOW_TEST=1 bash obstacle_course/scripts/launch.sh single_wall
```

Qualification runs all three scenes on domain 178. For each scene it plans
without obstacles, adds the shared boxes and demonstrates collision of that
baseline using MoveIt's complete URDF collision meshes. It then requests a new
GPU plan, validates the final controller spline, renders the matching USD
geometry, completes GHOST preview and sends the identical file to the mock FJT
controller. The gate additionally requires a baseline collision whose point
TCP is clear, demonstrating why checking TCP alone is insufficient. A blocked
target and a self-colliding joint goal must each produce **PLANNING FAILED**
with no usable trajectory file.

`runs/qualification.json` records results, contacts, runtime planner evidence,
geometry error and execution metrics. Each scenario has planning logs,
trajectory JSON, preview attestation, `ur5e-preview.png`, `ur5e-twins.usda` and
`planned-vs-actual.csv`. Generated run data is local and excluded from Git.
The measured host results are summarized in [VALIDATION.md](VALIDATION.md).

The GPU XRDF is generated from NVIDIA's bare-UR5e sphere model. It retains
self-collision checks/exclusions and adds a conservative sphere derived from
the installed fixed-base collision STL, covering all URDF collision links.
MoveIt independently validates the prepared quintic controller spline with
full robot meshes and the SRDF self-collision matrix. Samples are spaced by
the existing velocity-based rule (at most 0.02 rad movement per interval, with
at least two substeps per segment); this is sampled validation, not a continuous
collision certificate. Endpoint stop/retiming is performed before validation.

Runtime verification requires the active `isaac_ros_cumotion` pipeline,
`isaac_ros_cumotion_moveit/CumotionPlanner`, one `cumotion_planner` publisher,
one new backend action goal, `read_esdf_world=false` and exact Planning Scene
object readback. Scene changes invalidate execution; target/visible box/YAML
changes invalidate preview. Failed planning clears the executable plan and
path display. There are no cameras, point clouds, nvblox or perception ESDF.

NVIDIA's [cuMotion MoveIt documentation](https://nvidia-isaac-ros.github.io/v/release-5.0/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion_moveit/index.html)
describes this static Planning Scene interface and the GPU backend action.
