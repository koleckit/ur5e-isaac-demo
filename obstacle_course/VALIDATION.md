# Qualification on the RTX 5090 host (2026-09-28)

Run from the project root:

```bash
python3 obstacle_course/tests/test_world.py
bash obstacle_course/scripts/launch.sh single_wall qualify
UR_COURSE_WORKFLOW_TEST=1 bash obstacle_course/scripts/launch.sh single_wall
bash obstacle_course/scripts/regression.sh
```

All commands completed with exit code 0. The first test covers schema and
oriented-box geometry; the GPU qualification report is saved locally at
`obstacle_course/runs/qualification.json`. No physical trajectory was sent.

| Scenario | Boxes | Baseline collision samples | Of these, point TCP clear | Successful GPU planning time | Prepared duration | Points | Mock maximum tracking error |
|---|---:|---:|---:|---:|---:|---:|---:|
| `single_wall` | 1 | 66 | 52 | 137.7 ms | 22.88 s | 23 | 3.73e-06 rad |
| `corridor` | 3 | 7 | 7 | 149.2 ms | 23.48 s | 24 | 3.07e-06 rad |
| `gate_3d` | 3 | 168 | 168 | 142.4 ms | 23.03 s | 23 | 4.32e-06 rad |

The baseline is a plan for the same start and TCP target without course boxes.
MoveIt mesh collision checks reject its listed samples after the boxes are
applied. The prepared cuMotion trajectory had zero rejected samples in all
three scenes (127–137 samples per plan). A free point TCP in a colliding sample
shows that point-only checking would miss collisions of the arm or wrist.

The active MoveIt pipeline was `isaac_ros_cumotion`, with plugin
`isaac_ros_cumotion_moveit/CumotionPlanner`. Each successful request produced
one new goal on the `cumotion_planner` backend action, and `read_esdf_world`
was false. Failed IK branches were recorded separately. The backend log also
reported a robot-world collision when rejecting a candidate configuration.
The course XRDF covered every URDF collision link (including the fixed base)
with 29 spheres and retained self-collision checks. Isaac box readback matched
the same YAML to better than 3e-8 m in the sampled scene transforms.

A reachable TCP goal enclosed by a test BOX was rejected by the GPU backend.
A sampled self-colliding joint goal was also rejected; MoveIt reported
shoulder–forearm and upper-arm–wrist contacts. Both returned PLANNING FAILED
with no trajectory file. The new UI's Plan → GHOST → Execute MOCK button path
passed; REAL followed FJT on GenericSystem. The original offline workflow
also passed, as did the guarded physical executor's isolated GenericSystem
qualification (exact trajectory, Stop/controller cleanup, stale preview
rejection).

A separate **read-only** check of the connected UR5e passed calibrated FK,
zero TCP, running External Control and inactive motion controllers. The new
obstacle scenes have **not** been matched to measured physical obstacles, and
no physical motion in this course was tested. Physical Execute remains an
operator action after GHOST preview and explicit confirmation that the work
area and obstacles match the model. Sampled collision validation and cuMotion
planning are not a robot safety system.
