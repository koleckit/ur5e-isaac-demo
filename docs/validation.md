# Validation scope

Recorded software qualification on the development machine:

| Check | Result / scope |
|---|---|
| ROS pub/sub, UR mock /joint_states and FJT | PASS, local ROS |
| Docker / NVIDIA GPU and cuMotion | PASS, local RTX 5090 |
| Isaac Sim renderer, import and DDS bridge | PASS, standalone build |
| Offline Plan → Preview → Execute MOCK → REAL | PASS, GenericSystem |
| Calibrated physical state, Reset TargetTCP, 10 mm Plan/Preview | PASS, real state and planning only |
| Physical executor --check-only | PASS, no activation or motion goal |
| Exact prepared FJT execution and Stop | PASS, isolated GenericSystem domain 176 |
| Physical motion and speed-slider tracking | Not part of the unattended software qualification; validate locally with the operator |

Trajectory validation checks analytic extrema of the quintic interpolation, including between knots; joint limits, retiming, finite strictly increasing times, zero endpoint derivatives, model/calibration identity and preview SHA are checked. After physical preparation, MoveIt state validity samples the final spline. No physical workstation geometry or ESDF sensor is configured in this example.

The executor requires an inactive motion controller and never takes over an already active controller. It activates JTC only for an approved execution, sends unchanged prepared points and cancels only its own goal UUID. Cleanup is checked after success and Stop. The feedback comparison uses desired.time_from_start and the exact preview spline; speed scaling is read from the UR hardware state. The UR speed broadcaster's percent value is normalized before use.

To reproduce the guarded executor test, first generate the local generic and calibrated models required by the project (root README). Source `ros-env.sh` in two terminals, then run:

```bash
# Terminal A: physically isolated GenericSystem
ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=127.0.0.1 \
  use_mock_hardware:=true launch_rviz:=false launch_dashboard_client:=false \
  activate_joint_controller:=false initial_joint_controller:=joint_trajectory_controller

# Terminal B: exact execution, Stop, changed-preview rejection, cleanup checks
ROS_DOMAIN_ID=176 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
python3 test-physical-executor.py
```

The test flag requires domain 176, its own report directory and a verified GenericSystem plugin. The physical executor requires domain 174, the physical UR plugin, calibration/IP checks and a current operator approval. Tests do not automatically submit a physical goal.

Fresh clones include only synthetic mathematical fixtures. Runtime reports and per-robot evidence remain local. Inspect generated JSON `result`, not only the process exit status, because Isaac Sim's close behavior can obscure earlier script errors.
