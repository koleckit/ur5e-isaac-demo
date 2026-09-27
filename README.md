# UR5e Isaac Demo

Local technology demo: **TargetTCP in Isaac Sim → MoveIt 2 / cuMotion plan → GHOST preview → FollowJointTrajectory execution → REAL twin and planned-vs-actual**.

The Isaac Sim panel is English ASCII and includes Plan, Preview, Reset TargetTCP, Execute on UR5e and Stop. An independent offline mode executes only on ros2_control GenericSystem. Physical execution checks the exact preview hash, calibration, zero TCP, fresh state/start position, robot modes, collision scene and controller configuration before arming. It deactivates its motion controller after completion or cancellation.

## Static obstacle course

An optional, independent [collision-aware obstacle course](obstacle_course/README.md) adds a wall, corridor and 3D gate. One YAML drives both Isaac Sim BOX visuals and MoveIt CollisionObjects. The demo uses the real cuMotion GPU backend, verifies the complete robot trajectory for collisions, then keeps the same GHOST preview and approval workflow. Its virtual mode runs on a separate GenericSystem domain 178; physical Execute remains an explicit operator action after matching actual obstacles to the model.

## Development environment

Recorded on Ubuntu 24.04 x86_64, RTX 5090 with NVIDIA 595.91.07, ROS 2 Lyrical, UR ROS 2 Driver 6.0.0, Isaac ROS/cuMotion 5.0 and standalone Isaac Sim 6.0.1-rc.7. This is the tested local stack; the repository does not assert validation on other combinations. Sim uses its bundled Jazzy bridge and communicates over DDS with host/container Lyrical. Docker image versions are pinned in `Dockerfile.isaac-ros`.

## Install and run offline

Install Isaac Sim separately from NVIDIA and configure its path. Installation scripts use sudo and write their own logs; inspect them before running on another host.

```bash
git clone https://github.com/koleckit/ur5e-isaac-demo.git
cd ur5e-isaac-demo
export ISAAC_SIM_DIR="$HOME/Downloads/isaac-sim-standalone-6.0.1-linux-x86_64"
bash install-ros.sh
bash install-platform.sh
bash build-isaac-ros.sh
bash prepare-offline-assets.sh
bash launch-offline-demo.sh
```

Docker must be usable through the docker group and the NVIDIA runtime. The offline launcher starts its own mock driver and GPU planner, displays the twins, and stops its owned processes on exit. Select `/World/TargetTCP`, move/rotate the target, click Plan, watch GHOST, then Execute MOCK. The seed included in `tests/fixtures/` is synthetic and only initializes this offline workflow.

## Physical UR5e

The demo uses PC Ethernet **192.168.60.200/24**, robot **192.168.60.201/24**, and External Control port **50002**. These addresses are explicit in the launcher, executor, calibration reader and panel. Keep those values consistent if adapting the demo to another network. ROS domains are isolated: offline 172, standalone planner checks 173, physical 174, executor mock qualification 176.

Install External Control URCap on PolyScope 5 and save a program with one External Control node. URCap host IP is the PC address. Detailed procedure: [robot-setup/README.md](robot-setup/README.md).

1. Extract this robot's factory calibration with `bash robot-setup/extract-calibration.sh`. This creates local, ignored `ur5e-calibration.yaml` and `ur5e-calibrated.urdf`.
2. Import the calibrated asset:

   ```bash
   bash run-isaac-python.sh import-ur5e.py --urdf robot-setup/ur5e-calibrated.urdf --asset-dir assets-physical --prefix ur5e-physical
   python3 check-dod.py ur5e-physical-import-dod.json
   ```

3. In another terminal, start `bash launch-ur-physical-state.sh`, then press Play on the pendant. Use zero active TCP and the bare-robot setup represented by this demo. The driver initially leaves motion controllers inactive.
4. Start `bash launch-ur-demo.sh` and follow [FINAL-DEMO.md](FINAL-DEMO.md).

Physical Execute is available only after the prepared preview completes and the operator confirms Work area is clear. **Execute on UR5e starts real motion**. Surrounding workstation geometry is not modeled. Stop cancels the ROS goal and deactivates the motion controller; it is separate from the pendant emergency stop. Closing the application also requests cancellation. GHOST follows scaled controller time during execution. New motions require new Plan/Preview.

Execution produces local `physical-planned-vs-actual.csv`, `physical-execute-dod.json`, and immutable `execution-history/<execution_id>/` snapshots. The public repository excludes factory calibration, robot telemetry, logs, recorded trajectories, generated USD assets and local approval files. Generate the model and collect qualification results for your own robot.

## Checks

```bash
# Pure trajectory bounds / rejection checks; no ROS, GPU or robot required:
python3 -m pip install numpy PyYAML
python3 test-execution-contract.py

# Full offline GUI workflow (after setup):
UR_DEMO_WORKFLOW_TEST=1 bash launch-offline-demo.sh

# Read-only physical Plan/Preview/preflight qualification; never auto-Execute:
UR_DEMO_PHYSICAL_PREVIEW_TEST=1 UR_DEMO_TEST_GUI=1 bash launch-ur-demo.sh
```

The isolated production-executor mock test is described in [docs/validation.md](docs/validation.md). Local source/definition-of-done scripts are included; hardware, rendering, collision environment and physical tracking are not evaluated by GitHub Actions. The CI job checks syntax and the pure mathematical contract.

Calibration sources retain their upstream notices and provenance: [THIRD_PARTY.md](THIRD_PARTY.md). NVIDIA binaries, ROS packages, URCap binaries and generated vendor assets are installed separately.
