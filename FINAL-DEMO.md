# UR5e: Plan / Preview / Execute

Start with the physical UR driver running and the External Control program playing on the pendant:

```bash
cd 'path/to/ur5e-isaac-demo'
bash launch-ur-demo.sh
```

If the PC/driver has been restarted, first run `bash launch-ur-physical-state.sh` in another terminal, then press Play on the pendant. Robot IP: 192.168.60.201. External Control host: 192.168.60.200, port 50002. The launcher does not start a pendant program, release brakes, reset faults or change the speed slider.

1. Click **Reset TargetTCP to current REAL TCP**.
2. Select `/World/TargetTCP` in Stage and move/rotate it. Position units are metres. For the first physical test, use a small offset, e.g. 0.01 m, within the cleared workspace.
3. Click **Plan**. MoveIt seeded IK and cuMotion plan from the current physical joints. The final spline is prepared before display: at most 0.15 rad/s and 0.30 rad/s², at least 2 seconds, zero initial/final velocity and acceleration, calibrated joint limits and sampled MoveIt collision validation.
4. Watch the orange **GHOST** complete its preview. **Preview** replays the same prepared file. **REAL** continues to follow the physical robot.
5. Confirm **Work area is clear**, then click **Execute on UR5e**. This button starts physical motion after preflight succeeds. The surroundings are not modeled; the checkbox records the operator's check of the actual work area.
6. **Stop** cancels the current ROS goal and deactivates the motion controller. It is a software cancellation; the pendant's emergency stop remains separate. Closing the application also requests cancellation and waits for cleanup.

Execute remains disabled until a prepared plan completes preview, the target is unchanged, External Control is running, the start position still matches and the work area is confirmed. Moving the robot after planning requires a new Plan/Preview. Execution copies the exact previewed positions, derivatives and timestamps; it does not move the robot to the plan start or replan automatically. A changed TCP, calibration, collision scene, stale state, unsafe mode or competing motion controller rejects execution. Speed scaling is supplied by the UR hardware interface; GHOST follows the scaled controller trajectory time.

After execution: `physical-execute-dod.json`, `physical-planned-vs-actual.csv`, and `execution-history/<execution_id>/` contain the result, tracking errors, immutable plan, preview and approval. CSV compares the prepared spline with actual joints at the same scaled controller time. After any execution or Stop, use Plan/Preview again.

Qualification: real Plan/Preview/Reset and read-only physical execution preflight PASS; exact FJT execution and Stop PASS on an isolated GenericSystem in domain 176. No physical trajectory was sent during qualification. Physical tracking and speed-slider behavior remain pending the operator's first physical execution. See [docs/validation.md](docs/validation.md) for the recorded qualification scope. Local reports are not published.
