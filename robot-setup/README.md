# UR5e / PolyScope 5 setup

1. Install [External Control URCap 1.0.6](https://github.com/UniversalRobots/Universal_Robots_ExternalControl_URCap/releases/tag/v1.0.6) using Settings → System → URCaps and restart as prompted. The developed installation used PolyScope 5.26; PolyScope X uses a separate integration.
2. Enable the Primary Client Interface and RTDE services needed by the driver and calibration reader. Dashboard is not used to start the program in this workflow.
3. Configure distinct Ethernet addresses: PC 192.168.60.200/24, robot 192.168.60.201/24. The sample driver binds the reverse connection to the PC address.
4. Installation → URCaps → External Control: host IP 192.168.60.200, port 50002.
5. Save a program containing an External Control node and its associated installation. Start the PC driver before pressing Play on the pendant.
6. Set the TCP to zero for the bare robot represented by this model. Extract factory calibration and import the calibrated asset as described in the root README.

The PC runs ROS, MoveIt, cuMotion and Isaac Sim; these packages are not installed on the robot. No dashboard auto-start, brake release, fault reset or speed-slider changes are performed by the demo.

`extract-calibration.sh` builds the pinned calibration sources, including explicit URCL pipeline initialization, and writes local files excluded by `.gitignore`. `test-calibrated-fk.py` compares q_actual and actual TCP from the same read-only Primary packet against the calibrated model; it sends no motion command. It also expects the generic model created by `prepare-offline-assets.sh` for its reference comparison.

[UR setup documentation](https://docs.universal-robots.com/Universal_Robots_ROS2_Documentation/doc/ur_client_library/doc/setup/robot_setup.html).
