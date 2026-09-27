# Third-party components

The three C++ files in `robot-setup/calibration-tool/` originate from Universal Robots ROS 2 Driver tag 6.0.0 (`ur_calibration`). Their original copyright and BSD redistribution notices are retained in each file. Exact source URLs and upstream SHA-256 values are recorded in `sources.json`. The local calibration correction calls `pipeline.init()` before `pipeline.run()` and uses the included standalone CMake configuration.

URDFs and USD meshes are generated locally from installed `ur_description`; the repository does not include generated vendor meshes or robot-specific calibration. ROS/UR packages, Isaac ROS/cuMotion, NVIDIA Docker images, Isaac Sim and External Control URCap are obtained separately from their vendors and upstream package repositories.
