# explorer_aggr

Autonomous frontier-based exploration package for ROS 2 and Nav2 mobile platforms.

## Overview

The package implements an autonomous exploration pipeline for mobile robots using 2D occupancy grids and planar LiDAR scans. It extracts frontiers with vectorized NumPy operations, evaluates candidates while preventing topological collapse, and executes robust recovery behaviors during navigation deadlocks.

## Architecture and Control Modules

Frontier extraction isolates free cells bordering unknown map space. To resolve the ring topology issue where circular scans collapse the geometric mean onto the robot footprint, the algorithm selects the physical median element along the cluster, ensuring the target falls on traversable space.

Candidate frontiers are scored by balancing cluster size against travel distance, applying exponential decay to coordinates that previously aborted or failed:

$$\text{score} = \frac{\text{size}}{\text{safedistance} \cdot 2^{\text{failures}}}$$

To prevent false positives where Nav2 reports success due to goal tolerances without genuine movement, an anti-cheat filter requires at least 10 centimeters of displacement before accepting a completed trajectory.

Continuous execution safety relies on a dual-tier supervisor. An active stall inspector cancels the goal if the chassis fails to advance 5 centimeters within 20 seconds, while an overarching 70-second watchdog aborts lingering actions. A sliding history buffer tracks recent coordinates, triggering recovery if the planner ping-pongs between identical targets.

When navigation stalls or loops persist, authority transfers to a three-phase LiDAR recovery state machine. Phase 0 clusters consecutive LiDAR rays exceeding 40 centimeters of clearance and selects the median angle of the widest sector to steer cleanly through the center of doorways. Phase 1 rotates the robot toward this escape heading. Phase 2 pushes the platform forward by 40 centimeters, protected by an active emergency brake that halts motion if an obstacle appears within 22 centimeters. Once all frontiers are resolved, the node automatically dispatches Nav2 back to the starting home coordinates.

## Installation and Launch

```bash
cd ~/ros2_ws/src
git clone [https://github.com/your-username/explorer_aggr.git](https://github.com/your-username/explorer_aggr.git)
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --packages-select explorer_aggr
source install/setup.bash
ros2 launch explorer_aggr exploration.launch.py use_sim_time:=true
