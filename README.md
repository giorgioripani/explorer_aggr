# explorer_aggr

Autonomous frontier-based exploration package for ROS 2 and Nav2 mobile robotics platforms.

## Overview

The explorer_aggr package provides a fully integrated autonomous exploration pipeline designed to navigate unknown indoor environments using 2D occupancy grids and planar LiDAR scans. Instead of relying on naive geometric centroids, the node identifies exploration boundaries using vectorized spatial filtering, scores candidates through a multi-factor heuristic, and manages edge cases through active stall monitors, trajectory loop breakers, and an autonomous reactive LiDAR recovery routine.

## Architecture and Control Modules

The system is organized into two primary components working in tight coordination: the mathematical frontier extractor (`GoalSetter`) and the ROS 2 lifecycle coordinator (`CommunicationNode`).

### Frontier Detection and Ring Topology Mitigation

Frontier extraction operates through vectorized spatial shifts using NumPy arrays, identifying free map cells directly adjacent to unknown space. In typical exploration scenarios where the robot performs an initial 360-degree scan, circular or concave frontiers produce a geometric mean that collapses into the empty center, placing the candidate target directly beneath the robot base. The algorithm resolves this failure mode by extracting the physical median index along the ordered boundary coordinates rather than computing the mathematical centroid, guaranteeing that the target pose always lies on a valid, traversable boundary.

### Candidate Utility Scoring and Failure Penalties

Each candidate cluster is evaluated through an information-to-distance utility function where the score is proportional to cluster size and inversely proportional to the Euclidean distance from the robot pose. To prevent persistent attempts at unreachable poses behind thin walls or glass obstacles, every goal rejected or aborted by Nav2 increments a dedicated failure register tied to that coordinate. Each registered failure halves the subsequent utility score using exponential decay:

$$\text{score} = \frac{\text{size}}{\text{safe\_distance} \cdot 2^{\text{failures}}}$$

### Anti-Cheat False Arrival Verification

Standard Nav2 planners mark a goal as completed once the base link falls within the configured positional arrival tolerance. When frontiers are selected close to the robot, this mechanism can trigger immediate success without generating physical movement, causing premature exploration termination. The communication node tracks initial departure coordinates and computes the physical distance traversed upon receipt of the success status. If the total displacement remains below 10 centimeters, the node flags the event as a false-positive arrival, registers a failure penalty against that target, and commands an immediate replan.

### Dynamic Stall Inspector and Navigation Watchdog

To prevent indefinite deadlocks in complex geometry, the node executes a two-tier monitoring routine. An active stall inspection timer evaluates physical displacement every two seconds; if the robot fails to progress at least 5 centimeters over 20 consecutive seconds, the active action goal is formally canceled. In addition, an overarching 70-second execution watchdog disarms and aborts any trajectory that exceeds the maximum operational window, unlocking the state machine to pursue alternative frontiers.

### Anti Ping-Pong History Buffer

SLAM mapping latency can occasionally leave previously visited sectors marked as unexplored for several update cycles, prompting the planner to oscillate back and forth between two identical points. The node records the four most recent goal positions in a sliding history buffer. If a newly computed frontier falls within 20 centimeters of a recently visited target for three consecutive cycles, the node breaks the infinite loop, clears the buffer, bypasses Nav2, and hands over authority directly to the recovery subsystem.

### Custom Reactive LiDAR Recovery Subsystem

When the robot encounters severe entrapment, repeated recalculation failures, or map lag oscillations, the communication node bypasses global path planning and engages a dedicated three-phase recovery state machine. 

Phase 0 performs real-time sensor fusion by projecting planar laser scans into the global costmap space. Instead of steering toward the single longest ray, which frequently clips wall corners and door jambs, the algorithm groups consecutive rays with clearance above 40 centimeters into distinct angular clusters and selects the median heading of the widest open sector. This centering technique directs the vehicle safely through the geometric center of doors and corridors.

Phase 1 rotates the robot in place until the angular heading error relative to the chosen escape vector is reduced below 0.1 radians.

Phase 2 executes an open-loop forward push of 40 centimeters at 0.15 meters per second. Throughout the push maneuver, an emergency collision brake inspects a 30-degree frontal cone; if an obstacle is detected within 22 centimeters of the chassis, the push is aborted instantly, bringing the robot to a complete halt before returning control to the main exploration loop.

### Return to Home (RTH)

Upon startup, the node records the initial valid robot pose in the map frame. When the mathematical module verifies that all valid frontier clusters have been resolved and the environment is completely mapped, the exploration phase transitions to completion and automatically commands Nav2 to return to the original home coordinates.

## Installation and Execution

Clone the package inside the source directory of your active ROS 2 workspace, resolve required dependencies via rosdep, and compile using colcon:

```bash
cd ~/ros2_ws/src
git clone [https://github.com/your-username/explorer_aggr.git](https://github.com/your-username/explorer_aggr.git)
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install --packages-select explorer_aggr
source install/setup.bash
