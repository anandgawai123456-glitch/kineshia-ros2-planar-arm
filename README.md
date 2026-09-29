# Kineshia Robotics — 3-DoF Planar Arm ROS 2 System

A compact ROS 2 implementation of a 3-DoF planar robotic arm with Cartesian target control, inverse kinematics, smooth joint-space trajectory generation, safety validation, live PyQt5/PyQtGraph telemetry, and a pick-and-place demonstration.

video: https://drive.google.com/file/d/1r7n4lYzV61VdGeP0ZoXH9nTk7CJUfqZ1/view?usp=drive_link

## System Overview

The system contains two ROS 2 nodes:

```text
Operator
   |
   v
PyQt5 / PyQtGraph GUI
   |
   | /target_pose
   v
Controller Node
   |
   +-- Reachability
   +-- Inverse Kinematics
   +-- Safety Validation
   +-- Quintic Trajectory
   |
   | /joint_states
   v
GUI Telemetry
Arm Model

The provided PlanarArm implementation is used as the kinematics and constraint library and was not modified.

Property	Value
Link lengths	3.0, 2.0, 1.5 m
J1 limit	0° to 180°
J2 limit	-120° to 120°
J3 limit	-120° to 120°
Ground constraint	y >= 0
Maximum radial reach	6.5 m
Controller

The controller:

Receives Cartesian targets through /target_pose.
Checks workspace reachability.
Projects unreachable targets to the closest reachable boundary.
Solves inverse kinematics.
Validates joint limits and the ground constraint.
Generates a smooth fifth-order joint trajectory.
Publishes sensor_msgs/JointState at 50 Hz.
Publishes controller status for GUI feedback.

The quintic trajectory uses:

s(t) = 10t^3 - 15t^4 + 6t^5
Operator GUI

The GUI provides:

Live 2D arm visualization
Workspace boundary and ground constraint
End-effector position
Joint telemetry
Joint-angle history
Cartesian target input
Editable PICK target
Editable PLACE target
Required (7.0, 3.0) edge-case command
Pick-and-place sequence control
Controller and planning status

The Qt event loop remains responsive while ROS 2 callbacks are serviced without blocking the GUI.

Pick-and-Place

The operator can configure PICK and PLACE Cartesian targets directly from the mission controls.

Default demonstration targets:

PICK  = (4.0, 2.0)
PLACE = (-3.0, 3.0)

Sequence:

PICK target
    |
    v
Move
    |
    v
PICK action
    |
    v
PLACE target
    |
    v
Move
    |
    v
Complete
Edge Case

The required test target is:

(7.0, 3.0)

This is outside the nominal 6.5 m radial workspace. The controller projects it to the closest reachable target and reports the projected condition to the operator.

ROS 2 Interfaces
Subscribed

/target_pose — geometry_msgs/PointStamped

Published

/joint_states — sensor_msgs/JointState

/controller_status — std_msgs/String

Package Structure
kineshia_ros2_arm_task/
├── README.md
├── DESIGN_NOTE.md
├── PART2_SIM_TO_REAL.md
└── src/
    └── planar_arm_control/
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        ├── resource/
        │   └── planar_arm_control
        ├── launch/
        │   └── bringup.launch.py
        └── planar_arm_control/
            ├── __init__.py
            ├── planar_arm.py
            ├── controller_node.py
            └── gui_node.py

planar_arm.py is the provided kinematics implementation and was kept unchanged.

Requirements
Ubuntu
ROS 2 Jazzy
Python 3
NumPy
PyQt5
PyQtGraph

Install GUI dependencies if required:

sudo apt install python3-pyqt5 python3-pyqtgraph
Build
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
Run
ros2 launch planar_arm_control bringup.launch.py

This launches both the controller and GUI nodes.

Design Decisions

A quintic joint-space trajectory was selected because it provides smooth endpoint velocity and acceleration while remaining lightweight.

Individual Cartesian target commands use a ROS 2 topic. A ROS 2 Action would be a natural next step for production pick-and-place because it would provide explicit goal feedback, cancellation, and completion semantics.

The current planning and safety responsibilities are logically separated while remaining lightweight within the controller implementation. A future production architecture can formalize these as separate planner, safety, and hardware-backend interfaces.

Sim-to-Real

The current system models actuator output through simulated joint-state publication.

For physical deployment, the final output stage can be replaced with a hardware interface while retaining the Cartesian planning and IK logic.

Important real-hardware additions include:

Encoder feedback
Calibration and zero-offset handling
Velocity and acceleration limits
Current/torque limits
Closed-loop trajectory tracking
Communication watchdogs
Emergency-stop handling
Fault recovery
Timestamped feedback
Latency and jitter monitoring

See PART2_SIM_TO_REAL.md for the detailed discussion.

Documentation
DESIGN_NOTE.md — architecture, planning, safety, trade-offs, and future work
PART2_SIM_TO_REAL.md — sim-to-real considerations and hardware architecture
Validation

The implementation was tested for:

Normal Cartesian target execution
PICK target execution
PLACE target execution
Editable PICK/PLACE mission targets
Full pick-and-place sequence
Workspace edge case (7.0, 3.0)
Target projection
Joint-limit and ground-constraint validation
50 Hz trajectory publication
GUI/ROS 2 operation
Clean GUI shutdown


If those are already present in the actual `README.md`, **don't change anything**.

Now commit it:

```bash
cd ~/kineshia_ros2_arm_task
git add README.md
git diff --cached --check
git commit -m "docs: finalize submission documentation"



