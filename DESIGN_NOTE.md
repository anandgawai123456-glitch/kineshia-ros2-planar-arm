# 3-DoF Planar Arm — Design Note

## Architecture

The system consists of two ROS 2 nodes: a controller and an operator GUI.

```text
Operator
   │
   ▼
PyQt5/PyQtGraph GUI
   │ /target_pose
   ▼
ROS 2 Controller
   │
   ├── Reachability
   ├── Inverse Kinematics
   ├── Safety Validation
   └── Quintic Trajectory
           │
           ▼
     Joint-state output
           │
           ▼
      GUI telemetry
```

The GUI uses PyQt5/PyQtGraph and services ROS 2 callbacks without blocking the Qt event loop. The controller receives Cartesian targets, validates them, solves IK, generates a smooth joint-space trajectory, and publishes `sensor_msgs/JointState` at 50 Hz.

## Planning and Safety

For every target, the controller first checks workspace reachability. An unreachable target is projected to the closest reachable boundary and the condition is surfaced to the operator.

The resulting IK solution is checked against the joint limits and the ground constraint (`y >= 0`). Invalid solutions are rejected.

A fifth-order/quintic time-scaling function is used for joint interpolation:

`10τ³ - 15τ⁴ + 6τ⁵`

This gives zero velocity and acceleration at the beginning and end of the trajectory while remaining computationally lightweight.

## GUI

The operator console provides:

* live arm visualization
* joint telemetry
* end-effector position
* joint-angle history
* target input
* quick target commands
* pick-and-place execution
* workspace/edge-case feedback

The GUI and ROS 2 event handling are integrated using a Qt timer and non-blocking ROS spinning.

## Hardware Swappability

The current implementation publishes simulated joint states after trajectory generation. For physical deployment, this final output stage can be replaced by a hardware backend such as a Dynamixel interface.

The Cartesian target handling, IK, safety validation, and trajectory-generation logic should remain independent of the actuator implementation.

## Key Trade-offs

A quintic trajectory was selected because it provides smooth endpoint conditions with low computational complexity. A ROS 2 topic is sufficient for individual target commands in this exercise. A ROS 2 Action would be more appropriate for a production pick-and-place operation because it provides feedback, cancellation, and explicit completion.

The current safety/planning responsibilities are logically separated but remain lightweight within the controller implementation rather than being split into many independent packages. This keeps the take-home system compact while leaving a clear path for future modularization.

## Further Development

With additional time, I would introduce an explicit planner/safety/backend interface, a ROS 2 Action for pick-and-place, closed-loop trajectory tracking, actuator limits, watchdogs, fault handling, Gazebo/ros_gz validation, and automated robustness/timing tests.
