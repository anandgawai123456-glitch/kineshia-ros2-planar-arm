# Part 2 — Sim-to-Real Considerations

Moving this system from simulation to a physical robotic arm introduces differences in sensing, actuation, timing, communication, and mechanical behavior.

The current controller generates a desired joint trajectory and publishes simulated joint states. On real hardware, this final output layer should become a hardware interface responsible for converting desired joint positions into actuator commands. The planner and IK logic should remain independent of the hardware-specific implementation.

Real joints introduce encoder noise, backlash, friction, calibration offsets, mechanical tolerances, actuator saturation, and communication latency. The controller should therefore compare commanded and measured joint positions and expose trajectory-tracking error to the operator.

Physical operation also requires additional safety mechanisms. These include velocity and acceleration limits, actuator current/torque limits, communication watchdogs, command timeouts, emergency-stop handling, and fault recovery.

Timing is another important difference. The simulated 50 Hz controller can behave deterministically, while real communication and operating-system scheduling introduce latency and jitter. Commands and measurements should therefore be timestamped, stale feedback should be detected, and defined behavior should exist for communication loss.

The deployment process should be progressive: first validate the hardware interface without motion, then perform low-speed single-joint tests, calibrate joint zero positions, test coordinated motion with conservative limits, and finally validate Cartesian trajectories.

The main architectural change is therefore not to replace the planner, but to introduce a reliable hardware interface and closed-loop feedback path:

```text
Cartesian Target
       ↓
       IK
       ↓
Trajectory Planner
       ↓
Safety Layer
       ↓
Hardware Interface
       ↓
Actuators
       ↓
Encoders
       ↓
Measured Joint State
       └──────────────→ Controller / GUI
```

This preserves the existing planning concept while replacing simulation assumptions with measured physical behavior.
