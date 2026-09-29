#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from planar_arm_control.planar_arm import PlanarArm


LINK_LENGTHS = [3.0, 2.0, 1.5]


class ControllerNode(Node):
    """
    Simulation controller for the 3-DoF planar arm.

    Command path:
        target_pose -> IK -> joint trajectory -> simulated joint state

    The trajectory generator is intentionally separated from the simulated
    state publisher so a hardware backend can replace the final stage later.
    """

    def __init__(self):
        super().__init__("controller_node")

        self.arm = PlanarArm(LINK_LENGTHS)

        # Parameters
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("trajectory_duration", 3.0)
        self.declare_parameter("control_mode", "position")

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )
        self.trajectory_duration = float(
            self.get_parameter("trajectory_duration").value
        )
        self.control_mode = str(
            self.get_parameter("control_mode").value
        )

        # ROS interfaces
        self.joint_pub = self.create_publisher(
            JointState,
            "/joint_states",
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/controller_status",
            10,
        )

        self.target_sub = self.create_subscription(
            PointStamped,
            "/target_pose",
            self.target_callback,
            10,
        )

        # Current simulated joint state.
        # Start in a safe configuration.
        self.current_q = np.radians([45.0, 0.0, 0.0])

        # Trajectory state
        self.trajectory_active = False
        self.trajectory_start_time = None
        self.trajectory_start_q = self.current_q.copy()
        self.trajectory_goal_q = self.current_q.copy()
        self.last_target = None

        self.joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
        ]

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.control_timer_callback,
        )

        # Publish initial state immediately.
        self.publish_joint_state()
        self.publish_status("IDLE")

        self.get_logger().info(
            "Controller started | rate=%.1f Hz | duration=%.2f s | mode=%s"
            % (
                self.publish_rate_hz,
                self.trajectory_duration,
                self.control_mode,
            )
        )

    # ------------------------------------------------------------------
    # Target handling
    # ------------------------------------------------------------------

    def target_callback(self, msg: PointStamped):
        target = np.array(
            [msg.point.x, msg.point.y],
            dtype=float,
        )

        self.last_target = target.copy()

        self.get_logger().info(
            "Received target: (%.3f, %.3f)"
            % (target[0], target[1])
        )

        # Detect the provided library's workspace projection.
        reachable_target = np.asarray(
            self.arm.reachable_target(target),
            dtype=float,
        )

        projected = not np.allclose(
            target,
            reachable_target,
            atol=1e-8,
        )

        if projected:
            self.get_logger().warn(
                "Target outside workspace. "
                "Projected to (%.3f, %.3f)"
                % (
                    reachable_target[0],
                    reachable_target[1],
                )
            )

        try:
            goal_q = np.asarray(
                self.arm.inverse_kinematics(
                    reachable_target,
                    initial_guess=self.current_q,
                ),
                dtype=float,
            )
        except Exception as exc:
            self.get_logger().error(
                "IK failed: %s" % exc
            )
            self.publish_status("IK_FAILED")
            return

        # The supplied Jacobian fallback does not explicitly enforce
        # constraints, so validate the final result here.
        if not self.arm.within_joint_limits(goal_q):
            self.get_logger().error(
                "IK result violates joint limits."
            )
            self.publish_status("INVALID_IK")
            return

        if not self.arm.arm_above_base(goal_q):
            self.get_logger().error(
                "IK result violates ground constraint."
            )
            self.publish_status("INVALID_IK")
            return

        actual_target = np.asarray(
            self.arm.end_effector(goal_q),
            dtype=float,
        )

        position_error = np.linalg.norm(
            reachable_target - actual_target
        )

        if position_error > 1e-3:
            self.get_logger().error(
                "IK result has position error %.4f m"
                % position_error
            )
            self.publish_status("IK_FAILED")
            return

        if projected:
            self.publish_status("TARGET_PROJECTED")
        else:
            self.publish_status("TARGET_ACCEPTED")

        self.start_trajectory(goal_q)

    # ------------------------------------------------------------------
    # Trajectory generation
    # ------------------------------------------------------------------

    @staticmethod
    def quintic_scale(tau):
        """
        Quintic time scaling with zero velocity and acceleration
        at both ends.
        """
        return (
            10.0 * tau**3
            - 15.0 * tau**4
            + 6.0 * tau**5
        )

    def start_trajectory(self, goal_q):
        self.trajectory_start_q = self.current_q.copy()
        self.trajectory_goal_q = goal_q.copy()

        self.trajectory_start_time = self.get_clock().now()
        self.trajectory_active = True

        self.publish_status("MOVING")

        self.get_logger().info(
            "Trajectory started | goal(deg)=%s"
            % np.round(np.degrees(goal_q), 2).tolist()
        )

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def control_timer_callback(self):
        if not self.trajectory_active:
            self.publish_joint_state()
            return

        elapsed = (
            self.get_clock().now()
            - self.trajectory_start_time
        ).nanoseconds / 1e9

        duration = max(self.trajectory_duration, 0.1)

        tau = min(max(elapsed / duration, 0.0), 1.0)

        scale = self.quintic_scale(tau)

        self.current_q = (
            self.trajectory_start_q
            + scale
            * (
                self.trajectory_goal_q
                - self.trajectory_start_q
            )
        )

        # Final safety validation.
        if not self.arm.within_joint_limits(self.current_q):
            self.get_logger().error(
                "Trajectory violated joint limits."
            )
            self.trajectory_active = False
            self.publish_status("SAFETY_STOP")
            return

        if not self.arm.arm_above_base(self.current_q):
            self.get_logger().error(
                "Trajectory violated ground constraint."
            )
            self.trajectory_active = False
            self.publish_status("SAFETY_STOP")
            return

        self.publish_joint_state()

        if tau >= 1.0:
            self.current_q = self.trajectory_goal_q.copy()
            self.trajectory_active = False

            self.publish_joint_state()
            self.publish_status("REACHED")

            ee = self.arm.end_effector(self.current_q)

            self.get_logger().info(
                "Target reached | EE=(%.3f, %.3f)"
                % (ee[0], ee[1])
            )

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def publish_joint_state(self):
        msg = JointState()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_q.tolist()

        msg.velocity = [0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0]

        self.joint_pub.publish(msg)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
