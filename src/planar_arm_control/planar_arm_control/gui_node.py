#!/usr/bin/env python3

import sys
import signal
import time
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg

from planar_arm_control.planar_arm import PlanarArm


LINK_LENGTHS = [3.0, 2.0, 1.5]


class GuiNode(Node):
    """ROS 2 client node used by the PyQt5 operator interface."""

    def __init__(self):
        super().__init__("gui_node")

        self.arm = PlanarArm(LINK_LENGTHS)

        self.current_q = np.radians([45.0, 0.0, 0.0])
        self.status = "WAITING"
        self.target = None

        # Pick-and-place sequence state machine.
        self.sequence_state = "IDLE"
        self.sequence_saw_moving = False
        self.sequence_pause_until = 0.0

        self.joint_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.status_sub = self.create_subscription(
            String,
            "/controller_status",
            self.status_callback,
            10,
        )

        self.target_pub = self.create_publisher(
            PointStamped,
            "/target_pose",
            10,
        )

        # History for plotting.
        self.history_time = deque(maxlen=500)
        self.history_q = [
            deque(maxlen=500),
            deque(maxlen=500),
            deque(maxlen=500),
        ]

        self.start_time = time.monotonic()

        self.window = None

        self.get_logger().info("GUI ROS node started.")


    def joint_state_callback(self, msg):
        if len(msg.position) >= 3:
            self.current_q = np.asarray(
                msg.position[:3],
                dtype=float,
            )

            now = time.monotonic() - self.start_time

            self.history_time.append(now)

            for i in range(3):
                self.history_q[i].append(
                    np.degrees(self.current_q[i])
                )


    def status_callback(self, msg):
        self.status = msg.data

        # Advance the GUI-side pick-and-place state machine.
        if self.sequence_state == "MOVING_TO_PICK":
            if msg.data == "MOVING":
                self.sequence_saw_moving = True

            elif msg.data == "REACHED" and self.sequence_saw_moving:
                self.sequence_state = "PICKED"
                self.sequence_pause_until = time.monotonic() + 0.5
                self.get_logger().info(
                    "Pick position reached. Simulating PICK action."
                )

        elif self.sequence_state == "MOVING_TO_PLACE":
            if msg.data == "MOVING":
                self.sequence_saw_moving = True

            elif msg.data == "REACHED" and self.sequence_saw_moving:
                self.sequence_state = "COMPLETE"
                self.get_logger().info(
                    "Pick-and-place sequence complete."
                )


    def send_target(self, x, y):
        msg = PointStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"

        msg.point.x = float(x)
        msg.point.y = float(y)
        msg.point.z = 0.0

        self.target = np.array([float(x), float(y)])

        self.target_pub.publish(msg)

        self.get_logger().info(
            "GUI target sent: (%.2f, %.2f)" % (x, y)
        )


    def start_pick_and_place(self):
        if self.sequence_state not in ("IDLE", "COMPLETE"):
            self.get_logger().warn(
                "Pick-and-place sequence already running."
            )
            return

        self.sequence_state = "MOVING_TO_PICK"
        self.sequence_saw_moving = False
        self.sequence_pause_until = 0.0

        self.get_logger().info(
            "Starting pick-and-place sequence."
        )

        self.send_target(4.0, 2.0)


    def update_sequence(self):
        if self.sequence_state == "PICKED":
            if time.monotonic() >= self.sequence_pause_until:
                self.sequence_state = "MOVING_TO_PLACE"
                self.sequence_saw_moving = False

                self.get_logger().info(
                    "PICK action complete. Moving to PLACE target."
                )

                self.send_target(-3.0, 3.0)


class ArmWindow(QtWidgets.QMainWindow):
    """Main operator GUI."""

    def __init__(self, node):
        super().__init__()

        self.node = node

        self.setWindowTitle("Kineshia Robotics - Planar Arm Controller")
        self.resize(1200, 750)

        self.build_ui()

        # Main GUI refresh timer.
        self.gui_timer = QtCore.QTimer()
        self.gui_timer.timeout.connect(self.update_gui)
        self.gui_timer.start(50)  # 20 Hz GUI refresh


    def build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        # ==============================================================
        # PROFESSIONAL ROBOTICS CONSOLE THEME
        # ==============================================================

        self.setStyleSheet("""
            QMainWindow, QWidget {
                background: #0b1220;
                color: #e6edf7;
                font-family: "DejaVu Sans";
            }

            QLabel {
                color: #d8e2f0;
            }

            QGroupBox {
                background: #111b2e;
                border: 1px solid #263650;
                border-radius: 8px;
                margin-top: 10px;
                padding: 10px;
                font-weight: bold;
                color: #8fc7ff;
            }

            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 5px;
            }

            QPushButton {
                background: #18263d;
                border: 1px solid #31506f;
                border-radius: 6px;
                padding: 8px 10px;
                color: #e6edf7;
                font-weight: 600;
            }

            QPushButton:hover {
                background: #213653;
                border-color: #4d8fc9;
            }

            QPushButton:pressed {
                background: #10243b;
            }

            QDoubleSpinBox {
                background: #0a1424;
                border: 1px solid #304662;
                border-radius: 5px;
                padding: 6px;
                color: #ffffff;
            }

            QDoubleSpinBox:focus {
                border-color: #4db6ff;
            }

            QScrollBar:vertical {
                background: #0b1220;
                width: 10px;
            }

            QScrollBar::handle:vertical {
                background: #263650;
                border-radius: 5px;
            }
        """)

        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setContentsMargins(14, 12, 14, 12)
        main_layout.setSpacing(10)

        # ==============================================================
        # HEADER
        # ==============================================================

        header = QtWidgets.QFrame()
        header.setStyleSheet("""
            QFrame {
                background: #111b2e;
                border: 1px solid #263650;
                border-radius: 9px;
            }
        """)

        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(16, 10, 16, 10)

        title_box = QtWidgets.QVBoxLayout()

        title = QtWidgets.QLabel("KINESHIA ROBOTICS")
        title.setStyleSheet(
            "font-size: 22px; font-weight: 800; color: #ffffff;"
        )

        subtitle = QtWidgets.QLabel(
            "PLANAR ARM CONTROL  •  3-DOF OPERATOR CONSOLE"
        )
        subtitle.setStyleSheet(
            "font-size: 11px; color: #7fa4c7;"
        )

        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header_layout.addLayout(title_box)

        header_layout.addStretch()

        self.connection_label = QtWidgets.QLabel(
            "●  ROS 2 CONNECTED"
        )
        self.connection_label.setStyleSheet(
            "font-size: 12px; font-weight: bold; color: #55e6a5;"
        )

        rate_label = QtWidgets.QLabel("50 Hz CONTROLLER")
        rate_label.setStyleSheet(
            "font-size: 11px; color: #8fa7c2;"
        )

        mode_label = QtWidgets.QLabel("POSITION MODE")
        mode_label.setStyleSheet(
            "font-size: 11px; color: #8fa7c2;"
        )

        header_layout.addWidget(rate_label)
        header_layout.addSpacing(18)
        header_layout.addWidget(mode_label)
        header_layout.addSpacing(18)
        header_layout.addWidget(self.connection_label)

        main_layout.addWidget(header)

        # ==============================================================
        # MAIN CONTENT
        # ==============================================================

        content_layout = QtWidgets.QHBoxLayout()
        content_layout.setSpacing(12)

        # ==============================================================
        # LEFT: ROBOT VIEW
        # ==============================================================

        left_layout = QtWidgets.QVBoxLayout()
        left_layout.setSpacing(10)

        self.arm_plot = pg.PlotWidget()
        self.arm_plot.setBackground("#080f1c")

        self.arm_plot.setLabel(
            "left", "Y", units="m",
            color="#8fa7c2"
        )
        self.arm_plot.setLabel(
            "bottom", "X", units="m",
            color="#8fa7c2"
        )

        self.arm_plot.setTitle(
            "LIVE ROBOT STATE",
            color="#d8e2f0",
            size="12pt",
        )

        self.arm_plot.setXRange(-7.0, 7.0)
        self.arm_plot.setYRange(-1.0, 7.0)
        self.arm_plot.setAspectLocked(True)
        self.arm_plot.showGrid(
            x=True,
            y=True,
            alpha=0.18,
        )

        # Workspace boundary.
        theta = np.linspace(0.0, 2.0 * np.pi, 240)
        workspace_x = 6.5 * np.cos(theta)
        workspace_y = 6.5 * np.sin(theta)

        self.workspace_curve = self.arm_plot.plot(
            workspace_x,
            workspace_y,
            pen=pg.mkPen(
                "#31506f",
                width=1,
                style=QtCore.Qt.DashLine,
            ),
        )

        # Ground line.
        self.ground_curve = self.arm_plot.plot(
            [-7.0, 7.0],
            [0.0, 0.0],
            pen=pg.mkPen("#8b5e3c", width=2),
        )

        self.arm_curve = self.arm_plot.plot(
            [],
            [],
            pen=pg.mkPen("#4db6ff", width=6),
            symbol="o",
            symbolSize=14,
            symbolBrush="#4db6ff",
            symbolPen=pg.mkPen("#dff4ff", width=2),
        )

        # Requested target.
        self.target_curve = self.arm_plot.plot(
            [],
            [],
            pen=None,
            symbol="x",
            symbolSize=18,
            symbolPen=pg.mkPen("#ffcf5c", width=3),
        )

        # Projected reachable target.
        self.projected_target_curve = self.arm_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=13,
            symbolBrush="#ff8c42",
            symbolPen=pg.mkPen("#ffd2b0", width=2),
        )

        # End effector.
        self.ee_curve = self.arm_plot.plot(
            [],
            [],
            pen=None,
            symbol="o",
            symbolSize=11,
            symbolBrush="#55e6a5",
            symbolPen=pg.mkPen("#d9ffec", width=2),
        )

        left_layout.addWidget(self.arm_plot, stretch=4)

        # Target information.
        target_info = QtWidgets.QFrame()
        target_info.setStyleSheet("""
            QFrame {
                background: #111b2e;
                border: 1px solid #263650;
                border-radius: 8px;
            }
        """)

        target_info_layout = QtWidgets.QHBoxLayout(target_info)
        target_info_layout.setContentsMargins(12, 8, 12, 8)

        self.target_status_label = QtWidgets.QLabel(
            "TARGET  •  NONE"
        )
        self.target_status_label.setStyleSheet(
            "font-weight: bold; color: #8fa7c2;"
        )

        target_info_layout.addWidget(self.target_status_label)
        target_info_layout.addStretch()

        legend = QtWidgets.QLabel(
            "× Requested    ● Projected    ● End Effector"
        )
        legend.setStyleSheet(
            "font-size: 10px; color: #7f94ad;"
        )

        target_info_layout.addWidget(legend)

        left_layout.addWidget(target_info)

        # ==============================================================
        # JOINT HISTORY
        # ==============================================================

        plot_group = QtWidgets.QGroupBox("JOINT TELEMETRY  •  LIVE HISTORY")
        plot_layout = QtWidgets.QVBoxLayout(plot_group)

        self.joint_plot = pg.PlotWidget()
        self.joint_plot.setBackground("#080f1c")

        self.joint_plot.setLabel(
            "left", "Angle", units="deg",
            color="#8fa7c2"
        )

        self.joint_plot.setLabel(
            "bottom", "Time", units="s",
            color="#8fa7c2"
        )

        self.joint_plot.showGrid(
            x=True,
            y=True,
            alpha=0.15,
        )

        self.joint_plot.setYRange(-130, 190)

        self.joint_curves = [
            self.joint_plot.plot(
                [],
                [],
                pen=pg.mkPen("#4db6ff", width=2),
                name="J1",
            ),
            self.joint_plot.plot(
                [],
                [],
                pen=pg.mkPen("#55e6a5", width=2),
                name="J2",
            ),
            self.joint_plot.plot(
                [],
                [],
                pen=pg.mkPen("#ffcf5c", width=2),
                name="J3",
            ),
        ]

        plot_layout.addWidget(self.joint_plot)
        left_layout.addWidget(plot_group, stretch=2)

        content_layout.addLayout(left_layout, stretch=7)

        # ==============================================================
        # RIGHT: OPERATOR PANEL
        # ==============================================================

        right = QtWidgets.QVBoxLayout()
        right.setSpacing(9)

        # --------------------------------------------------------------
        # STATUS
        # --------------------------------------------------------------

        status_group = QtWidgets.QGroupBox("SYSTEM STATUS")
        status_layout = QtWidgets.QVBoxLayout(status_group)

        self.status_label = QtWidgets.QLabel("●  WAITING")
        self.status_label.setMinimumHeight(34)
        self.status_label.setStyleSheet(
            "font-size: 17px; font-weight: 800; color: #8fa7c2;"
        )

        status_layout.addWidget(self.status_label)

        self.sequence_label = QtWidgets.QLabel(
            "Sequence  •  IDLE"
        )
        self.sequence_label.setStyleSheet(
            "font-size: 11px; color: #8fa7c2;"
        )

        status_layout.addWidget(self.sequence_label)

        right.addWidget(status_group)

        # --------------------------------------------------------------
        # TELEMETRY
        # --------------------------------------------------------------

        telemetry_group = QtWidgets.QGroupBox("LIVE TELEMETRY")
        telemetry_layout = QtWidgets.QVBoxLayout(telemetry_group)

        self.ee_label = QtWidgets.QLabel(
            "END EFFECTOR\n"
            "X   0.000 m\n"
            "Y   0.000 m"
        )

        self.ee_label.setStyleSheet(
            "font-size: 14px; font-weight: 600; "
            "color: #55e6a5; padding: 4px;"
        )

        telemetry_layout.addWidget(self.ee_label)

        telemetry_layout.addSpacing(5)

        self.joint_labels = []

        for i in range(3):
            label = QtWidgets.QLabel(
                "J%d     0.00°" % (i + 1)
            )

            label.setStyleSheet(
                "font-size: 13px; "
                "color: #d8e2f0; padding: 3px;"
            )

            telemetry_layout.addWidget(label)
            self.joint_labels.append(label)

        right.addWidget(telemetry_group)

        # --------------------------------------------------------------
        # TARGET COMMAND
        # --------------------------------------------------------------

        target_group = QtWidgets.QGroupBox("TARGET COMMAND")
        target_layout = QtWidgets.QGridLayout(target_group)
        target_layout.setSpacing(7)

        x_label = QtWidgets.QLabel("X  [m]")
        y_label = QtWidgets.QLabel("Y  [m]")

        self.x_input = QtWidgets.QDoubleSpinBox()
        self.x_input.setRange(-10.0, 10.0)
        self.x_input.setDecimals(2)
        self.x_input.setSingleStep(0.1)
        self.x_input.setValue(4.0)

        self.y_input = QtWidgets.QDoubleSpinBox()
        self.y_input.setRange(-10.0, 10.0)
        self.y_input.setDecimals(2)
        self.y_input.setSingleStep(0.1)
        self.y_input.setValue(2.0)

        target_layout.addWidget(x_label, 0, 0)
        target_layout.addWidget(self.x_input, 0, 1)
        target_layout.addWidget(y_label, 1, 0)
        target_layout.addWidget(self.y_input, 1, 1)

        send_button = QtWidgets.QPushButton("SEND TARGET")
        send_button.setMinimumHeight(36)
        send_button.setStyleSheet("""
            QPushButton {
                background: #1769aa;
                border: 1px solid #4db6ff;
                color: white;
                font-weight: 800;
            }
            QPushButton:hover {
                background: #2182c9;
            }
        """)

        send_button.clicked.connect(
            self.send_custom_target
        )

        target_layout.addWidget(
            send_button,
            2,
            0,
            1,
            2,
        )

        right.addWidget(target_group)

        # --------------------------------------------------------------
        # QUICK ACTIONS
        # --------------------------------------------------------------

        scenario_group = QtWidgets.QGroupBox(
            "MISSION QUICK ACTIONS"
        )

        scenario_layout = QtWidgets.QVBoxLayout(
            scenario_group
        )
        scenario_layout.setSpacing(6)

        pick_button = QtWidgets.QPushButton(
            "PICK     (4.0, 2.0)"
        )

        pick_button.clicked.connect(
            lambda: self.send_target(4.0, 2.0)
        )

        scenario_layout.addWidget(pick_button)

        place_button = QtWidgets.QPushButton(
            "PLACE   (-3.0, 3.0)"
        )

        place_button.clicked.connect(
            lambda: self.send_target(-3.0, 3.0)
        )

        scenario_layout.addWidget(place_button)

        edge_button = QtWidgets.QPushButton(
            "EDGE CASE     (7.0, 3.0)"
        )

        edge_button.setStyleSheet("""
            QPushButton {
                background: #2b2114;
                border: 1px solid #8a6424;
                color: #ffcf5c;
            }
            QPushButton:hover {
                background: #3a2c18;
            }
        """)

        edge_button.clicked.connect(
            lambda: self.send_target(7.0, 3.0)
        )

        scenario_layout.addWidget(edge_button)

        sequence_button = QtWidgets.QPushButton(
            "▶  RUN PICK & PLACE"
        )

        sequence_button.setMinimumHeight(42)
        sequence_button.setStyleSheet("""
            QPushButton {
                background: #176c50;
                border: 1px solid #55e6a5;
                color: #eafff5;
                font-size: 13px;
                font-weight: 800;
            }
            QPushButton:hover {
                background: #218764;
            }
        """)

        sequence_button.clicked.connect(
            self.node.start_pick_and_place
        )

        scenario_layout.addWidget(sequence_button)

        right.addWidget(scenario_group)

        # --------------------------------------------------------------
        # TARGET / SAFETY INFORMATION
        # --------------------------------------------------------------

        info_group = QtWidgets.QGroupBox("PLANNING / SAFETY")
        info_layout = QtWidgets.QVBoxLayout(info_group)

        info = QtWidgets.QLabel(
            "Workspace radius    6.50 m\n"
            "Ground constraint   Y ≥ 0\n"
            "J1                   0° … 180°\n"
            "J2 / J3             −120° … 120°"
        )

        info.setStyleSheet(
            "font-size: 11px; color: #8fa7c2; "
            "line-height: 140%;"
        )

        info_layout.addWidget(info)
        right.addWidget(info_group)

        right.addStretch()

        content_layout.addLayout(right, stretch=3)

        main_layout.addLayout(content_layout, stretch=1)

    def send_target(self, x, y):
        self.node.send_target(x, y)


    def send_custom_target(self):
        self.send_target(
            self.x_input.value(),
            self.y_input.value(),
        )


    def closeEvent(self, event):
        self.gui_timer.stop()

        if rclpy.ok():
            self.node.destroy_node()

        event.accept()

    def update_gui(self):
        # --------------------------------------------------------------
        # ROS spinning
        # --------------------------------------------------------------
        #
        # Keep ROS callbacks responsive without blocking Qt.
        #
        if not rclpy.ok():
            self.gui_timer.stop()
            return

        try:
            rclpy.spin_once(
                self.node,
                timeout_sec=0.0,
            )
        except rclpy.exceptions.RCLError:
            self.gui_timer.stop()
            return

        self.node.update_sequence()

        self.sequence_label.setText(
            "Sequence: %s"
            % self.node.sequence_state
        )

        q = self.node.current_q

        # --------------------------------------------------------------
        # Arm visualization
        # --------------------------------------------------------------

        points = self.node.arm.forward_kinematics(q)

        xs = [p[0] for p in points]
        ys = [p[1] for p in points]

        self.arm_curve.setData(
            xs,
            ys,
        )

        # --------------------------------------------------------------
        # Target visualization
        # --------------------------------------------------------------

        if self.node.target is not None:
            tx = self.node.target[0]
            ty = self.node.target[1]

            self.target_curve.setData(
                [tx],
                [ty],
            )

        # --------------------------------------------------------------
        # End-effector telemetry
        # --------------------------------------------------------------

        ee = self.node.arm.end_effector(q)

        self.ee_label.setText(
            "X: %.3f m\nY: %.3f m"
            % (ee[0], ee[1])
        )

        # --------------------------------------------------------------
        # Joint telemetry
        # --------------------------------------------------------------

        for i in range(3):
            self.joint_labels[i].setText(
                "Joint %d: %.2f°"
                % (
                    i + 1,
                    np.degrees(q[i]),
                )
            )

        # --------------------------------------------------------------
        # Status
        # --------------------------------------------------------------

        self.status_label.setText(
            self.node.status
        )

        # --------------------------------------------------------------
        # Joint history
        # --------------------------------------------------------------

        if len(self.node.history_time) > 1:
            t = np.asarray(
                self.node.history_time
            )

            for i in range(3):
                values = np.asarray(
                    self.node.history_q[i]
                )

                self.joint_curves[i].setData(
                    t,
                    values,
                )


def main(args=None):
    rclpy.init(args=args)

    app = QtWidgets.QApplication(sys.argv)

    # Convert Ctrl+C into a normal Qt shutdown.
    signal.signal(signal.SIGINT, lambda *_: app.quit())

    node = GuiNode()
    window = ArmWindow(node)

    window.show()

    try:
        exit_code = app.exec_()
    finally:
        window.close()
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
