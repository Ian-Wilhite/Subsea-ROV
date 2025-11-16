"""Telemetry visualization UI scaffold."""

from __future__ import annotations

import random
from typing import Dict, Tuple

try:  # pragma: no cover - import resolution depends on the host environment.
    from PyQt6 import QtCore, QtGui, QtWidgets  # type: ignore
except ImportError:  # pragma: no cover
    try:
        from PyQt5 import QtCore, QtGui, QtWidgets  # type: ignore
    except ImportError:
        try:
            from PySide6 import QtCore, QtGui, QtWidgets  # type: ignore
        except ImportError as exc:  # pragma: no cover
            raise ImportError(
                "A Qt binding (PyQt6, PyQt5, or PySide6) is required to use telemetry_ui_app."
            ) from exc

AlignmentFlag = getattr(QtCore.Qt, "AlignmentFlag", QtCore.Qt)
FrameShape = getattr(QtWidgets.QFrame, "Shape", QtWidgets.QFrame)


class StatusBadge(QtWidgets.QLabel):
    """Simple colored label used for health strip indicators."""

    def __init__(self, title: str, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(title, parent)
        self.setAlignment(AlignmentFlag.AlignCenter)
        self.setMargin(6)
        self.setMinimumWidth(110)
        self.setStyleSheet(
            "background: #3a3a3a; color: white; border-radius: 6px; font-weight: bold;"
        )

    def set_state(self, level: str) -> None:
        palette = {
            "ok": "#2e8b57",
            "warn": "#c99a2d",
            "alarm": "#c0392b",
            "stale": "#7f8c8d",
        }
        color = palette.get(level, "#2e8b57")
        self.setStyleSheet(
            f"background: {color}; color: white; border-radius: 6px; font-weight: bold;"
        )


class JoystickAxisIndicator(QtWidgets.QWidget):
    """Widget showing an axis value and its numeric readout."""

    def __init__(self, axis_name: str, parent: QtWidgets.QWidget | None = None) -> None:
        super().__init__(parent)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.label = QtWidgets.QLabel(axis_name)
        self.label.setAlignment(AlignmentFlag.AlignCenter)
        self.bar = QtWidgets.QProgressBar()
        self.bar.setRange(-100, 100)
        self.bar.setValue(0)
        self.bar.setFormat("%v")
        layout.addWidget(self.label)
        layout.addWidget(self.bar)

    def set_value(self, value: float) -> None:
        self.bar.setValue(int(value * 100))


class TelemetryMainWindow(QtWidgets.QMainWindow):
    """Dashboard-style main window that composes all telemetry panes."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("Subsea ROV Telemetry")
        self.resize(1600, 900)

        self._axis_widgets: Dict[str, JoystickAxisIndicator] = {}
        self._health_badges: Dict[str, StatusBadge] = {}
        self._mission_running = False
        self._mission_elapsed_ms = 0
        self._mission_timer = QtCore.QElapsedTimer()
        self._battery_level = 100.0

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        root_layout = QtWidgets.QVBoxLayout(central)
        self._grid = QtWidgets.QGridLayout()
        root_layout.addLayout(self._grid, stretch=1)
        root_layout.addWidget(self._build_health_strip())

        self._camera_pane = self._build_camera_pane()
        self._viz_pane = self._build_visualization_pane()
        self._user_input_pane = self._build_user_input_pane()
        self._runtime_pane = self._build_runtime_pane()

        self._grid.addWidget(self._camera_pane, 0, 0, 2, 2)
        self._grid.addWidget(self._viz_pane, 0, 2)
        self._grid.addWidget(self._user_input_pane, 1, 2)
        self._grid.addWidget(self._runtime_pane, 2, 0, 1, 2)

        for row, stretch in enumerate((3, 2, 1)):
            self._grid.setRowStretch(row, stretch)
        for col, stretch in enumerate((3, 1, 1)):
            self._grid.setColumnStretch(col, stretch)

        self._telemetry_timer = QtCore.QTimer(self)
        self._telemetry_timer.timeout.connect(self._simulate_telemetry_update)
        self._telemetry_timer.start(1000)

    def _build_camera_pane(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Dual Camera View")
        layout = QtWidgets.QVBoxLayout(group)

        streams_layout = QtWidgets.QHBoxLayout()
        streams_layout.addWidget(self._make_video_placeholder("Front Camera"))
        streams_layout.addWidget(self._make_video_placeholder("Bottom Camera"))
        layout.addLayout(streams_layout)

        controls_layout = QtWidgets.QHBoxLayout()
        controls_layout.setSpacing(8)
        controls_layout.addWidget(self._make_control_button("Prev"))
        self._play_button = self._make_control_button("Play")
        self._play_button.setCheckable(True)
        self._play_button.toggled.connect(self._toggle_playback)
        controls_layout.addWidget(self._play_button)
        controls_layout.addWidget(self._make_control_button("Next"))

        self._record_button = self._make_control_button("Record")
        self._record_button.setCheckable(True)
        self._record_button.toggled.connect(self._toggle_recording)
        controls_layout.addWidget(self._record_button)

        controls_layout.addStretch()
        self._latency_label = QtWidgets.QLabel("Latency: -- ms")
        controls_layout.addWidget(self._latency_label)

        layout.addLayout(controls_layout)
        return group

    def _build_visualization_pane(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("3D State Visualization")
        layout = QtWidgets.QVBoxLayout(group)
        placeholder = QtWidgets.QLabel("OpenGL/VTK visualization placeholder")
        placeholder.setAlignment(AlignmentFlag.AlignCenter)
        placeholder.setFrameShape(FrameShape.StyledPanel)
        placeholder.setMinimumHeight(200)
        layout.addWidget(placeholder, stretch=1)

        form = QtWidgets.QFormLayout()
        self._pose_label = QtWidgets.QLabel("(0, 0, 0) m / (0, 0, 0) deg")
        self._accel_label = QtWidgets.QLabel("(0, 0, 0) m/s²")
        self._velocity_label = QtWidgets.QLabel("(0, 0, 0) m/s")
        form.addRow("Pose:", self._pose_label)
        form.addRow("Acceleration:", self._accel_label)
        form.addRow("Velocity:", self._velocity_label)
        layout.addLayout(form)
        return group

    def _build_user_input_pane(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("User Input HUD")
        layout = QtWidgets.QVBoxLayout(group)

        axes_layout = QtWidgets.QHBoxLayout()
        for axis in ("Surge", "Sway", "Heave", "Yaw"):
            widget = JoystickAxisIndicator(axis)
            self._axis_widgets[axis] = widget
            axes_layout.addWidget(widget)
        layout.addLayout(axes_layout)

        buttons_layout = QtWidgets.QHBoxLayout()
        for label in ("Arm", "Lights", "Stab Mode", "Auto Depth"):
            btn = QtWidgets.QPushButton(label)
            btn.setCheckable(True)
            buttons_layout.addWidget(btn)
        layout.addLayout(buttons_layout)
        return group

    def _build_runtime_pane(self) -> QtWidgets.QGroupBox:
        group = QtWidgets.QGroupBox("Runtime & Battery")
        layout = QtWidgets.QGridLayout(group)

        self._mission_clock_label = QtWidgets.QLabel("Mission Clock: 00:00:00")
        font = self._mission_clock_label.font()
        font.setPointSize(font.pointSize() + 2)
        self._mission_clock_label.setFont(font)
        layout.addWidget(self._mission_clock_label, 0, 0, 1, 3)

        self._mission_control_button = QtWidgets.QPushButton("Start Mission")
        self._mission_control_button.clicked.connect(self._toggle_mission_clock)
        layout.addWidget(self._mission_control_button, 1, 0)

        reset_btn = QtWidgets.QPushButton("Reset Clock")
        reset_btn.clicked.connect(self._reset_mission_clock)
        layout.addWidget(reset_btn, 1, 1)

        layout.addWidget(QtWidgets.QLabel("Battery Estimate"), 2, 0, 1, 3)
        self._battery_bar = QtWidgets.QProgressBar()
        self._battery_bar.setValue(int(self._battery_level))
        layout.addWidget(self._battery_bar, 3, 0, 1, 3)

        self._bag_status_label = QtWidgets.QLabel("Rosbag: idle")
        layout.addWidget(self._bag_status_label, 4, 0, 1, 2)

        self._disk_usage_label = QtWidgets.QLabel("Disk Usage: --%")
        layout.addWidget(self._disk_usage_label, 4, 2)
        return group

    def _build_health_strip(self) -> QtWidgets.QWidget:
        widget = QtWidgets.QGroupBox("Health Summary")
        layout = QtWidgets.QHBoxLayout(widget)
        layout.setContentsMargins(12, 6, 12, 6)

        for title in ("Voltage", "Temps", "Leaks", "Link Quality", "Latency"):
            badge = StatusBadge(title)
            self._health_badges[title] = badge
            layout.addWidget(badge)
        layout.addStretch()
        return widget

    @staticmethod
    def _make_control_button(text: str) -> QtWidgets.QPushButton:
        button = QtWidgets.QPushButton(text)
        button.setMinimumWidth(90)
        return button

    @staticmethod
    def _make_video_placeholder(title: str) -> QtWidgets.QWidget:
        frame = QtWidgets.QFrame()
        frame.setFrameShape(FrameShape.StyledPanel)
        frame.setMinimumSize(QtCore.QSize(320, 240))
        layout = QtWidgets.QVBoxLayout(frame)
        label = QtWidgets.QLabel(title)
        label.setAlignment(AlignmentFlag.AlignCenter)
        layout.addWidget(label, alignment=AlignmentFlag.AlignTop)
        feed_placeholder = QtWidgets.QLabel("Video Stream")
        feed_placeholder.setAlignment(AlignmentFlag.AlignCenter)
        feed_placeholder.setStyleSheet("background: #1f1f1f; color: #aaaaaa;")
        layout.addWidget(feed_placeholder, stretch=1)
        return frame

    def _toggle_playback(self, playing: bool) -> None:
        self._play_button.setText("Pause" if playing else "Play")

    def _toggle_recording(self, recording: bool) -> None:
        self._bag_status_label.setText("Rosbag: recording" if recording else "Rosbag: idle")
        self._record_button.setText("Stop Recording" if recording else "Record")

    def _toggle_mission_clock(self) -> None:
        if not self._mission_running:
            self._mission_timer.start()
            self._mission_running = True
            self._mission_control_button.setText("Pause Mission")
        else:
            self._mission_elapsed_ms += self._mission_timer.elapsed()
            self._mission_running = False
            self._mission_control_button.setText("Resume Mission")

    def _reset_mission_clock(self) -> None:
        self._mission_running = False
        self._mission_elapsed_ms = 0
        self._mission_control_button.setText("Start Mission")
        self._mission_clock_label.setText("Mission Clock: 00:00:00")

    def _simulate_telemetry_update(self) -> None:
        self._update_mission_clock()
        self._update_latency()
        self._update_axes()
        self._update_health()
        self._update_disk_usage()

    def _update_mission_clock(self) -> None:
        elapsed_ms = self._mission_elapsed_ms
        if self._mission_running:
            elapsed_ms += self._mission_timer.elapsed()
        seconds = elapsed_ms // 1000
        h, rem = divmod(seconds, 3600)
        m, s = divmod(rem, 60)
        self._mission_clock_label.setText(f"Mission Clock: {h:02}:{m:02}:{s:02}")

        if self._mission_running:
            self._battery_level = max(0.0, self._battery_level - random.uniform(0.1, 0.5))
            self._battery_bar.setValue(int(self._battery_level))

    def _update_latency(self) -> None:
        simulated_latency = random.uniform(80, 150)
        self._latency_label.setText(f"Latency: {simulated_latency:.0f} ms")

    def _update_axes(self) -> None:
        for widget in self._axis_widgets.values():
            widget.set_value(random.uniform(-1.0, 1.0))

    def _update_health(self) -> None:
        for key, badge in self._health_badges.items():
            state = random.choice(("ok", "ok", "warn", "alarm" if key == "Leaks" else "ok"))
            badge.set_state(state)

    def _update_disk_usage(self) -> None:
        usage = random.uniform(35, 70) if not self._record_button.isChecked() else random.uniform(50, 90)
        self._disk_usage_label.setText(f"Disk Usage: {usage:.0f}%")


def run() -> Tuple[int, TelemetryMainWindow]:
    """Launch the telemetry UI and return the exit code with the main window."""
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = TelemetryMainWindow()
    window.show()
    return app.exec(), window


if __name__ == "__main__":
    run()
