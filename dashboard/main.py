#!/usr/bin/env python3
"""
HoloAssist Dashboard — E-Stop & Debug Console
==============================================

Run:  source /opt/ros/humble/setup.bash && python dashboard/main.py
      (or with --no-ros to test UI without ROS)

Steam Deck recommended key mappings (via Steam Input):
  D-pad Left/Right  →  Left/Right arrow keys   (cycle tabs)
  A button           →  (click on-screen buttons with touchscreen/trackpad)

Window: 1280x800 (Steam Deck OLED native resolution)
"""

import math
import sys
import time
from datetime import datetime

from PyQt5.QtCore import Qt, QTimer, QElapsedTimer
from PyQt5.QtGui import QFont, QKeyEvent, QPainter, QColor, QPen
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTabWidget, QTextEdit, QGridLayout,
    QFrame, QProgressBar, QSizePolicy, QStackedWidget,
)

from ros_interface import RosInterface, RobotState, ROS_AVAILABLE, TOPIC_DEFAULTS


# ── Colours ─────────────────────────────────────────────────────────

DARK_BG = "#0d1117"
PANEL_BG = "#161b22"
BORDER = "#30363d"
TEXT = "#e6edf3"
TEXT_DIM = "#7d8590"
GREEN = "#3fb950"
RED = "#f85149"
YELLOW = "#d29922"
BLUE = "#58a6ff"
ORANGE = "#f0883e"


GLOBAL_STYLE = f"""
    QMainWindow, QWidget {{
        background-color: {DARK_BG};
        color: {TEXT};
    }}
    QLabel {{
        color: {TEXT};
    }}
    QTabWidget::pane {{
        border: 1px solid {BORDER};
        background: {PANEL_BG};
    }}
    QTabBar::tab {{
        background: {DARK_BG};
        color: {TEXT_DIM};
        padding: 4px 14px;
        margin-right: 2px;
        border: 1px solid {BORDER};
        border-bottom: none;
        font-size: 8px;
        font-weight: bold;
    }}
    QTabBar::tab:selected {{
        background: {PANEL_BG};
        color: {BLUE};
        border-bottom: 2px solid {BLUE};
    }}
    QTabBar::tab:hover {{
        color: {TEXT};
    }}
    QTextEdit {{
        background-color: {DARK_BG};
        color: {TEXT};
        border: 1px solid {BORDER};
        font-family: monospace;
        font-size: 8px;
    }}
    QFrame#separator {{
        background-color: {BORDER};
    }}
"""


# ── Status Bar Widget ───────────────────────────────────────────────

class StatusBar(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFixedHeight(22)
        self.setStyleSheet(f"background: {PANEL_BG}; border-bottom: 1px solid {BORDER};")

        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 0, 8, 0)

        self.title = QLabel("HOLOASSIST")
        self.title.setFont(QFont("monospace", 8, QFont.Bold))
        self.title.setStyleSheet(f"color: {BLUE};")
        layout.addWidget(self.title)

        layout.addStretch()

        self.ros_indicator = QLabel("ROS: ---")
        self.ros_indicator.setFont(QFont("monospace", 8))
        layout.addWidget(self.ros_indicator)

        self._add_separator(layout)

        self.ctrl_indicator = QLabel("CTRL: ---")
        self.ctrl_indicator.setFont(QFont("monospace", 8))
        layout.addWidget(self.ctrl_indicator)

        self._add_separator(layout)

        self.joint_hz_label = QLabel("JNT: --- Hz")
        self.joint_hz_label.setFont(QFont("monospace", 8))
        layout.addWidget(self.joint_hz_label)

        self._add_separator(layout)

        self.state_label = QLabel("STATE: ---")
        self.state_label.setFont(QFont("monospace", 8, QFont.Bold))
        layout.addWidget(self.state_label)

    def _add_separator(self, layout):
        sep = QLabel("|")
        sep.setStyleSheet(f"color: {BORDER};")
        sep.setFont(QFont("monospace", 8))
        layout.addWidget(sep)

    def update_status(self, status):
        if status.ros_connected:
            self.ros_indicator.setText("ROS: CONNECTED")
            self.ros_indicator.setStyleSheet(f"color: {GREEN};")
        else:
            self.ros_indicator.setText("ROS: OFFLINE")
            self.ros_indicator.setStyleSheet(f"color: {RED};")

        if status.controller_active:
            self.ctrl_indicator.setText("CTRL: ACTIVE")
            self.ctrl_indicator.setStyleSheet(f"color: {GREEN};")
        else:
            self.ctrl_indicator.setText("CTRL: INACTIVE")
            self.ctrl_indicator.setStyleSheet(f"color: {YELLOW};")

        if status.joint_hz > 0:
            self.joint_hz_label.setText(f"JNT: {status.joint_hz:.0f} Hz")
            self.joint_hz_label.setStyleSheet(f"color: {GREEN};")
        else:
            self.joint_hz_label.setText("JNT: --- Hz")
            self.joint_hz_label.setStyleSheet(f"color: {TEXT_DIM};")

        state = status.robot_state
        if state == RobotState.RUNNING:
            self.state_label.setText("STATE: RUNNING")
            self.state_label.setStyleSheet(f"color: {GREEN};")
        elif state == RobotState.ESTOPPED:
            self.state_label.setText("STATE: E-STOPPED")
            self.state_label.setStyleSheet(f"color: {RED};")
        elif state == RobotState.RESUMING:
            self.state_label.setText("STATE: RESUMING")
            self.state_label.setStyleSheet(f"color: {YELLOW};")
        else:
            self.state_label.setText("STATE: DISCONNECTED")
            self.state_label.setStyleSheet(f"color: {TEXT_DIM};")


# ── E-Stop Widget ──────────────────────────────────────────────────

class EstopWidget(QFrame):
    """
    Large e-stop column on the right side of the window.
    - Normal state: big red EMERGENCY STOP button
    - Estopped state: pulsing red background + yellow HOLD TO RESUME button with progress
    """

    RESUME_HOLD_MS = 5000  # 5 seconds
    COOLDOWN_MS = 1000     # 1 second cooldown after resume before estop is clickable

    def __init__(self, ros: RosInterface, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.setFixedWidth(120)
        self._estopped = False
        self._resume_held = False
        self._resume_elapsed = 0
        self._pulse_phase = 0.0
        self._cooldown_active = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)

        # Stacked widget: estop button vs resume button
        self.stack = QStackedWidget()
        layout.addWidget(self.stack)

        # Page 0: E-STOP button
        self.estop_btn = QPushButton("EMERGENCY\nSTOP")
        self.estop_btn.setFont(QFont("monospace", 11, QFont.Bold))
        self.estop_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.estop_btn.setCursor(Qt.PointingHandCursor)
        self.estop_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {RED};
                color: white;
                border: 3px solid #ff6b6b;
                border-radius: 12px;
            }}
            QPushButton:hover {{
                background-color: #ff2020;
                border-color: white;
            }}
            QPushButton:pressed {{
                background-color: #cc0000;
            }}
        """)
        self.estop_btn.clicked.connect(self._on_estop)
        self.stack.addWidget(self.estop_btn)

        # Page 1: Resume panel (shown during e-stop)
        resume_panel = QWidget()
        resume_layout = QVBoxLayout(resume_panel)
        resume_layout.setContentsMargins(0, 0, 0, 0)
        resume_layout.setSpacing(8)

        self.stopped_label = QLabel("ROBOT\nSTOPPED")
        self.stopped_label.setAlignment(Qt.AlignCenter)
        self.stopped_label.setFont(QFont("monospace", 8, QFont.Bold))
        self.stopped_label.setStyleSheet(f"color: {RED};")
        resume_layout.addWidget(self.stopped_label)

        self.resume_btn = QPushButton("HOLD TO\nRESUME\n(5s)")
        self.resume_btn.setFont(QFont("monospace", 8, QFont.Bold))
        self.resume_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.resume_btn.setCursor(Qt.PointingHandCursor)
        self.resume_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {YELLOW};
                color: black;
                border: 3px solid {ORANGE};
                border-radius: 12px;
            }}
            QPushButton:hover {{
                background-color: {ORANGE};
            }}
        """)
        self.resume_btn.pressed.connect(self._on_resume_press)
        self.resume_btn.released.connect(self._on_resume_release)
        resume_layout.addWidget(self.resume_btn)

        self.progress = QProgressBar()
        self.progress.setFixedHeight(10)
        self.progress.setRange(0, self.RESUME_HOLD_MS)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
        self.progress.setStyleSheet(f"""
            QProgressBar {{
                background-color: {DARK_BG};
                border: 1px solid {BORDER};
                border-radius: 5px;
            }}
            QProgressBar::chunk {{
                background-color: {GREEN};
                border-radius: 4px;
            }}
        """)
        resume_layout.addWidget(self.progress)

        self.stack.addWidget(resume_panel)

        # Timers
        self._resume_timer = QTimer()
        self._resume_timer.setInterval(50)
        self._resume_timer.timeout.connect(self._resume_tick)

        self._pulse_timer = QTimer()
        self._pulse_timer.setInterval(50)
        self._pulse_timer.timeout.connect(self._pulse_tick)

        self._cooldown_timer = QTimer()
        self._cooldown_timer.setSingleShot(True)
        self._cooldown_timer.timeout.connect(self._cooldown_done)

    def _on_estop(self):
        if self._cooldown_active:
            return  # ignore clicks during cooldown after resume
        self._estopped = True
        self.ros.emergency_stop()
        self.stack.setCurrentIndex(1)
        self.progress.setValue(0)
        self._resume_elapsed = 0
        self._pulse_timer.start()

    def _on_resume_press(self):
        self._resume_held = True
        self._resume_elapsed = 0
        self._elapsed_timer = QElapsedTimer()
        self._elapsed_timer.start()
        self._resume_timer.start()

    def _on_resume_release(self):
        self._resume_held = False
        self._resume_timer.stop()
        self._resume_elapsed = 0
        self.progress.setValue(0)
        self.resume_btn.setText("HOLD TO\nRESUME\n(5s)")

    def _resume_tick(self):
        if not self._resume_held:
            return
        self._resume_elapsed = self._elapsed_timer.elapsed()
        self.progress.setValue(min(self._resume_elapsed, self.RESUME_HOLD_MS))
        remaining = max(0, (self.RESUME_HOLD_MS - self._resume_elapsed) / 1000)
        self.resume_btn.setText(f"HOLD\n{remaining:.1f}s")

        if self._resume_elapsed >= self.RESUME_HOLD_MS:
            self._resume_timer.stop()
            self._pulse_timer.stop()
            self._estopped = False
            self._resume_held = False
            self.ros.resume()
            # Start cooldown — block estop clicks briefly so the
            # mouse-release from the resume hold doesn't re-trigger it
            self._cooldown_active = True
            self.estop_btn.setEnabled(False)
            self._cooldown_timer.start(self.COOLDOWN_MS)
            self.stack.setCurrentIndex(0)
            self.setStyleSheet("")

    def _cooldown_done(self):
        self._cooldown_active = False
        self.estop_btn.setEnabled(True)

    def _pulse_tick(self):
        self._pulse_phase += 0.15
        intensity = int(20 + 15 * (1 + math.sin(self._pulse_phase)))
        self.setStyleSheet(f"background-color: rgb({intensity}, 0, 0);")

    def sync_state(self, status):
        if status.robot_state == RobotState.ESTOPPED and not self._estopped:
            self._estopped = True
            self.stack.setCurrentIndex(1)
            self._pulse_timer.start()
        elif status.robot_state == RobotState.RUNNING and self._estopped:
            self._estopped = False
            self.stack.setCurrentIndex(0)
            self._pulse_timer.stop()
            self.setStyleSheet("")
            self.progress.setValue(0)


# ── Screen: Status ──────────────────────────────────────────────────

class StatusScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(8)

        # Left: joint states
        left = QVBoxLayout()
        left.setSpacing(1)
        left_title = QLabel("JOINT STATES")
        left_title.setFont(QFont("monospace", 8, QFont.Bold))
        left_title.setStyleSheet(f"color: {BLUE};")
        left.addWidget(left_title)

        self.joint_labels = []
        for i in range(6):
            lbl = QLabel(f"joint_{i}: ---")
            lbl.setFont(QFont("monospace", 8))
            lbl.setStyleSheet(f"color: {TEXT};")
            left.addWidget(lbl)
            self.joint_labels.append(lbl)
        left.addStretch()
        layout.addLayout(left, 1)

        # Vertical separator
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"color: {BORDER};")
        layout.addWidget(sep)

        # Right: event log
        right = QVBoxLayout()
        right_title = QLabel("EVENT LOG")
        right_title.setFont(QFont("monospace", 8, QFont.Bold))
        right_title.setStyleSheet(f"color: {BLUE};")
        right.addWidget(right_title)

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        right.addWidget(self.event_log)
        layout.addLayout(right, 2)

    def update_status(self, status):
        for i, lbl in enumerate(self.joint_labels):
            if i < len(status.joint_names):
                name = status.joint_names[i]
                pos_deg = math.degrees(status.joint_positions[i]) if i < len(status.joint_positions) else 0
                vel = status.joint_velocities[i] if i < len(status.joint_velocities) else 0
                lbl.setText(f"{name}: {pos_deg:+7.1f}°  v:{vel:+.2f}")
            else:
                lbl.setText(f"joint_{i}: waiting...")

        lines = []
        for ts, msg in status.events:
            t = datetime.fromtimestamp(ts).strftime("%H:%M:%S")
            lines.append(f"[{t}] {msg}")
        text = "\n".join(lines)
        if text != self.event_log.toPlainText():
            self.event_log.setPlainText(text)
            scrollbar = self.event_log.verticalScrollBar()
            scrollbar.setValue(scrollbar.maximum())


# ── Screen: Camera (debug image from depth tracker) ────────────────

class CameraScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        header = QHBoxLayout()
        title = QLabel("DEPTH CAMERA")
        title.setFont(QFont("monospace", 8, QFont.Bold))
        title.setStyleSheet(f"color: {BLUE};")
        header.addWidget(title)
        header.addStretch()
        self.camera_info = QLabel("Waiting for image...")
        self.camera_info.setFont(QFont("monospace", 8))
        self.camera_info.setStyleSheet(f"color: {TEXT_DIM};")
        header.addWidget(self.camera_info)
        layout.addLayout(header)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet(f"background: {DARK_BG}; border: 1px solid {BORDER};")
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.image_label, 1)

        topic_label = QLabel(f"Topic: {TOPIC_DEFAULTS['debug_image']}")
        topic_label.setFont(QFont("monospace", 8))
        topic_label.setStyleSheet(f"color: {TEXT_DIM};")
        layout.addWidget(topic_label)

    def update_status(self, status):
        rate = status.topic_rates.get("debug_image")
        hz = rate.hz if rate else 0
        if status.camera_jpeg and status.camera_width > 0:
            self.camera_info.setText(
                f"{status.camera_width}x{status.camera_height} @ {hz:.0f} Hz"
            )
            try:
                from PyQt5.QtGui import QImage, QPixmap
                encoding = getattr(status, '_camera_encoding', 'rgb8')
                w, h = status.camera_width, status.camera_height
                data = status.camera_jpeg
                if encoding in ('rgb8', 'bgr8'):
                    fmt = QImage.Format_RGB888
                    img = QImage(data, w, h, w * 3, fmt)
                    if encoding == 'bgr8':
                        img = img.rgbSwapped()
                elif encoding == 'mono8':
                    img = QImage(data, w, h, w, QImage.Format_Grayscale8)
                else:
                    self.camera_info.setText(f"Unsupported encoding: {encoding}")
                    return
                pixmap = QPixmap.fromImage(img)
                scaled = pixmap.scaled(
                    self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                )
                self.image_label.setPixmap(scaled)
            except Exception as e:
                self.camera_info.setText(f"Render error: {e}")
        else:
            if hz > 0:
                self.camera_info.setText(f"Receiving @ {hz:.0f} Hz (decoding...)")
            else:
                self.camera_info.setText("Waiting for image...")


# ── Screen: Headset (Quest 3 compressed image stream) ──────────────

class HeadsetScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        header = QHBoxLayout()
        title = QLabel("HEADSET VIEW")
        title.setFont(QFont("monospace", 8, QFont.Bold))
        title.setStyleSheet(f"color: {BLUE};")
        header.addWidget(title)
        header.addStretch()
        self.headset_info = QLabel("Waiting for stream...")
        self.headset_info.setFont(QFont("monospace", 8))
        self.headset_info.setStyleSheet(f"color: {TEXT_DIM};")
        header.addWidget(self.headset_info)
        layout.addLayout(header)

        self.image_label = QLabel()
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setStyleSheet(f"background: {DARK_BG}; border: 1px solid {BORDER};")
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        layout.addWidget(self.image_label, 1)

        topic_label = QLabel(f"Topic: {TOPIC_DEFAULTS['headset_image']}")
        topic_label.setFont(QFont("monospace", 8))
        topic_label.setStyleSheet(f"color: {TEXT_DIM};")
        layout.addWidget(topic_label)

    def update_status(self, status):
        rate = status.topic_rates.get("headset_image")
        hz = rate.hz if rate else 0
        if status.headset_jpeg:
            try:
                from PyQt5.QtGui import QImage, QPixmap
                img = QImage()
                img.loadFromData(status.headset_jpeg, "JPEG")
                if not img.isNull():
                    self.headset_info.setText(f"{img.width()}x{img.height()} @ {hz:.0f} Hz")
                    pixmap = QPixmap.fromImage(img)
                    scaled = pixmap.scaled(
                        self.image_label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation
                    )
                    self.image_label.setPixmap(scaled)
                else:
                    self.headset_info.setText("Decode error")
            except Exception as e:
                self.headset_info.setText(f"Error: {e}")
        else:
            if hz > 0:
                self.headset_info.setText(f"Receiving @ {hz:.0f} Hz...")
            else:
                self.headset_info.setText("Waiting for stream...")


# ── Screen: Stats ──────────────────────────────────────────────────

def _make_metric_label(parent_layout, text):
    lbl = QLabel(text)
    lbl.setFont(QFont("monospace", 8))
    lbl.setStyleSheet(f"color: {TEXT};")
    parent_layout.addWidget(lbl)
    return lbl


class StatsScreen(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(8)

        # Left column: topic rates
        left = QVBoxLayout()
        left.setSpacing(2)
        left_title = QLabel("TOPIC RATES")
        left_title.setFont(QFont("monospace", 8, QFont.Bold))
        left_title.setStyleSheet(f"color: {BLUE};")
        left.addWidget(left_title)

        self.rate_labels = {}
        for name, topic in TOPIC_DEFAULTS.items():
            row = QHBoxLayout()
            row.setSpacing(4)
            name_lbl = QLabel(name)
            name_lbl.setFont(QFont("monospace", 8))
            name_lbl.setStyleSheet(f"color: {TEXT_DIM};")
            name_lbl.setFixedWidth(80)
            row.addWidget(name_lbl)

            bar = QProgressBar()
            bar.setFixedHeight(8)
            bar.setRange(0, 100)
            bar.setTextVisible(False)
            bar.setStyleSheet(f"""
                QProgressBar {{ background: {DARK_BG}; border: 1px solid {BORDER}; border-radius: 2px; }}
                QProgressBar::chunk {{ background: {GREEN}; border-radius: 1px; }}
            """)
            row.addWidget(bar, 1)

            hz_lbl = QLabel("---")
            hz_lbl.setFont(QFont("monospace", 8))
            hz_lbl.setStyleSheet(f"color: {TEXT};")
            hz_lbl.setFixedWidth(40)
            hz_lbl.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            row.addWidget(hz_lbl)

            left.addLayout(row)
            self.rate_labels[name] = (bar, hz_lbl)

        left.addStretch()
        layout.addLayout(left, 2)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"color: {BORDER};")
        layout.addWidget(sep)

        # Right column: latency + poses + unity
        right = QVBoxLayout()
        right.setSpacing(2)

        lat_title = QLabel("LATENCY / AGES")
        lat_title.setFont(QFont("monospace", 8, QFont.Bold))
        lat_title.setStyleSheet(f"color: {BLUE};")
        right.addWidget(lat_title)

        self.target_age_lbl = _make_metric_label(right, "Target age: ---")
        self.twist_age_lbl = _make_metric_label(right, "Twist age:  ---")

        right.addSpacing(6)

        pose_title = QLabel("TARGET POSE")
        pose_title.setFont(QFont("monospace", 8, QFont.Bold))
        pose_title.setStyleSheet(f"color: {BLUE};")
        right.addWidget(pose_title)

        self.target_x = _make_metric_label(right, "X: ---")
        self.target_y = _make_metric_label(right, "Y: ---")
        self.target_z = _make_metric_label(right, "Z: ---")
        self.target_q = _make_metric_label(right, "Q: ---")

        right.addSpacing(6)

        unity_title = QLabel("UNITY")
        unity_title.setFont(QFont("monospace", 8, QFont.Bold))
        unity_title.setStyleSheet(f"color: {BLUE};")
        right.addWidget(unity_title)

        self.unity_map_lbl = _make_metric_label(right, "Map loaded: ---")

        right.addStretch()
        layout.addLayout(right, 1)

    def update_status(self, status):
        # Topic rates
        max_hz = {"debug_image": 30, "pointcloud": 30, "joint_states": 500,
                   "target_pose": 50, "twist_cmd": 50, "clicked_point": 10,
                   "bbox": 30, "obstacle": 10, "unity_map_loaded": 1,
                   "velocity_cmd": 50}
        for name, (bar, hz_lbl) in self.rate_labels.items():
            rate = status.topic_rates.get(name)
            hz = rate.hz if rate else 0.0
            cap = max_hz.get(name, 50)
            bar.setValue(min(int(hz / cap * 100), 100))
            if hz > 0:
                hz_lbl.setText(f"{hz:.0f}")
                hz_lbl.setStyleSheet(f"color: {GREEN};")
                bar.setStyleSheet(f"""
                    QProgressBar {{ background: {DARK_BG}; border: 1px solid {BORDER}; border-radius: 2px; }}
                    QProgressBar::chunk {{ background: {GREEN}; border-radius: 1px; }}
                """)
            else:
                hz_lbl.setText("---")
                hz_lbl.setStyleSheet(f"color: {TEXT_DIM};")

        # Latency ages
        def fmt_age(s):
            if s < 0:
                return "---"
            if s < 1:
                return f"{s*1000:.0f} ms"
            return f"{s:.1f} s"

        self.target_age_lbl.setText(f"Target pose age: {fmt_age(status.last_target_age_s)}")
        c = GREEN if 0 <= status.last_target_age_s < 1 else (YELLOW if status.last_target_age_s >= 1 else TEXT_DIM)
        self.target_age_lbl.setStyleSheet(f"color: {c};")

        self.twist_age_lbl.setText(f"Twist cmd age:   {fmt_age(status.last_twist_age_s)}")
        c = GREEN if 0 <= status.last_twist_age_s < 1 else (YELLOW if status.last_twist_age_s >= 1 else TEXT_DIM)
        self.twist_age_lbl.setStyleSheet(f"color: {c};")

        # Target pose
        tp = status.target_pose
        if tp:
            self.target_x.setText(f"X: {tp['x']:+.4f}")
            self.target_y.setText(f"Y: {tp['y']:+.4f}")
            self.target_z.setText(f"Z: {tp['z']:+.4f}")
            self.target_q.setText(f"Q: [{tp['qx']:.3f}, {tp['qy']:.3f}, {tp['qz']:.3f}, {tp['qw']:.3f}]")
        else:
            self.target_x.setText("X: ---")
            self.target_y.setText("Y: ---")
            self.target_z.setText("Z: ---")
            self.target_q.setText("Q: ---")

        # Unity
        um = status.unity_map_loaded
        if um is None:
            self.unity_map_lbl.setText("Map loaded: ---")
            self.unity_map_lbl.setStyleSheet(f"color: {TEXT_DIM};")
        elif um:
            self.unity_map_lbl.setText("Map loaded: YES")
            self.unity_map_lbl.setStyleSheet(f"color: {GREEN};")
        else:
            self.unity_map_lbl.setText("Map loaded: NO")
            self.unity_map_lbl.setStyleSheet(f"color: {YELLOW};")


# ── Main Window ─────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    POLL_INTERVAL_MS = 33  # ~30Hz UI updates

    def __init__(self, ros: RosInterface):
        super().__init__()
        self.ros = ros
        self.setWindowTitle("HoloAssist Dashboard")
        self.resize(1280, 800)
        self.setMinimumSize(800, 600)

        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.setSpacing(0)

        # Status bar (top)
        self.status_bar = StatusBar()
        main_layout.addWidget(self.status_bar)

        # Body: tabs (left) + estop (right)
        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)

        # Tab widget (left, fills remaining space)
        self.tabs = QTabWidget()
        self.tabs.setFont(QFont("monospace", 8))

        self.status_screen = StatusScreen()
        self.tabs.addTab(self.status_screen, "STATUS")

        self.headset_screen = HeadsetScreen()
        self.tabs.addTab(self.headset_screen, "HEADSET")

        self.camera_screen = CameraScreen()
        self.tabs.addTab(self.camera_screen, "CAMERA")

        self.stats_screen = StatsScreen()
        self.tabs.addTab(self.stats_screen, "STATS")

        body.addWidget(self.tabs, 1)

        # E-stop (right side, always visible)
        self.estop = EstopWidget(ros)
        body.addWidget(self.estop)

        main_layout.addLayout(body, 1)

        # Poll timer
        self._poll_timer = QTimer()
        self._poll_timer.setInterval(self.POLL_INTERVAL_MS)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start()

    def _poll(self):
        status = self.ros.get_status()
        self.status_bar.update_status(status)
        self.status_screen.update_status(status)
        self.headset_screen.update_status(status)
        self.camera_screen.update_status(status)
        self.stats_screen.update_status(status)
        self.estop.sync_state(status)

    def keyPressEvent(self, event: QKeyEvent):
        key = event.key()

        if key == Qt.Key_Left:
            idx = (self.tabs.currentIndex() - 1) % self.tabs.count()
            self.tabs.setCurrentIndex(idx)
        elif key == Qt.Key_Right:
            idx = (self.tabs.currentIndex() + 1) % self.tabs.count()
            self.tabs.setCurrentIndex(idx)
        elif key == Qt.Key_F11:
            if self.isFullScreen():
                self.showNormal()
            else:
                self.showFullScreen()
        else:
            super().keyPressEvent(event)


# ── Entry Point ─────────────────────────────────────────────────────

def main():
    import os
    no_ros = "--no-ros" in sys.argv
    fullscreen = "--fullscreen" in sys.argv or "-f" in sys.argv

    # Scale UI so it looks right when streaming a HiDPI laptop to a Steam Deck.
    # The laptop is 3456x2160, the Deck is 1280x800 — without scaling,
    # everything renders tiny. QT_SCALE_FACTOR makes Qt draw larger.
    if fullscreen and "QT_SCALE_FACTOR" not in os.environ:
        os.environ["QT_SCALE_FACTOR"] = "2.5"

    app = QApplication(sys.argv)
    app.setStyleSheet(GLOBAL_STYLE)

    ros = RosInterface()

    if not no_ros:
        if ROS_AVAILABLE:
            ros.start()
        else:
            print("WARNING: rclpy not found. Running in offline mode.")
            print("  Install ROS 2 or run with --no-ros to suppress this warning.")
    else:
        print("Running in offline mode (--no-ros)")

    window = MainWindow(ros)
    if fullscreen:
        window.showFullScreen()
    else:
        window.show()

    exit_code = app.exec_()
    ros.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
