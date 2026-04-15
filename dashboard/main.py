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

JOINT_COLORS = ["#f85149", "#f0883e", "#d29922", "#3fb950", "#58a6ff", "#bc8cff"]


class RollingGraph(QWidget):
    """QPainter-based rolling line graph for real-time metrics on Steam Deck."""

    def __init__(self, title="", window_s=30.0, y_range=(-1.0, 1.0),
                 series=None, auto_y=False, parent=None):
        super().__init__(parent)
        self.title = title
        self.window_s = window_s
        self._y_min, self._y_max = y_range
        self.auto_y = auto_y
        self.series = series or [("value", GREEN)]
        self._data = []
        self.setMinimumHeight(60)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_data(self, data):
        self._data = data
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        ML, MR, MT, MB = 36, 6, 16, 12
        pw, ph = w - ML - MR, h - MT - MB

        if pw <= 0 or ph <= 0:
            p.end()
            return

        p.fillRect(0, 0, w, h, QColor(DARK_BG))
        p.fillRect(ML, MT, pw, ph, QColor(PANEL_BG))

        # Title
        p.setPen(QColor(BLUE))
        p.setFont(QFont("monospace", 7, QFont.Bold))
        p.drawText(ML, MT - 4, self.title)

        # Legend (right side of title bar)
        legend_x = w - MR
        p.setFont(QFont("monospace", 6))
        for name, color in reversed(self.series):
            text = f" {name}"
            tw = p.fontMetrics().horizontalAdvance(text) + 10
            legend_x -= tw
            p.setPen(QColor(color))
            p.drawText(legend_x + 10, MT - 4, text)
            p.fillRect(legend_x + 2, MT - 10, 6, 6, QColor(color))

        if not self._data:
            p.setPen(QColor(TEXT_DIM))
            p.setFont(QFont("monospace", 8))
            p.drawText(ML + pw // 2 - 20, MT + ph // 2, "No data")
            p.end()
            return

        now = self._data[-1][0]
        t0 = now - self.window_s

        # Y range
        y_min, y_max = self._y_min, self._y_max
        if self.auto_y:
            vals = []
            for t, v in self._data:
                if t < t0:
                    continue
                if isinstance(v, (list, tuple)):
                    vals.extend(v)
                else:
                    vals.append(v)
            if vals:
                y_min = min(vals)
                y_max = max(vals)
                pad = (y_max - y_min) * 0.15 or 0.5
                y_min = min(y_min - pad, self._y_min) if not self.auto_y else y_min - pad
                y_max = max(y_max + pad, self._y_max) if not self.auto_y else y_max + pad
        if y_max <= y_min:
            y_max = y_min + 1

        # Grid lines + Y labels
        p.setFont(QFont("monospace", 6))
        for i in range(5):
            gy = MT + int(ph * i / 4)
            p.setPen(QPen(QColor(BORDER), 1))
            p.drawLine(ML, gy, ML + pw, gy)
            val = y_max - (y_max - y_min) * i / 4
            p.setPen(QColor(TEXT_DIM))
            p.drawText(0, gy - 4, ML - 2, 12, Qt.AlignRight | Qt.AlignVCenter,
                        f"{val:.1f}" if abs(val) < 10 else f"{val:.0f}")

        # Zero line
        if y_min < 0 < y_max:
            zy = MT + int(ph * y_max / (y_max - y_min))
            p.setPen(QPen(QColor(TEXT_DIM), 1, Qt.DashLine))
            p.drawLine(ML, zy, ML + pw, zy)

        # Data lines
        n_series = len(self.series)
        for s in range(n_series):
            color = self.series[s][1]
            p.setPen(QPen(QColor(color), 1))
            prev = None
            for t, v in self._data:
                if t < t0:
                    continue
                val = v[s] if isinstance(v, (list, tuple)) else v
                px = ML + int((t - t0) / self.window_s * pw)
                py = MT + int((y_max - val) / (y_max - y_min) * ph)
                py = max(MT, min(MT + ph, py))
                if prev is not None:
                    p.drawLine(prev[0], prev[1], px, py)
                prev = (px, py)

        # Border
        p.setPen(QPen(QColor(BORDER), 1))
        p.drawRect(ML, MT, pw, ph)
        p.end()


class StatsScreen(QWidget):
    """Session metrics + rolling graphs for the STATS tab."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        # ── Session info panel (top strip) ──
        session = QFrame()
        session.setFixedHeight(42)
        session.setStyleSheet(
            f"background: {PANEL_BG}; border: 1px solid {BORDER}; border-radius: 4px;"
        )
        sl = QHBoxLayout(session)
        sl.setContentsMargins(8, 2, 8, 2)
        sl.setSpacing(12)

        self.sess_time = QLabel("00:00")
        self.sess_time.setFont(QFont("monospace", 9, QFont.Bold))
        self.sess_time.setStyleSheet(f"color: {TEXT}; border: none;")
        sl.addWidget(self.sess_time)

        self.sess_mode = QLabel("Mode: ---")
        self.sess_mode.setFont(QFont("monospace", 8))
        self.sess_mode.setStyleSheet(f"color: {TEXT}; border: none;")
        sl.addWidget(self.sess_mode)

        self.sess_switches = QLabel("Switches: 0")
        self.sess_switches.setFont(QFont("monospace", 8))
        self.sess_switches.setStyleSheet(f"color: {TEXT_DIM}; border: none;")
        sl.addWidget(self.sess_switches)

        self.sess_estops = QLabel("E-stops: 0")
        self.sess_estops.setFont(QFont("monospace", 8, QFont.Bold))
        self.sess_estops.setStyleSheet(f"color: {TEXT_DIM}; border: none;")
        sl.addWidget(self.sess_estops)

        sl.addStretch()

        self.sess_breakdown = QLabel("")
        self.sess_breakdown.setFont(QFont("monospace", 7))
        self.sess_breakdown.setStyleSheet(f"color: {TEXT_DIM}; border: none;")
        sl.addWidget(self.sess_breakdown)

        layout.addWidget(session)

        # ── Joint velocity rolling graph ──
        self.vel_graph = RollingGraph(
            title="JOINT VELOCITIES (rad/s)",
            window_s=30.0,
            y_range=(-2.0, 2.0),
            series=list(zip(
                ["pan", "lift", "elbow", "wr1", "wr2", "wr3"],
                JOINT_COLORS,
            )),
            auto_y=True,
        )
        layout.addWidget(self.vel_graph, 2)

        # ── Topic health rolling graph ──
        self.rate_graph = RollingGraph(
            title="TOPIC HEALTH (%)",
            window_s=60.0,
            y_range=(0, 120),
            series=[
                ("joints", GREEN),
                ("vel_cmd", BLUE),
                ("headset", ORANGE),
            ],
        )
        layout.addWidget(self.rate_graph, 1)

    def update_status(self, status):
        # Session info from Unity SessionLogger
        info = status.session_info
        if info:
            secs = info.get("session_s", 0)
            self.sess_time.setText(f"{int(secs) // 60:02d}:{int(secs) % 60:02d}")

            mode = info.get("mode", "---")
            sub = info.get("sub_mode", "")
            mode_text = f"RMRC ({sub})" if mode == "RMRC" and sub else mode
            self.sess_mode.setText(f"Mode: {mode_text}")

            self.sess_switches.setText(f"Switches: {info.get('mode_switches', 0)}")

            durations = info.get("mode_durations", {})
            total = sum(durations.values()) or 1
            parts = []
            for key, short in [("RMRC_Translate", "Trans"), ("RMRC_Rotate", "Rot"),
                               ("DirectJoint", "Joint"), ("HandGuide", "Hand")]:
                pct = durations.get(key, 0) / total * 100
                if pct >= 1:
                    parts.append(f"{short}:{pct:.0f}%")
            self.sess_breakdown.setText("  ".join(parts))

        # E-stop count from dashboard events
        estop_count = sum(1 for _, msg in status.events if "EMERGENCY STOP" in msg)
        self.sess_estops.setText(f"E-stops: {estop_count}")
        color = RED if estop_count > 0 else TEXT_DIM
        self.sess_estops.setStyleSheet(f"color: {color}; border: none;")

        # Feed rolling graphs
        self.vel_graph.set_data(status.velocity_history)
        self.rate_graph.set_data(status.rate_history)


# ── Screen: Latency ────────────────────────────────────────────────

class LatencyScreen(QWidget):
    """Rolling latency graphs — joint state freshness, command timing."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        # ── Live numbers strip ──
        strip = QFrame()
        strip.setFixedHeight(36)
        strip.setStyleSheet(
            f"background: {PANEL_BG}; border: 1px solid {BORDER}; border-radius: 4px;"
        )
        sl = QHBoxLayout(strip)
        sl.setContentsMargins(8, 2, 8, 2)
        sl.setSpacing(16)

        self.joint_age_lbl = QLabel("Joint age: ---")
        self.joint_age_lbl.setFont(QFont("monospace", 8))
        self.joint_age_lbl.setStyleSheet(f"color: {TEXT}; border: none;")
        sl.addWidget(self.joint_age_lbl)

        self.cmd_age_lbl = QLabel("Cmd age: ---")
        self.cmd_age_lbl.setFont(QFont("monospace", 8))
        self.cmd_age_lbl.setStyleSheet(f"color: {TEXT}; border: none;")
        sl.addWidget(self.cmd_age_lbl)

        self.cmd_interval_lbl = QLabel("Cmd interval: ---")
        self.cmd_interval_lbl.setFont(QFont("monospace", 8))
        self.cmd_interval_lbl.setStyleSheet(f"color: {TEXT}; border: none;")
        sl.addWidget(self.cmd_interval_lbl)

        sl.addStretch()
        layout.addWidget(strip)

        # ── Joint state age graph (how fresh is the latest joint data) ──
        self.age_graph = RollingGraph(
            title="MESSAGE AGE (ms) — lower is better",
            window_s=30.0,
            y_range=(0, 100),
            series=[
                ("joint_state", GREEN),
                ("vel_cmd", BLUE),
            ],
            auto_y=True,
        )
        layout.addWidget(self.age_graph, 1)

        # ── Command interval graph (time between velocity commands) ──
        self.interval_graph = RollingGraph(
            title="COMMAND INTERVAL (ms) — expected ~20ms at 50Hz",
            window_s=30.0,
            y_range=(0, 50),
            series=[
                ("interval", ORANGE),
            ],
            auto_y=True,
        )
        layout.addWidget(self.interval_graph, 1)

    def update_status(self, status):
        data = status.latency_history
        if data:
            latest = data[-1][1]
            joint_ms = latest[0]
            vel_ms = latest[1]
            interval_ms = latest[2]

            def fmt(ms):
                if ms < 0:
                    return "---"
                return f"{ms:.0f} ms"

            def age_color(ms):
                if ms < 0:
                    return TEXT_DIM
                if ms < 50:
                    return GREEN
                if ms < 200:
                    return YELLOW
                return RED

            self.joint_age_lbl.setText(f"Joint age: {fmt(joint_ms)}")
            self.joint_age_lbl.setStyleSheet(f"color: {age_color(joint_ms)}; border: none;")

            self.cmd_age_lbl.setText(f"Cmd age: {fmt(vel_ms)}")
            self.cmd_age_lbl.setStyleSheet(f"color: {age_color(vel_ms)}; border: none;")

            self.cmd_interval_lbl.setText(f"Cmd interval: {fmt(interval_ms)}")
            interval_color = GREEN if 0 < interval_ms < 30 else (YELLOW if 0 < interval_ms < 50 else TEXT_DIM)
            self.cmd_interval_lbl.setStyleSheet(f"color: {interval_color}; border: none;")

        # Feed age graph (series 0 = joint age, series 1 = vel cmd age)
        # Filter out negative values for clean display
        age_data = []
        for t, vals in data:
            j = max(vals[0], 0) if vals[0] >= 0 else 0
            v = max(vals[1], 0) if vals[1] >= 0 else 0
            age_data.append((t, [j, v]))
        self.age_graph.set_data(age_data)

        # Feed interval graph (series 0 = cmd interval)
        interval_data = []
        for t, vals in data:
            iv = max(vals[2], 0) if vals[2] >= 0 else 0
            interval_data.append((t, [iv]))
        self.interval_graph.set_data(interval_data)


# ── Screen: Session ────────────────────────────────────────────────

class SessionScreen(QWidget):
    """Text-based session overview — mode, connection, topic rates, durations."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(8)

        # ── Left column: session & control info ──
        left = QVBoxLayout()
        left.setSpacing(2)

        sess_title = QLabel("SESSION")
        sess_title.setFont(QFont("monospace", 8, QFont.Bold))
        sess_title.setStyleSheet(f"color: {BLUE};")
        left.addWidget(sess_title)

        self.duration_lbl = self._metric(left, "Duration: ---")
        self.mode_lbl = self._metric(left, "Control mode: ---")
        self.submode_lbl = self._metric(left, "Sub-mode: ---")
        self.handguide_lbl = self._metric(left, "Hand guide: ---")
        self.switches_lbl = self._metric(left, "Mode switches: ---")
        self.estops_lbl = self._metric(left, "E-stop count: ---")

        left.addSpacing(8)

        dur_title = QLabel("MODE DURATIONS")
        dur_title.setFont(QFont("monospace", 8, QFont.Bold))
        dur_title.setStyleSheet(f"color: {BLUE};")
        left.addWidget(dur_title)

        self.dur_rmrc_t = self._metric(left, "RMRC Translate: ---")
        self.dur_rmrc_r = self._metric(left, "RMRC Rotate:    ---")
        self.dur_joint = self._metric(left, "Direct Joint:   ---")
        self.dur_hand = self._metric(left, "Hand Guide:     ---")

        left.addStretch()
        layout.addLayout(left, 1)

        # Separator
        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"color: {BORDER};")
        layout.addWidget(sep)

        # ── Right column: connection & topic rates ──
        right = QVBoxLayout()
        right.setSpacing(2)

        conn_title = QLabel("CONNECTION")
        conn_title.setFont(QFont("monospace", 8, QFont.Bold))
        conn_title.setStyleSheet(f"color: {BLUE};")
        right.addWidget(conn_title)

        self.ros_lbl = self._metric(right, "ROS: ---")
        self.ctrl_lbl = self._metric(right, "Controller: ---")
        self.state_lbl = self._metric(right, "Robot state: ---")

        right.addSpacing(8)

        rates_title = QLabel("TOPIC RATES")
        rates_title.setFont(QFont("monospace", 8, QFont.Bold))
        rates_title.setStyleSheet(f"color: {BLUE};")
        right.addWidget(rates_title)

        self.rate_labels = {}
        for name in TOPIC_DEFAULTS:
            lbl = self._metric(right, f"{name}: ---")
            self.rate_labels[name] = lbl

        right.addStretch()
        layout.addLayout(right, 1)

    def _metric(self, parent_layout, text):
        lbl = QLabel(text)
        lbl.setFont(QFont("monospace", 8))
        lbl.setStyleSheet(f"color: {TEXT};")
        parent_layout.addWidget(lbl)
        return lbl

    def update_status(self, status):
        info = status.session_info

        # ── Session info ──
        if info:
            secs = info.get("session_s", 0)
            mins, s = int(secs) // 60, int(secs) % 60
            self.duration_lbl.setText(f"Duration: {mins:02d}:{s:02d}")

            mode = info.get("mode", "---")
            self.mode_lbl.setText(f"Control mode: {mode}")
            mode_colors = {"RMRC": GREEN, "DirectJoint": YELLOW, "HandGuide": ORANGE}
            self.mode_lbl.setStyleSheet(f"color: {mode_colors.get(mode, TEXT)};")

            self.submode_lbl.setText(f"Sub-mode: {info.get('sub_mode', '---')}")

            hg = info.get("hand_guide_active", False)
            self.handguide_lbl.setText(f"Hand guide: {'ACTIVE' if hg else 'inactive'}")
            self.handguide_lbl.setStyleSheet(f"color: {ORANGE if hg else TEXT_DIM};")

            self.switches_lbl.setText(f"Mode switches: {info.get('mode_switches', 0)}")

            durations = info.get("mode_durations", {})
            self.dur_rmrc_t.setText(f"RMRC Translate: {self._fmt_dur(durations.get('RMRC_Translate', 0))}")
            self.dur_rmrc_r.setText(f"RMRC Rotate:    {self._fmt_dur(durations.get('RMRC_Rotate', 0))}")
            self.dur_joint.setText(f"Direct Joint:   {self._fmt_dur(durations.get('DirectJoint', 0))}")
            self.dur_hand.setText(f"Hand Guide:     {self._fmt_dur(durations.get('HandGuide', 0))}")
        else:
            self.duration_lbl.setText("Duration: waiting for Unity...")
            self.duration_lbl.setStyleSheet(f"color: {TEXT_DIM};")

        # E-stop count
        estop_count = sum(1 for _, msg in status.events if "EMERGENCY STOP" in msg)
        self.estops_lbl.setText(f"E-stop count: {estop_count}")
        self.estops_lbl.setStyleSheet(f"color: {RED if estop_count > 0 else TEXT};")

        # ── Connection ──
        if status.ros_connected:
            self.ros_lbl.setText("ROS: CONNECTED")
            self.ros_lbl.setStyleSheet(f"color: {GREEN};")
        else:
            self.ros_lbl.setText("ROS: OFFLINE")
            self.ros_lbl.setStyleSheet(f"color: {RED};")

        if status.controller_active:
            self.ctrl_lbl.setText("Controller: ACTIVE")
            self.ctrl_lbl.setStyleSheet(f"color: {GREEN};")
        else:
            self.ctrl_lbl.setText("Controller: INACTIVE")
            self.ctrl_lbl.setStyleSheet(f"color: {YELLOW};")

        state_colors = {
            RobotState.RUNNING: (GREEN, "RUNNING"),
            RobotState.ESTOPPED: (RED, "E-STOPPED"),
            RobotState.RESUMING: (YELLOW, "RESUMING"),
            RobotState.DISCONNECTED: (TEXT_DIM, "DISCONNECTED"),
        }
        color, label = state_colors.get(status.robot_state, (TEXT_DIM, "UNKNOWN"))
        self.state_lbl.setText(f"Robot state: {label}")
        self.state_lbl.setStyleSheet(f"color: {color};")

        # ── Topic rates ──
        for name, lbl in self.rate_labels.items():
            rate = status.topic_rates.get(name)
            hz = rate.hz if rate else 0.0
            if hz > 0:
                lbl.setText(f"{name}: {hz:.0f} Hz")
                lbl.setStyleSheet(f"color: {GREEN};")
            else:
                lbl.setText(f"{name}: ---")
                lbl.setStyleSheet(f"color: {TEXT_DIM};")

    @staticmethod
    def _fmt_dur(secs):
        if secs <= 0:
            return "---"
        mins, s = int(secs) // 60, int(secs) % 60
        if mins > 0:
            return f"{mins}m {s}s"
        return f"{s}s"


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

        self.latency_screen = LatencyScreen()
        self.tabs.addTab(self.latency_screen, "LATENCY")

        self.session_screen = SessionScreen()
        self.tabs.addTab(self.session_screen, "SESSION")

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
        self.latency_screen.update_status(status)
        self.session_screen.update_status(status)
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
