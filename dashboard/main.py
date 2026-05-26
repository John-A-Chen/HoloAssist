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
import signal
import sys
import time
from datetime import datetime

from PyQt5.QtCore import Qt, QTimer, QElapsedTimer
from PyQt5.QtGui import QFont, QKeyEvent, QPainter, QColor, QPen
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTabWidget, QTextEdit, QGridLayout,
    QFrame, QProgressBar, QSizePolicy, QStackedWidget, QComboBox,
)

from ros_interface import RosInterface, RobotState, OperatingMode, ROS_AVAILABLE, TOPIC_DEFAULTS
from net_interface import NetInterface


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

BASE_WINDOW_WIDTH = 1280
BASE_WINDOW_HEIGHT = 800
ESTOP_WIDTH_RATIO = 400 / BASE_WINDOW_WIDTH


def _clamp(value, low, high):
    return max(low, min(high, value))


def _scaled_px(base_px: float, scale: float, min_px: int = 1) -> int:
    return max(min_px, int(round(base_px * scale)))


def _window_scale(window_width: int) -> float:
    # Width-first scaling keeps vertical layouts stable while adapting
    # to wider displays.
    return _clamp(window_width / BASE_WINDOW_WIDTH, 0.85, 2.4)


def build_global_style(scale: float = 1.0) -> str:
    tab_pad_v = _scaled_px(4, scale, 2)
    tab_pad_h = _scaled_px(14, scale, 8)
    tab_font = _scaled_px(8, scale, 7)
    return f"""
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
            padding: {tab_pad_v}px {tab_pad_h}px;
            margin-right: 2px;
            border: 1px solid {BORDER};
            border-bottom: none;
            font-size: {tab_font}px;
            font-weight: bold;
            min-width: 72px;
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

        self.joint_hz_label = QLabel("JNT: --- Hz")
        self.joint_hz_label.setFont(QFont("monospace", 8))
        layout.addWidget(self.joint_hz_label)

        self._add_separator(layout)

        self.grip_label = QLabel("GRIP: ---")
        self.grip_label.setFont(QFont("monospace", 8))
        layout.addWidget(self.grip_label)

        self._add_separator(layout)

        self.camera_label = QLabel("CAM: ---")
        self.camera_label.setFont(QFont("monospace", 8))
        layout.addWidget(self.camera_label)

        self._add_separator(layout)

        self.headset_label = QLabel("HEADSET: ---")
        self.headset_label.setFont(QFont("monospace", 8))
        layout.addWidget(self.headset_label)

    def _add_separator(self, layout):
        sep = QLabel("|")
        sep.setStyleSheet(f"color: {BORDER};")
        sep.setFont(QFont("monospace", 8))
        layout.addWidget(sep)

    def apply_scale(self, scale: float):
        self.setFixedHeight(_scaled_px(22, scale, 18))
        layout = self.layout()
        if layout:
            m = _scaled_px(8, scale, 4)
            layout.setContentsMargins(m, 0, m, 0)

    def update_status(self, status):
        if status.joint_hz > 0:
            self.joint_hz_label.setText(f"JNT: {status.joint_hz:.0f} Hz")
            self.joint_hz_label.setStyleSheet(f"color: {GREEN};")
        else:
            self.joint_hz_label.setText("JNT: --- Hz")
            self.joint_hz_label.setStyleSheet(f"color: {TEXT_DIM};")

        g = status.gripper_value
        if g > 0.9:
            self.grip_label.setText("GRIP: CLOSED")
            self.grip_label.setStyleSheet(f"color: {RED};")
        elif g > 0.05:
            self.grip_label.setText(f"GRIP: {int(g * 100)}%")
            self.grip_label.setStyleSheet(f"color: {YELLOW};")
        else:
            self.grip_label.setText("GRIP: OPEN")
            self.grip_label.setStyleSheet(f"color: {GREEN};")

        cam_type = str(getattr(status, "camera_type", "")).strip().lower()
        cam_rate = status.topic_rates.get("debug_image")
        cam_hz = cam_rate.hz if cam_rate else 0.0
        if cam_type:
            self.camera_label.setText(f"CAM: {cam_type.upper()}")
            self.camera_label.setStyleSheet(f"color: {GREEN};")
        elif cam_hz > 0:
            self.camera_label.setText("CAM: ACTIVE")
            self.camera_label.setStyleSheet(f"color: {YELLOW};")
        else:
            self.camera_label.setText("CAM: ---")
            self.camera_label.setStyleSheet(f"color: {TEXT_DIM};")

        headset_type = str(getattr(status, "headset_type", "")).strip().lower()
        headset_rate = status.topic_rates.get("headset_image")
        headset_hz = headset_rate.hz if headset_rate else 0.0
        if headset_type:
            label = headset_type.upper().replace("QUEST", "QUEST ").strip()
            self.headset_label.setText(f"HEADSET: {label}")
            self.headset_label.setStyleSheet(f"color: {GREEN};")
        elif headset_hz > 0:
            self.headset_label.setText("HEADSET: QUEST")
            self.headset_label.setStyleSheet(f"color: {GREEN};")
        else:
            self.headset_label.setText("HEADSET: ---")
            self.headset_label.setStyleSheet(f"color: {TEXT_DIM};")


# ── E-Stop Widget ──────────────────────────────────────────────────

class EstopWidget(QFrame):
    """
    Large e-stop column on the right side of the window.
    - Normal state: big red EMERGENCY STOP button
    - Estopped state: pulsing red background + yellow HOLD TO RESUME button with progress
    """

    RESUME_HOLD_MS = 3000  # 3 seconds
    COOLDOWN_MS = 400     # 1 second cooldown after resume before estop is clickable

    def __init__(self, ros: RosInterface, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.setFixedWidth(400)
        self._estopped = False
        self._resume_held = False
        self._resume_elapsed = 0
        self._pulse_phase = 0.0
        self._cooldown_active = False
        self._resuming = False  # debounce: ignore ESTOPPED from ROS while controller reactivates

        layout = QVBoxLayout(self)
        layout.setContentsMargins(4, 4, 4, 4)
        self._outer_layout = layout

        # Stacked widget: estop button vs resume button
        self.stack = QStackedWidget()
        layout.addWidget(self.stack)

        # Page 0: E-STOP button
        self.estop_btn = QPushButton("EMERGENCY\nSTOP")
        self.estop_btn.setFont(QFont("monospace", 11, QFont.Bold))
        self.estop_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.estop_btn.setCursor(Qt.PointingHandCursor)
        self.estop_btn.clicked.connect(self._on_estop)
        self.stack.addWidget(self.estop_btn)

        # Page 1: Resume panel (shown during e-stop)
        resume_panel = QWidget()
        resume_layout = QVBoxLayout(resume_panel)
        resume_layout.setContentsMargins(0, 0, 0, 0)
        resume_layout.setSpacing(8)
        self._resume_layout = resume_layout

        self.stopped_label = QLabel("ROBOT\nSTOPPED")
        self.stopped_label.setAlignment(Qt.AlignCenter)
        self.stopped_label.setFont(QFont("monospace", 8, QFont.Bold))
        self.stopped_label.setStyleSheet(f"color: {RED};")
        resume_layout.addWidget(self.stopped_label)

        self.resume_btn = QPushButton("HOLD TO\nRESUME\n(3s)")
        self.resume_btn.setFont(QFont("monospace", 8, QFont.Bold))
        self.resume_btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.resume_btn.setCursor(Qt.PointingHandCursor)
        self.resume_btn.pressed.connect(self._on_resume_press)
        self.resume_btn.released.connect(self._on_resume_release)
        resume_layout.addWidget(self.resume_btn)

        self.progress = QProgressBar()
        self.progress.setFixedHeight(10)
        self.progress.setRange(0, self.RESUME_HOLD_MS)
        self.progress.setValue(0)
        self.progress.setTextVisible(False)
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

        self._resume_debounce_timer = QTimer()
        self._resume_debounce_timer.setSingleShot(True)
        self._resume_debounce_timer.timeout.connect(self._resume_debounce_done)

        self.apply_scale(1.0, BASE_WINDOW_WIDTH)

    def _apply_button_styles(self, scale: float):
        border = _scaled_px(3, scale, 2)
        radius = _scaled_px(12, scale, 8)
        self.estop_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {RED};
                color: white;
                border: {border}px solid #ff6b6b;
                border-radius: {radius}px;
            }}
            QPushButton:hover {{
                background-color: #ff2020;
                border-color: white;
            }}
            QPushButton:pressed {{
                background-color: #cc0000;
            }}
        """)
        self.resume_btn.setStyleSheet(f"""
            QPushButton {{
                background-color: {YELLOW};
                color: black;
                border: {border}px solid {ORANGE};
                border-radius: {radius}px;
            }}
            QPushButton:hover {{
                background-color: {ORANGE};
            }}
        """)
        progress_h = _scaled_px(10, scale, 8)
        progress_radius = _scaled_px(5, scale, 3)
        progress_chunk_radius = _scaled_px(4, scale, 2)
        self.progress.setFixedHeight(progress_h)
        self.progress.setStyleSheet(f"""
            QProgressBar {{
                background-color: {DARK_BG};
                border: 1px solid {BORDER};
                border-radius: {progress_radius}px;
            }}
            QProgressBar::chunk {{
                background-color: {GREEN};
                border-radius: {progress_chunk_radius}px;
            }}
        """)

    def apply_scale(self, scale: float, window_width: int):
        estop_w = int(window_width * ESTOP_WIDTH_RATIO)
        estop_w = _clamp(estop_w, _scaled_px(320, scale, 280), _scaled_px(1200, scale, 900))
        self.setFixedWidth(estop_w)
        m = _scaled_px(4, scale, 2)
        self._outer_layout.setContentsMargins(m, m, m, m)
        self._resume_layout.setSpacing(_scaled_px(8, scale, 4))
        self._apply_button_styles(scale)

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
        self.resume_btn.setText("HOLD TO\nRESUME\n(3s)")

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
            self._resuming = True
            self._resume_debounce_timer.start(5000)  # 5s grace while controller reactivates
            self.ros.resume()
            self._cooldown_active = True
            self.estop_btn.setEnabled(False)
            self._cooldown_timer.start(self.COOLDOWN_MS)
            self.stack.setCurrentIndex(0)
            self.setStyleSheet("")

    def _cooldown_done(self):
        self._cooldown_active = False
        self.estop_btn.setEnabled(True)

    def _resume_debounce_done(self):
        self._resuming = False

    def _pulse_tick(self):
        self._pulse_phase += 0.15
        intensity = int(20 + 15 * (1 + math.sin(self._pulse_phase)))
        self.setStyleSheet(f"background-color: rgb({intensity}, 0, 0);")

    def sync_state(self, status):
        if status.robot_state == RobotState.ESTOPPED and not self._estopped and not self._resuming:
            self._estopped = True
            self.stack.setCurrentIndex(1)
            self._pulse_timer.start()
        elif status.robot_state == RobotState.RUNNING and self._estopped:
            self._estopped = False
            self._resuming = False
            self._resume_debounce_timer.stop()
            self.stack.setCurrentIndex(0)
            self._pulse_timer.stop()
            self.setStyleSheet("")
            self.progress.setValue(0)
        elif status.robot_state == RobotState.RUNNING and self._resuming:
            self._resuming = False
            self._resume_debounce_timer.stop()


# ── Screen: Status ──────────────────────────────────────────────────

class MergedStatusScreen(QWidget):
    """STATUS tab — joint table + session/safety/connection | event log."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(8)

        # ── Left column: joint table + session + safety + connection ──
        left = QVBoxLayout()
        left.setSpacing(4)

        def _heading(text):
            lbl = QLabel(text)
            lbl.setFont(QFont("monospace", 10, QFont.Bold))
            lbl.setStyleSheet(f"color: {BLUE};")
            left.addWidget(lbl)

        def _row(text, color=TEXT):
            lbl = QLabel(text)
            lbl.setFont(QFont("monospace", 9))
            lbl.setStyleSheet(f"color: {color};")
            left.addWidget(lbl)
            return lbl

        # Joint states table
        _heading("JOINT STATES")

        table = QFrame()
        table.setStyleSheet(
            f"QFrame {{ background: {PANEL_BG}; border: 1px solid {BORDER}; border-radius: 3px; }}"
        )
        tl = QGridLayout(table)
        tl.setContentsMargins(6, 4, 6, 4)
        tl.setSpacing(2)
        tl.setColumnStretch(0, 4)
        tl.setColumnStretch(1, 3)
        tl.setColumnStretch(2, 3)

        for col, txt in enumerate(("JOINT", "POS", "VEL")):
            h = QLabel(txt)
            h.setFont(QFont("monospace", 8, QFont.Bold))
            h.setStyleSheet(f"color: {TEXT_DIM}; border: none;")
            if col > 0:
                h.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            tl.addWidget(h, 0, col)

        self.joint_name_lbls = []
        self.joint_pos_lbls = []
        self.joint_vel_lbls = []
        for i in range(6):
            name_l = QLabel(f"joint_{i}")
            pos_l = QLabel("---")
            vel_l = QLabel("---")
            name_l.setFont(QFont("monospace", 9))
            name_l.setStyleSheet(f"color: {TEXT_DIM}; border: none;")
            for cell in (pos_l, vel_l):
                cell.setFont(QFont("monospace", 9, QFont.Bold))
                cell.setStyleSheet(f"color: {TEXT}; border: none;")
                cell.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            tl.addWidget(name_l, i + 1, 0)
            tl.addWidget(pos_l,  i + 1, 1)
            tl.addWidget(vel_l,  i + 1, 2)
            self.joint_name_lbls.append(name_l)
            self.joint_pos_lbls.append(pos_l)
            self.joint_vel_lbls.append(vel_l)

        hsep = QFrame()
        hsep.setFrameShape(QFrame.HLine)
        hsep.setStyleSheet(f"color: {BORDER};")
        tl.addWidget(hsep, 7, 0, 1, 3)

        self.gripper_label = QLabel("Gripper: ---")
        self.gripper_label.setFont(QFont("monospace", 9))
        self.gripper_label.setStyleSheet(f"color: {TEXT}; border: none;")
        tl.addWidget(self.gripper_label, 8, 0, 1, 3)

        left.addWidget(table)
        left.addSpacing(8)

        # Session
        _heading("SESSION")
        self.duration_lbl = _row("Duration: ---")
        self.mode_lbl     = _row("Mode: ---")
        self.submode_lbl  = _row("Sub-mode: ---")
        self.switches_lbl = _row("Switches: ---")
        self.estops_lbl   = _row("E-stops: 0")

        left.addSpacing(4)

        # Safety
        _heading("SAFETY")
        self.collision_status_lbl = _row("Collision: ---")
        self.ee_lock_lbl          = _row("EE lock: off")

        left.addSpacing(4)

        # Connection
        _heading("CONNECTION")
        self.ros_lbl    = _row("ROS: ---")
        self.ctrl_lbl   = _row("Controller: ---")
        self.state_lbl  = _row("Robot state: ---")
        self.moveit_lbl = _row("MoveIt service: ---")

        left.addStretch()
        layout.addLayout(left, 1)

        sep = QFrame()
        sep.setFrameShape(QFrame.VLine)
        sep.setStyleSheet(f"color: {BORDER};")
        layout.addWidget(sep)

        # ── Right column: event log ──
        right = QVBoxLayout()
        right.setSpacing(3)

        log_title = QLabel("EVENT LOG")
        log_title.setFont(QFont("monospace", 10, QFont.Bold))
        log_title.setStyleSheet(f"color: {BLUE};")
        right.addWidget(log_title)

        self.event_log = QTextEdit()
        self.event_log.setReadOnly(True)
        self.event_log.setFont(QFont("monospace", 10))
        right.addWidget(self.event_log, 1)
        layout.addLayout(right, 1)

    def update_status(self, status):
        # Joint table
        for i in range(6):
            if i < len(status.joint_names):
                name = status.joint_names[i].removesuffix("_joint")
                pos_deg = math.degrees(status.joint_positions[i]) if i < len(status.joint_positions) else 0.0
                vel = status.joint_velocities[i] if i < len(status.joint_velocities) else 0.0
                self.joint_name_lbls[i].setText(name)
                self.joint_pos_lbls[i].setText(f"{pos_deg:+7.1f}°")
                abs_vel = abs(vel)
                vel_color = TEXT if abs_vel < 0.05 else (YELLOW if abs_vel < 0.5 else RED)
                self.joint_vel_lbls[i].setText(f"{vel:+.3f}")
                self.joint_vel_lbls[i].setStyleSheet(f"color: {vel_color}; border: none;")
            else:
                self.joint_name_lbls[i].setText(f"joint_{i}")
                self.joint_pos_lbls[i].setText("---")
                self.joint_vel_lbls[i].setText("---")

        g = status.gripper_value
        pct = int(g * 100)
        bar = "|" * int(g * 10) + "." * (10 - int(g * 10))
        grip_color = GREEN if g < 0.05 else (RED if g > 0.9 else TEXT)
        self.gripper_label.setText(f"Gripper [{bar}] {pct}%")
        self.gripper_label.setStyleSheet(f"color: {grip_color}; border: none;")

        # Session
        info = status.session_info
        if info:
            secs = info.get("session_s", 0)
            self.duration_lbl.setText(f"Duration: {int(secs)//60:02d}:{int(secs)%60:02d}")
            mode = info.get("mode", "---")
            sub  = info.get("sub_mode", "")
            mode_text = f"RMRC ({sub})" if mode == "RMRC" and sub else mode
            mode_colors = {"RMRC": GREEN, "DirectJoint": YELLOW, "HandGuide": ORANGE}
            self.mode_lbl.setText(f"Mode: {mode_text}")
            self.mode_lbl.setStyleSheet(f"color: {mode_colors.get(mode, TEXT)};")
            self.submode_lbl.setText(f"Sub-mode: {sub or '---'}")
            self.switches_lbl.setText(f"Switches: {info.get('mode_switches', 0)}")
        else:
            self.duration_lbl.setText("Duration: waiting for Unity...")
            self.duration_lbl.setStyleSheet(f"color: {TEXT_DIM};")

        estop_count = sum(1 for _, msg in status.events if "EMERGENCY STOP" in msg)
        self.estops_lbl.setText(f"E-stops: {estop_count}")
        self.estops_lbl.setStyleSheet(f"color: {RED if estop_count > 0 else TEXT};")

        # Safety
        if status.collision_blocked:
            self.collision_status_lbl.setText("Collision: BLOCKED")
            self.collision_status_lbl.setStyleSheet(f"color: {RED};")
        elif status.collision_scale < 1.0:
            self.collision_status_lbl.setText(f"Collision: slow ({status.collision_scale:.0%})")
            self.collision_status_lbl.setStyleSheet(f"color: {YELLOW};")
        else:
            self.collision_status_lbl.setText("Collision: clear")
            self.collision_status_lbl.setStyleSheet(f"color: {GREEN};")

        ee_text = "EE lock: ON" if status.ee_locked else "EE lock: off"
        self.ee_lock_lbl.setText(f"{ee_text} ({status.ee_lock_count})")
        self.ee_lock_lbl.setStyleSheet(f"color: {BLUE if status.ee_locked else TEXT_DIM};")

        # Connection
        self.ros_lbl.setText("ROS: CONNECTED" if status.ros_connected else "ROS: OFFLINE")
        self.ros_lbl.setStyleSheet(f"color: {GREEN if status.ros_connected else RED};")

        self.ctrl_lbl.setText("Controller: ACTIVE" if status.controller_active else "Controller: INACTIVE")
        self.ctrl_lbl.setStyleSheet(f"color: {GREEN if status.controller_active else YELLOW};")

        state_map = {
            RobotState.RUNNING:      (GREEN,    "RUNNING"),
            RobotState.ESTOPPED:     (RED,      "E-STOPPED"),
            RobotState.RESUMING:     (YELLOW,   "RESUMING"),
            RobotState.DISCONNECTED: (TEXT_DIM, "DISCONNECTED"),
        }
        c, lbl_txt = state_map.get(status.robot_state, (TEXT_DIM, "UNKNOWN"))
        self.state_lbl.setText(f"Robot state: {lbl_txt}")
        self.state_lbl.setStyleSheet(f"color: {c};")

        moveit_ready = getattr(status, "pick_service_ready", False)
        self.moveit_lbl.setText("MoveIt service: READY" if moveit_ready else "MoveIt service: ---")
        self.moveit_lbl.setStyleSheet(f"color: {GREEN if moveit_ready else TEXT_DIM};")

        # Event log
        lines = [
            f"[{datetime.fromtimestamp(ts).strftime('%H:%M:%S')}] {msg}"
            for ts, msg in status.events
        ]
        text = "\n".join(lines)
        if text != self.event_log.toPlainText():
            self.event_log.setPlainText(text)
            self.event_log.verticalScrollBar().setValue(
                self.event_log.verticalScrollBar().maximum()
            )


# ── Screen: Camera (debug image from depth tracker) ────────────────

class CameraScreen(QWidget):
    _PRESETS = [
        ("640×480 @ 15 Hz",    640,  480,  15.0),
        ("640×480 @ 30 Hz",    640,  480,  30.0),
        ("848×480 @ 30 Hz",    848,  480,  30.0),   # RealSense wide-FOV depth
        ("1280×720 @ 15 Hz",  1280,  720,  15.0),
        ("1280×720 @ 30 Hz",  1280,  720,  30.0),
        ("1280×800 @ 30 Hz",  1280,  800,  30.0),   # RealSense D415 native
        ("1920×1080 @ 15 Hz", 1920, 1080,  15.0),
        ("1920×1080 @ 30 Hz", 1920, 1080,  30.0),
    ]

    def __init__(self, ros=None, parent=None):
        super().__init__(parent)
        self._ros = ros
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(4)

        header = QHBoxLayout()
        self.camera_info = QLabel("Waiting for image...")
        self.camera_info.setFont(QFont("monospace", 8))
        self.camera_info.setStyleSheet(f"color: {TEXT_DIM};")
        header.addWidget(self.camera_info)
        header.addStretch()
        self.res_combo = QComboBox()
        self.res_combo.setFont(QFont("monospace", 8))
        self.res_combo.setStyleSheet(f"""
            QComboBox {{
                background: {PANEL_BG};
                color: {TEXT};
                border: 1px solid {BORDER};
                border-radius: 3px;
                padding: 1px 6px;
            }}
            QComboBox::drop-down {{ border: none; }}
            QComboBox QAbstractItemView {{
                background: {PANEL_BG};
                color: {TEXT};
                selection-background-color: {BLUE};
            }}
        """)
        for label, *_ in self._PRESETS:
            self.res_combo.addItem(label)
        self.res_combo.activated.connect(self._on_reconfigure)
        header.addWidget(self.res_combo)
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

    def _on_reconfigure(self, index: int):
        if self._ros is None or index < 0 or index >= len(self._PRESETS):
            return
        _, w, h, fps = self._PRESETS[index]
        self._ros.reconfigure_camera(w, h, fps)

    def update_status(self, status):
        rate = status.topic_rates.get("debug_image")
        hz = rate.hz if rate else 0
        if status.camera_jpeg and status.camera_width > 0:
            self.camera_info.setText(
                f"{status.camera_width}x{status.camera_height} @ {hz:.0f} Hz"
            )
            try:
                from PyQt5.QtGui import QImage, QPixmap
                encoding = str(getattr(status, "_camera_encoding", "")).strip().lower()
                w, h = status.camera_width, status.camera_height
                data = status.camera_jpeg
                pixels = max(1, w * h)
                channels = len(data) // pixels
                img = None

                if encoding in ("bgr8", "8uc3") or (not encoding and channels == 3):
                    img = QImage(data, w, h, w * 3, QImage.Format_RGB888).rgbSwapped()
                elif encoding == "rgb8":
                    img = QImage(data, w, h, w * 3, QImage.Format_RGB888)
                elif encoding in ("bgra8", "8uc4") or (not encoding and channels == 4):
                    img = QImage(data, w, h, w * 4, QImage.Format_RGBA8888).rgbSwapped()
                elif encoding == "rgba8":
                    img = QImage(data, w, h, w * 4, QImage.Format_RGBA8888)
                elif encoding in ("mono8", "8uc1") or (not encoding and channels == 1):
                    img = QImage(data, w, h, w, QImage.Format_Grayscale8)
                else:
                    # Heuristic fallback for transport/type wrappers that omit canonical names.
                    if channels == 3:
                        img = QImage(data, w, h, w * 3, QImage.Format_RGB888).rgbSwapped()
                    elif channels == 4:
                        img = QImage(data, w, h, w * 4, QImage.Format_RGBA8888).rgbSwapped()
                    elif channels == 1:
                        img = QImage(data, w, h, w, QImage.Format_Grayscale8)
                    else:
                        self.camera_info.setText(f"Unsupported encoding: {encoding or f'{channels}ch'}")
                        return
                if img is None:
                    return
                pixmap = QPixmap.fromImage(img)
                scaled = pixmap.scaled(
                    self.image_label.size(), Qt.KeepAspectRatio, Qt.FastTransformation
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
        self._ui_scale = 1.0
        self.setMinimumHeight(60)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def apply_scale(self, scale: float):
        self._ui_scale = scale
        self.setMinimumHeight(_scaled_px(60, scale, 48))
        self.update()

    def set_data(self, data):
        self._data = data
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()

        scale = self._ui_scale
        ML = _scaled_px(36, scale, 22)
        MR = _scaled_px(6, scale, 4)
        MT = _scaled_px(16, scale, 10)
        MB = _scaled_px(12, scale, 8)
        pw, ph = w - ML - MR, h - MT - MB

        if pw <= 0 or ph <= 0:
            p.end()
            return

        p.fillRect(0, 0, w, h, QColor(DARK_BG))
        p.fillRect(ML, MT, pw, ph, QColor(PANEL_BG))

        # Title
        p.setPen(QColor(BLUE))
        p.setFont(QFont("monospace", _scaled_px(8, scale, 7), QFont.Bold))
        p.drawText(ML, MT - _scaled_px(4, scale, 2), self.title)

        # Legend (right side of title bar)
        legend_x = w - MR
        p.setFont(QFont("monospace", _scaled_px(6, scale, 5)))
        legend_swatch = _scaled_px(6, scale, 4)
        legend_gap = _scaled_px(10, scale, 6)
        for name, color in reversed(self.series):
            text = f" {name}"
            tw = p.fontMetrics().horizontalAdvance(text) + legend_gap
            legend_x -= tw
            p.setPen(QColor(color))
            p.drawText(legend_x + legend_gap, MT - _scaled_px(4, scale, 2), text)
            p.fillRect(
                legend_x + _scaled_px(2, scale, 1),
                MT - _scaled_px(10, scale, 6),
                legend_swatch,
                legend_swatch,
                QColor(color),
            )

        if not self._data:
            p.setPen(QColor(TEXT_DIM))
            p.setFont(QFont("monospace", _scaled_px(8, scale, 7)))
            p.drawText(ML + pw // 2 - _scaled_px(20, scale, 12), MT + ph // 2, "No data")
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
        p.setFont(QFont("monospace", _scaled_px(6, scale, 5)))
        y_label_h = _scaled_px(12, scale, 8)
        y_label_y_offset = _scaled_px(4, scale, 2)
        for i in range(5):
            gy = MT + int(ph * i / 4)
            p.setPen(QPen(QColor(BORDER), 1))
            p.drawLine(ML, gy, ML + pw, gy)
            val = y_max - (y_max - y_min) * i / 4
            p.setPen(QColor(TEXT_DIM))
            p.drawText(
                0,
                gy - y_label_y_offset,
                ML - _scaled_px(2, scale, 1),
                y_label_h,
                Qt.AlignRight | Qt.AlignVCenter,
                f"{val:.1f}" if abs(val) < 10 else f"{val:.0f}",
            )

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


class DebugScreen(QWidget):
    """Merged STATS + LATENCY → DEBUG tab — session strip, latency strip, 2×2 graph grid."""

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        layout.setSpacing(3)

        # ── Session info strip ──
        session = QFrame()
        session.setFixedHeight(28)
        session.setStyleSheet(
            f"background: {PANEL_BG}; border: 1px solid {BORDER}; border-radius: 4px;"
        )
        self.session_panel = session
        sl = QHBoxLayout(session)
        sl.setContentsMargins(8, 2, 8, 2)
        sl.setSpacing(10)
        self.session_layout = sl

        self.sess_time = QLabel("00:00")
        self.sess_time.setFont(QFont("monospace", 9, QFont.Bold))
        self.sess_time.setStyleSheet(f"color: {TEXT}; border: none;")
        sl.addWidget(self.sess_time)

        self.sess_mode = QLabel("Mode: ---")
        self.sess_mode.setFont(QFont("monospace", 9))
        self.sess_mode.setStyleSheet(f"color: {TEXT}; border: none;")
        sl.addWidget(self.sess_mode)

        sl.addStretch()

        self.sess_breakdown = QLabel("")
        self.sess_breakdown.setFont(QFont("monospace", 8))
        self.sess_breakdown.setStyleSheet(f"color: {TEXT_DIM}; border: none;")
        sl.addWidget(self.sess_breakdown)

        layout.addWidget(session)

        # ── Latency live strip ──
        latency = QFrame()
        latency.setFixedHeight(24)
        latency.setStyleSheet(
            f"background: {PANEL_BG}; border: 1px solid {BORDER}; border-radius: 4px;"
        )
        self.latency_panel = latency
        ll = QHBoxLayout(latency)
        ll.setContentsMargins(8, 2, 8, 2)
        ll.setSpacing(16)
        self.latency_layout = ll

        self.joint_age_lbl = QLabel("Jt age: ---")
        self.joint_age_lbl.setFont(QFont("monospace", 8))
        self.joint_age_lbl.setStyleSheet(f"color: {TEXT}; border: none;")
        ll.addWidget(self.joint_age_lbl)

        self.cmd_age_lbl = QLabel("Cmd age: ---")
        self.cmd_age_lbl.setFont(QFont("monospace", 8))
        self.cmd_age_lbl.setStyleSheet(f"color: {TEXT}; border: none;")
        ll.addWidget(self.cmd_age_lbl)

        self.cmd_interval_lbl = QLabel("Interval: ---")
        self.cmd_interval_lbl.setFont(QFont("monospace", 8))
        self.cmd_interval_lbl.setStyleSheet(f"color: {TEXT}; border: none;")
        ll.addWidget(self.cmd_interval_lbl)

        ll.addStretch()
        layout.addWidget(latency)

        # ── 2×2 graph grid ──
        grid = QHBoxLayout()
        grid.setSpacing(4)

        left_col = QVBoxLayout()
        left_col.setSpacing(3)
        right_col = QVBoxLayout()
        right_col.setSpacing(3)

        self.vel_graph = RollingGraph(
            title="JOINT VEL (rad/s)",
            window_s=30.0,
            y_range=(-2.0, 2.0),
            series=list(zip(["pan", "lift", "elbow", "wr1", "wr2", "wr3"], JOINT_COLORS)),
            auto_y=True,
        )
        self.vel_graph.setMinimumHeight(85)
        left_col.addWidget(self.vel_graph, 1)

        self.rate_graph = RollingGraph(
            title="TOPIC HEALTH (%)",
            window_s=60.0,
            y_range=(0, 120),
            series=[("joints", GREEN), ("vel_cmd", BLUE), ("headset", ORANGE)],
        )
        self.rate_graph.setMinimumHeight(65)
        left_col.addWidget(self.rate_graph, 1)

        self.age_graph = RollingGraph(
            title="MSG AGE (ms)",
            window_s=30.0,
            y_range=(0, 100),
            series=[("jt_state", GREEN), ("vel_cmd", BLUE)],
            auto_y=True,
        )
        self.age_graph.setMinimumHeight(85)
        right_col.addWidget(self.age_graph, 1)

        self.interval_graph = RollingGraph(
            title="CMD INTERVAL (ms)",
            window_s=30.0,
            y_range=(0, 50),
            series=[("interval", ORANGE)],
            auto_y=True,
        )
        self.interval_graph.setMinimumHeight(65)
        right_col.addWidget(self.interval_graph, 1)

        grid.addLayout(left_col, 1)
        grid.addLayout(right_col, 1)
        layout.addLayout(grid, 1)

    def apply_scale(self, scale: float):
        self.session_panel.setFixedHeight(_scaled_px(28, scale, 22))
        self.session_layout.setContentsMargins(
            _scaled_px(8, scale, 4), _scaled_px(2, scale, 1),
            _scaled_px(8, scale, 4), _scaled_px(2, scale, 1),
        )
        self.session_layout.setSpacing(_scaled_px(10, scale, 6))
        self.latency_panel.setFixedHeight(_scaled_px(24, scale, 20))
        self.latency_layout.setContentsMargins(
            _scaled_px(8, scale, 4), _scaled_px(2, scale, 1),
            _scaled_px(8, scale, 4), _scaled_px(2, scale, 1),
        )
        self.latency_layout.setSpacing(_scaled_px(16, scale, 8))
        for g in (self.vel_graph, self.rate_graph, self.age_graph, self.interval_graph):
            g.apply_scale(scale)

    def update_status(self, status):
        # Session strip
        info = status.session_info
        if info:
            secs = info.get("session_s", 0)
            self.sess_time.setText(f"{int(secs)//60:02d}:{int(secs)%60:02d}")
            mode = info.get("mode", "---")
            sub  = info.get("sub_mode", "")
            mode_text = f"RMRC ({sub})" if mode == "RMRC" and sub else mode
            self.sess_mode.setText(f"Mode: {mode_text}")
            durations = info.get("mode_durations", {})
            total = sum(durations.values()) or 1
            parts = []
            for key, short in [("RMRC_Translate", "Trans"), ("RMRC_Rotate", "Rot"),
                               ("DirectJoint", "Joint"), ("HandGuide", "Hand")]:
                pct = durations.get(key, 0) / total * 100
                if pct >= 1:
                    parts.append(f"{short}:{pct:.0f}%")
            self.sess_breakdown.setText("  ".join(parts))

        # Latency strip
        data = status.latency_history
        if data:
            latest = data[-1][1]
            joint_ms, vel_ms, interval_ms = latest[0], latest[1], latest[2]

            def fmt(ms):
                return "---" if ms < 0 else f"{ms:.0f}ms"

            def age_color(ms):
                if ms < 0: return TEXT_DIM
                if ms < 50: return GREEN
                if ms < 200: return YELLOW
                return RED

            self.joint_age_lbl.setText(f"Jt age: {fmt(joint_ms)}")
            self.joint_age_lbl.setStyleSheet(f"color: {age_color(joint_ms)}; border: none;")

            self.cmd_age_lbl.setText(f"Cmd age: {fmt(vel_ms)}")
            self.cmd_age_lbl.setStyleSheet(f"color: {age_color(vel_ms)}; border: none;")

            self.cmd_interval_lbl.setText(f"Interval: {fmt(interval_ms)}")
            iv_color = GREEN if 0 < interval_ms < 30 else (YELLOW if 0 < interval_ms < 50 else TEXT_DIM)
            self.cmd_interval_lbl.setStyleSheet(f"color: {iv_color}; border: none;")

            # Dynamic graph titles with live values
            self.age_graph.title = f"MSG AGE (ms)   jt={fmt(joint_ms)} cmd={fmt(vel_ms)}"
            self.interval_graph.title = f"CMD INTERVAL (ms)   now={fmt(interval_ms)}"

        # Vel graph title with recent max
        vel_data = status.velocity_history
        if vel_data:
            recent_vels = [abs(v) for _, vs in vel_data[-10:] for v in vs]
            max_vel = max(recent_vels) if recent_vels else 0.0
            self.vel_graph.title = f"JOINT VEL (rad/s)   max={max_vel:.2f}"

        # Rate graph title with latest values
        rate_data = status.rate_history
        if rate_data:
            lr = rate_data[-1][1]
            self.rate_graph.title = f"TOPIC HEALTH (%)   jt={lr[0]:.0f}% vel={lr[1]:.0f}%"

        # Feed graphs
        self.vel_graph.set_data(vel_data if vel_data else [])
        self.rate_graph.set_data(rate_data if rate_data else [])

        age_data = [(t, [max(v[0], 0), max(v[1], 0)]) for t, v in data]
        self.age_graph.set_data(age_data)

        interval_data = [(t, [max(v[2], 0)]) for t, v in data]
        self.interval_graph.set_data(interval_data)


# ── Screen: MoveIt Cube Pick/Place ─────────────────────────────────

class CubePickScreen(QWidget):
    """MoveIt pick/place controls for AprilTag-tracked cubes."""

    def __init__(self, ros: RosInterface, parent=None):
        super().__init__(parent)
        self.ros = ros
        self.selected_bin_id = 1
        self._last_status_text = ""
        self._style_scale = 1.0
        self._cube_buttons_enabled = False

        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 6, 8, 8)
        layout.setSpacing(8)

        header = QHBoxLayout()
        title = QLabel("CUBE PICK & PLACE")
        title.setFont(QFont("monospace", 11, QFont.Bold))
        title.setStyleSheet(f"color: {BLUE};")
        header.addWidget(title)

        header.addStretch()

        self.status_label = QLabel("SERVICE: ---")
        self.status_label.setFont(QFont("monospace", 10, QFont.Bold))
        self.status_label.setStyleSheet(f"color: {TEXT_DIM};")
        header.addWidget(self.status_label)
        layout.addLayout(header)

        body = QHBoxLayout()
        body.setSpacing(12)
        layout.addLayout(body, 1)

        controls = QVBoxLayout()
        controls.setSpacing(10)
        body.addLayout(controls, 3)

        bin_row = QHBoxLayout()
        bin_row.setSpacing(8)
        bin_label = QLabel("DESTINATION")
        bin_label.setFont(QFont("monospace", 9, QFont.Bold))
        bin_label.setStyleSheet(f"color: {TEXT_DIM};")
        bin_row.addWidget(bin_label)

        self.bin_buttons = []
        for bin_id in (1, 2):
            btn = QPushButton(f"BIN {bin_id}")
            btn.setFont(QFont("monospace", 10, QFont.Bold))
            btn.setCheckable(True)
            btn.setFixedHeight(52)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked=False, b=bin_id: self._select_bin(b))
            bin_row.addWidget(btn, 1)
            self.bin_buttons.append(btn)
        controls.addLayout(bin_row)
        self._update_bin_buttons()

        grid = QGridLayout()
        grid.setSpacing(8)

        self.cube_buttons = []
        for cube_id in range(1, 5):
            btn = QPushButton(f"APRIL CUBE\n{cube_id}")
            btn.setFont(QFont("monospace", 12, QFont.Bold))
            btn.setMinimumHeight(120)
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            btn.setCursor(Qt.PointingHandCursor)
            btn.clicked.connect(lambda checked=False, c=cube_id: self._request_pick(c))
            grid.addWidget(btn, (cube_id - 1) // 2, (cube_id - 1) % 2)
            self.cube_buttons.append(btn)

        controls.addLayout(grid, 1)

        self.service_label = QLabel("Service: /holoassist/pick_cube_to_bin")
        self.service_label.setFont(QFont("monospace", 8))
        self.service_label.setStyleSheet(f"color: {TEXT_DIM};")
        self.service_label.setWordWrap(True)
        controls.addWidget(self.service_label)

        status_panel = QVBoxLayout()
        status_panel.setSpacing(6)
        body.addLayout(status_panel, 2)

        status_title = QLabel("PICK/PLACE MONITOR")
        status_title.setFont(QFont("monospace", 8, QFont.Bold))
        status_title.setStyleSheet(f"color: {BLUE};")
        status_panel.addWidget(status_title)

        self.monitor_block = QLabel("Block: ---")
        self.monitor_block.setFont(QFont("monospace", 8, QFont.Bold))
        self.monitor_block.setStyleSheet(f"color: {TEXT};")
        status_panel.addWidget(self.monitor_block)

        self.monitor_step = QLabel("Step: ---")
        self.monitor_step.setFont(QFont("monospace", 8, QFont.Bold))
        self.monitor_step.setStyleSheet(f"color: {TEXT};")
        self.monitor_step.setWordWrap(True)
        status_panel.addWidget(self.monitor_step)

        self.monitor_state = QLabel("State: waiting")
        self.monitor_state.setFont(QFont("monospace", 8, QFont.Bold))
        self.monitor_state.setStyleSheet(f"color: {TEXT_DIM};")
        status_panel.addWidget(self.monitor_state)

        self.pick_place_status = QTextEdit()
        self.pick_place_status.setReadOnly(True)
        self.pick_place_status.setMinimumWidth(360)
        self.pick_place_status.setFont(QFont("monospace", 10))
        self.pick_place_status.setStyleSheet(f"""
            QTextEdit {{
                background-color: {DARK_BG};
                color: {TEXT};
                border: 1px solid {BORDER};
            }}
        """)
        status_panel.addWidget(self.pick_place_status, 1)

        self._set_buttons_enabled(False)
        self.apply_scale(1.0)

    def _select_bin(self, bin_id: int):
        self.selected_bin_id = bin_id
        self._update_bin_buttons()

    def _update_bin_buttons(self):
        border = _scaled_px(2, self._style_scale, 1)
        radius = _scaled_px(8, self._style_scale, 5)
        for idx, btn in enumerate(self.bin_buttons, start=1):
            active = idx == self.selected_bin_id
            btn.setChecked(active)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {GREEN if active else DARK_BG};
                    color: {'white' if active else TEXT};
                    border: {border}px solid {('#6fdd8b' if active else BORDER)};
                    border-radius: {radius}px;
                }}
                QPushButton:hover {{
                    border-color: {GREEN};
                }}
            """)

    def _request_pick(self, cube_id: int):
        self.status_label.setText(
            f"REQUESTING: APRIL CUBE {cube_id} -> BIN {self.selected_bin_id}"
        )
        self.status_label.setStyleSheet(f"color: {YELLOW};")
        self.ros.pick_cube_to_bin(cube_id, self.selected_bin_id)

    def update_status(self, status):
        moveit = status.operating_mode == "MOVEIT"
        estopped = status.robot_state == RobotState.ESTOPPED
        ready = (
            moveit
            and status.ros_connected
            and status.pick_service_ready
            and not status.pick_request_pending
            and not estopped
        )
        self._set_buttons_enabled(ready)

        if status.pick_request_pending:
            cube = status.last_pick_cube.replace("april_cube_", "APRIL CUBE ")
            self.status_label.setText(f"PENDING: {cube}")
            self.status_label.setStyleSheet(f"color: {YELLOW};")
        elif status.last_pick_message:
            cube = status.last_pick_cube.replace("april_cube_", "APRIL CUBE ")
            prefix = "READY"
            color = GREEN
            if status.last_pick_success is False:
                prefix = "FAILED"
                color = RED
            elif status.last_pick_success is True:
                prefix = "QUEUED"
                color = GREEN
            self.status_label.setText(f"{prefix}: {cube}")
            self.status_label.setStyleSheet(f"color: {color};")
            self.service_label.setText(status.last_pick_message)
        elif not status.ros_connected:
            self.status_label.setText("ROS: OFFLINE")
            self.status_label.setStyleSheet(f"color: {RED};")
        elif estopped:
            self.status_label.setText("ROBOT: E-STOPPED")
            self.status_label.setStyleSheet(f"color: {RED};")
        elif not status.pick_service_ready:
            self.status_label.setText("SERVICE: WAITING")
            self.status_label.setStyleSheet(f"color: {YELLOW};")
        else:
            self.status_label.setText("SERVICE: READY")
            self.status_label.setStyleSheet(f"color: {GREEN};")

        block = status.pick_place_block_id or "---"
        destination = status.pick_place_destination or "---"
        self.monitor_block.setText(f"Block: {block} -> {destination}")

        step_label = (
            status.pick_place_step_label
            or status.pick_place_step
            or status.pick_place_status
            or "---"
        )
        if status.pick_place_step_total > 0:
            self.monitor_step.setText(
                f"Step {status.pick_place_step_index}/{status.pick_place_step_total}: {step_label}"
            )
        else:
            self.monitor_step.setText(f"Step: {step_label}")

        state = status.pick_place_state or "waiting"
        state_color = TEXT_DIM
        if state.lower() in {"running", "starting"}:
            state_color = YELLOW
        elif state.lower() == "complete":
            state_color = GREEN
        elif state.lower() == "error":
            state_color = RED
        self.monitor_state.setText(f"State: {state.upper()}")
        self.monitor_state.setStyleSheet(f"color: {state_color};")

        if status.pick_place_error:
            status_text = "ERROR:\n" + status.pick_place_error
            if status.pick_place_error_detail:
                status_text += "\n\nDETAIL:\n" + status.pick_place_error_detail
        elif status.pick_place_status_lines:
            status_text = "\n".join(status.pick_place_status_lines[-18:])
        else:
            status_text = "Waiting for pick/place status..."
        if status_text != self._last_status_text:
            self._last_status_text = status_text
            self.pick_place_status.setPlainText(status_text)
            scroll = self.pick_place_status.verticalScrollBar()
            scroll.setValue(scroll.maximum())

    def _set_buttons_enabled(self, enabled: bool):
        self._cube_buttons_enabled = enabled
        border = _scaled_px(3, self._style_scale, 2)
        radius = _scaled_px(8, self._style_scale, 5)
        for btn in self.cube_buttons:
            btn.setEnabled(enabled)
            btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {BLUE if enabled else DARK_BG};
                    color: {'white' if enabled else TEXT_DIM};
                    border: {border}px solid {('#79bbff' if enabled else BORDER)};
                    border-radius: {radius}px;
                }}
                QPushButton:hover {{
                    background-color: #79bbff;
                    color: white;
                }}
                QPushButton:pressed {{
                    background-color: #2f81f7;
                }}
            """)

    def apply_scale(self, scale: float):
        self._style_scale = scale
        for btn in self.bin_buttons:
            btn.setFixedHeight(_scaled_px(52, scale, 38))
        for btn in self.cube_buttons:
            btn.setMinimumHeight(_scaled_px(120, scale, 84))
        self.pick_place_status.setMinimumWidth(_scaled_px(360, scale, 260))
        self._update_bin_buttons()
        self._set_buttons_enabled(self._cube_buttons_enabled)


# ── Screen: Hand-Eye Calibration ───────────────────────────────────

class CalibrationScreen(QWidget):
    """Controls automatic physical hand-eye sampling through MoveIt."""

    def __init__(self, ros: RosInterface, parent=None):
        super().__init__(parent)
        self.ros = ros
        self._last_detail = ""

        layout = QVBoxLayout(self)
        layout.setContentsMargins(10, 8, 10, 10)
        layout.setSpacing(10)

        header = QHBoxLayout()
        title = QLabel("HAND-EYE CALIBRATION")
        title.setFont(QFont("monospace", 11, QFont.Bold))
        title.setStyleSheet(f"color: {BLUE};")
        header.addWidget(title)
        header.addStretch()
        self.ready_label = QLabel("SERVER: WAITING")
        self.ready_label.setFont(QFont("monospace", 10, QFont.Bold))
        header.addWidget(self.ready_label)
        layout.addLayout(header)

        self.tag_label = QLabel("Physical marker: tag36h11:1")
        self.tag_label.setFont(QFont("monospace", 9))
        self.tag_label.setStyleSheet(f"color: {TEXT_DIM};")
        layout.addWidget(self.tag_label)

        self.state_label = QLabel("Waiting for calibration coordinator...")
        self.state_label.setFont(QFont("monospace", 11, QFont.Bold))
        self.state_label.setWordWrap(True)
        layout.addWidget(self.state_label)

        self.progress = QProgressBar()
        self.progress.setRange(0, 1)
        self.progress.setFormat("POSE %v / %m")
        layout.addWidget(self.progress)

        self.sample_label = QLabel("Samples: 0")
        self.sample_label.setFont(QFont("monospace", 10, QFont.Bold))
        layout.addWidget(self.sample_label)

        buttons = QHBoxLayout()
        self.start_btn = self._button("START AUTO", "start")
        self.stop_btn = self._button("STOP", "stop")
        self.sample_btn = self._button("TAKE SAMPLE", "sample")
        self.compute_btn = self._button("COMPUTE", "compute")
        self.save_btn = self._button("SAVE", "save")
        for button in (
            self.start_btn,
            self.stop_btn,
            self.sample_btn,
            self.compute_btn,
            self.save_btn,
        ):
            buttons.addWidget(button, 1)
        layout.addLayout(buttons)

        warning = QLabel(
            "START AUTO commands physical MoveIt motion. Check tool tag visibility, "
            "pose clearance and workcell safety before starting."
        )
        warning.setFont(QFont("monospace", 8, QFont.Bold))
        warning.setStyleSheet(f"color: {ORANGE};")
        warning.setWordWrap(True)
        layout.addWidget(warning)

        self.details = QTextEdit()
        self.details.setReadOnly(True)
        self.details.setFont(QFont("monospace", 9))
        layout.addWidget(self.details, 1)
        self.apply_scale(1.0)

    def _button(self, text, action):
        button = QPushButton(text)
        button.setFont(QFont("monospace", 10, QFont.Bold))
        button.setCursor(Qt.PointingHandCursor)
        button.clicked.connect(
            lambda checked=False, command=action: self.ros.calibration_command(command)
        )
        return button

    def update_status(self, status):
        ready = status.calibration_ready
        running = status.calibration_running
        state = status.calibration_state.upper() if status.calibration_state else "WAITING"
        color = BLUE if running else YELLOW
        if state in {"SAVED", "COMPUTED"}:
            color = GREEN
        elif state == "ERROR":
            color = RED
        self.ready_label.setText("SERVER: READY" if ready else "SERVER: WAITING")
        self.ready_label.setStyleSheet(f"color: {GREEN if ready else YELLOW};")
        self.state_label.setText(f"{state}: {status.calibration_message or 'Waiting for status'}")
        self.state_label.setStyleSheet(f"color: {color};")
        self.tag_label.setText(f"Physical marker: {status.calibration_marker_frame}")
        self.progress.setMaximum(max(status.calibration_pose_total, 1))
        self.progress.setValue(status.calibration_pose_index)
        self.sample_label.setText(
            f"Samples: {status.calibration_sample_count}  |  "
            f"Result: {'computed' if status.calibration_computed else 'not computed'}"
        )
        self.start_btn.setEnabled(ready and not running)
        self.stop_btn.setEnabled(running)
        self.sample_btn.setEnabled(ready and not running)
        self.compute_btn.setEnabled(ready and not running and status.calibration_sample_count >= 8)
        self.save_btn.setEnabled(ready and not running and status.calibration_computed)

        detail = [
            f"State: {state}",
            f"Marker frame: {status.calibration_marker_frame}",
            f"Samples: {status.calibration_sample_count}",
            "Minimum compute samples: 8 varied poses",
        ]
        if status.calibration_latest_path:
            detail.append(f"Newest default: {status.calibration_latest_path}")
        if status.calibration_archive_path:
            detail.append(f"History copy: {status.calibration_archive_path}")
        if status.calibration_error:
            detail.append(f"Error: {status.calibration_error}")
        text = "\n".join(detail)
        if text != self._last_detail:
            self._last_detail = text
            self.details.setPlainText(text)

    def apply_scale(self, scale: float):
        self.progress.setFixedHeight(_scaled_px(32, scale, 26))
        for button in (
            self.start_btn,
            self.stop_btn,
            self.sample_btn,
            self.compute_btn,
            self.save_btn,
        ):
            button.setFixedHeight(_scaled_px(54, scale, 42))


# ── Main Window ─────────────────────────────────────────────────────

class MainWindow(QMainWindow):
    POLL_INTERVAL_MS = 33  # ~30Hz UI updates

    def __init__(self, ros: RosInterface):
        super().__init__()
        self.ros = ros
        self._ui_scale = 1.0
        self.setWindowTitle("HoloAssist Dashboard")
        self.resize(BASE_WINDOW_WIDTH, BASE_WINDOW_HEIGHT)
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

        # Left column: tabs + mode buttons
        left_col = QVBoxLayout()
        left_col.setContentsMargins(0, 0, 0, 0)
        left_col.setSpacing(0)

        # Tab widget
        self.tabs = QTabWidget()
        self.tabs.setFont(QFont("monospace", 8))

        self.status_screen = MergedStatusScreen()
        self.tabs.addTab(self.status_screen, "STATUS")

        self.headset_screen = HeadsetScreen()
        self.tabs.addTab(self.headset_screen, "HEADSET")

        self.camera_screen = CameraScreen(ros)
        self.tabs.addTab(self.camera_screen, "CAMERA")

        self.debug_screen = DebugScreen()
        self.tabs.addTab(self.debug_screen, "DEBUG")

        self.cube_screen = CubePickScreen(ros)
        self.calibration_screen = CalibrationScreen(ros)

        left_col.addWidget(self.tabs, 1)

        # Mode buttons (bottom of left column)
        mode_row = QHBoxLayout()
        mode_row.setContentsMargins(4, 4, 4, 4)
        mode_row.setSpacing(4)
        self.mode_row = mode_row

        self.teleop_btn = QPushButton("TELEOP")
        self.teleop_btn.setFont(QFont("monospace", 11, QFont.Bold))
        self.teleop_btn.setFixedHeight(80)
        self.teleop_btn.setCursor(Qt.PointingHandCursor)
        self.teleop_btn.clicked.connect(self._on_teleop)
        mode_row.addWidget(self.teleop_btn, 1)

        self.moveit_btn = QPushButton("MOVEIT")
        self.moveit_btn.setFont(QFont("monospace", 11, QFont.Bold))
        self.moveit_btn.setFixedHeight(80)
        self.moveit_btn.setCursor(Qt.PointingHandCursor)
        self.moveit_btn.clicked.connect(self._on_moveit)
        mode_row.addWidget(self.moveit_btn, 1)

        self._active_mode = "TELEOP"
        self._update_mode_buttons()
        self._set_cube_tab_visible(False)
        self._set_calibration_tab_visible(False)

        left_col.addLayout(mode_row)
        body.addLayout(left_col, 1)

        # E-stop (right side, always visible)
        self.estop = EstopWidget(ros)
        self.estop.setProperty("_skip_font_scale", True)
        body.addWidget(self.estop)

        main_layout.addLayout(body, 1)

        # Poll timer
        self._poll_timer = QTimer()
        self._poll_timer.setInterval(self.POLL_INTERVAL_MS)
        self._poll_timer.timeout.connect(self._poll)
        self._poll_timer.start()
        self._capture_base_fonts()
        self._apply_scale(force=True)

    def _capture_base_fonts(self):
        for widget in self._iter_font_widgets():
            font = widget.font()
            point_size = font.pointSizeF()
            if point_size <= 0:
                point_size = float(font.pointSize())
            if point_size > 0:
                widget.setProperty("_base_font_pt", point_size)

    def _apply_scaled_fonts(self, scale: float):
        for widget in self._iter_font_widgets():
            if widget.property("_skip_font_scale"):
                continue
            base_pt = widget.property("_base_font_pt")
            if base_pt is None:
                continue
            target_pt = _clamp(float(base_pt) * scale, 7.0, 42.0)
            font = widget.font()
            if abs(font.pointSizeF() - target_pt) > 0.05:
                font.setPointSizeF(target_pt)
                widget.setFont(font)

    def _iter_font_widgets(self):
        widgets = []
        for cls in (QLabel, QPushButton, QTextEdit, QTabWidget):
            widgets.extend(self.findChildren(cls))
        return widgets

    def _apply_scale(self, force: bool = False):
        width = self.width() if self.width() > 0 else BASE_WINDOW_WIDTH
        scale = _window_scale(width)
        if not force and abs(scale - self._ui_scale) < 0.01:
            return
        self._ui_scale = scale

        app = QApplication.instance()
        if app is not None:
            app.setStyleSheet(build_global_style(scale))

        self._apply_scaled_fonts(scale)
        self.status_bar.apply_scale(scale)
        self.estop.apply_scale(scale, self.width())
        self.debug_screen.apply_scale(scale)
        self.cube_screen.apply_scale(scale)
        self.calibration_screen.apply_scale(scale)
        self.mode_row.setContentsMargins(
            _scaled_px(4, scale, 2),
            _scaled_px(4, scale, 2),
            _scaled_px(4, scale, 2),
            _scaled_px(4, scale, 2),
        )
        self.mode_row.setSpacing(_scaled_px(4, scale, 2))
        self.teleop_btn.setFixedHeight(_scaled_px(80, scale, 56))
        self.moveit_btn.setFixedHeight(_scaled_px(80, scale, 56))
        self._update_mode_buttons()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._apply_scale()

    def _poll(self):
        status = self.ros.get_status()
        self.status_bar.update_status(status)
        self.status_screen.update_status(status)
        self.headset_screen.update_status(status)
        self.camera_screen.update_status(status)
        self.debug_screen.update_status(status)
        self.cube_screen.update_status(status)
        self.calibration_screen.update_status(status)
        self.estop.sync_state(status)
        if status.operating_mode != self._active_mode:
            self._active_mode = status.operating_mode
            self._update_mode_buttons()
        self._set_cube_tab_visible(status.operating_mode == "MOVEIT")
        self._set_calibration_tab_visible(
            status.operating_mode == "MOVEIT" or status.calibration_ready
        )

    def _on_teleop(self):
        self._active_mode = "TELEOP"
        self._update_mode_buttons()
        self._set_cube_tab_visible(False)
        self._set_calibration_tab_visible(False)
        self.ros.switch_to_teleop()

    def _on_moveit(self):
        self._active_mode = "MOVEIT"
        self._update_mode_buttons()
        self._set_cube_tab_visible(True)
        self._set_calibration_tab_visible(True)
        self.ros.switch_to_moveit()

    def _set_cube_tab_visible(self, visible: bool):
        index = self.tabs.indexOf(self.cube_screen)
        if visible and index < 0:
            calibration_index = self.tabs.indexOf(self.calibration_screen)
            insert_at = calibration_index if calibration_index >= 0 else self.tabs.count()
            self.tabs.insertTab(insert_at, self.cube_screen, "CUBE")
        elif not visible and index >= 0:
            if self.tabs.currentWidget() is self.cube_screen:
                self.tabs.setCurrentIndex(0)
            self.tabs.removeTab(index)

    def _set_calibration_tab_visible(self, visible: bool):
        index = self.tabs.indexOf(self.calibration_screen)
        if visible and index < 0:
            self.tabs.addTab(self.calibration_screen, "CALIBRATION")
        elif not visible and index >= 0:
            if self.tabs.currentWidget() is self.calibration_screen:
                self.tabs.setCurrentIndex(0)
            self.tabs.removeTab(index)

    def _update_mode_buttons(self):
        border = _scaled_px(3, self._ui_scale, 2)
        radius = _scaled_px(12, self._ui_scale, 8)
        if self._active_mode == "TELEOP":
            self.teleop_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {GREEN};
                    color: white;
                    border: {border}px solid #6fdd8b;
                    border-radius: {radius}px;
                }}
            """)
            self.moveit_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {DARK_BG};
                    color: {BLUE};
                    border: {border}px solid {BLUE};
                    border-radius: {radius}px;
                }}
                QPushButton:hover {{
                    background-color: #1a2332;
                    border-color: #79bbff;
                }}
            """)
        else:
            self.teleop_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {DARK_BG};
                    color: {GREEN};
                    border: {border}px solid {GREEN};
                    border-radius: {radius}px;
                }}
                QPushButton:hover {{
                    background-color: #1a2332;
                    border-color: #6fdd8b;
                }}
            """)
            self.moveit_btn.setStyleSheet(f"""
                QPushButton {{
                    background-color: {BLUE};
                    color: white;
                    border: {border}px solid #79bbff;
                    border-radius: {radius}px;
                }}
            """)

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
    no_ros = "--no-ros" in sys.argv
    fullscreen = "--fullscreen" in sys.argv or "-f" in sys.argv

    bridge_url = None
    for arg in sys.argv:
        if arg.startswith("--bridge="):
            bridge_url = arg[len("--bridge="):]
        elif arg == "--bridge" and sys.argv.index(arg) + 1 < len(sys.argv):
            bridge_url = sys.argv[sys.argv.index(arg) + 1]

    app = QApplication(sys.argv)
    app.setStyleSheet(build_global_style(1.0))

    # Allow Ctrl+C in terminal to close the app. PyQt5 blocks Python signals
    # while in the event loop, so a short timer wakes the interpreter regularly.
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sigint_timer = QTimer()
    sigint_timer.timeout.connect(lambda: None)
    sigint_timer.start(200)

    if bridge_url:
        print(f"Network mode: connecting to bridge at {bridge_url}")
        ros = NetInterface(url=bridge_url)
        ros.start()
    else:
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
