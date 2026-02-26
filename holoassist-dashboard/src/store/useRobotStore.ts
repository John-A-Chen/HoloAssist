import { create } from 'zustand';

import * as robotApi from '../services/robotApi';
import type {
  ConnectionStatus,
  DashboardSnapshot,
  EventLogEntry,
  JointTelemetry,
  LatencySample,
  LogLevel,
  RobotMode,
  SafetySettings,
  TargetPose,
  ToolPose,
  WarningItem
} from '../types/robot';

const DEFAULT_JOINTS: JointTelemetry = {
  positions: [0, 0, 0, 0, 0, 0],
  velocities: [0, 0, 0, 0, 0, 0]
};

const DEFAULT_TOOL_POSE: ToolPose = {
  x: 0,
  y: 0,
  z: 0,
  roll: 0,
  pitch: 0,
  yaw: 0
};

const DEFAULT_TARGET_POSE: TargetPose = {
  x: 0.3,
  y: 0,
  z: 0.25,
  roll: 3.14,
  pitch: 0,
  yaw: 0
};

const DEFAULT_SAFETY: SafetySettings = {
  jointLimits: true,
  speedLimits: true,
  collisionChecking: true
};

function makeLog(level: LogLevel, message: string): EventLogEntry {
  return {
    id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
    timestamp: new Date().toISOString(),
    level,
    message
  };
}

function recomputeStreaming(state: RobotStoreState): boolean {
  return (
    state.connectionStatus === 'Connected' &&
    state.robotMode !== 'Fault' &&
    state.servoEnabled &&
    state.deadmanHeld
  );
}

function canControl(state: RobotStoreState): boolean {
  return state.connectionStatus === 'Connected' && state.robotMode !== 'Fault';
}

interface RobotStoreState {
  initialised: boolean;
  loadingSnapshot: boolean;
  polling: boolean;
  connectionStatus: ConnectionStatus;
  robotMode: RobotMode;
  servoEnabled: boolean;
  deadmanHeld: boolean;
  commandStreamingEnabled: boolean;
  speedScalingPercent: number;
  targetPose: TargetPose;
  safetySettings: SafetySettings;
  warnings: WarningItem[];
  latencyMs: number;
  latencyHistory: LatencySample[];
  jointTelemetry: JointTelemetry;
  toolPose: ToolPose;
  eventLogs: EventLogEntry[];
  logFilters: Record<LogLevel, boolean>;
  eStopModalOpen: boolean;
  digitalTwinFullscreen: boolean;
  bootstrap: () => Promise<void>;
  pollTelemetry: () => Promise<void>;
  toggleConnection: () => Promise<void>;
  setRobotMode: (mode: RobotMode) => Promise<void>;
  openEstopModal: () => void;
  closeEstopModal: () => void;
  confirmEstop: () => Promise<void>;
  resetFault: () => Promise<void>;
  setServoEnabled: (enabled: boolean) => Promise<void>;
  setDeadmanHeld: (held: boolean) => void;
  sendZeroCommand: () => Promise<void>;
  setSpeedScalingPercent: (value: number) => void;
  updateTargetPoseField: (field: keyof TargetPose, value: number) => void;
  planTarget: () => Promise<void>;
  executePlan: () => Promise<void>;
  setTargetFromClickedPointPlaceholder: () => void;
  toggleSafetySetting: (field: keyof SafetySettings) => void;
  setDigitalTwinFullscreen: (value: boolean) => void;
  toggleLogFilter: (level: LogLevel) => void;
  addLog: (level: LogLevel, message: string) => void;
}

function applySnapshot(snapshot: DashboardSnapshot, prev: RobotStoreState): Partial<RobotStoreState> {
  const nextPartial: Partial<RobotStoreState> = {
    connectionStatus: snapshot.connectionStatus,
    robotMode: snapshot.robotMode,
    servoEnabled: snapshot.servoEnabled,
    speedScalingPercent: snapshot.speedScalingPercent,
    targetPose: snapshot.targetPose,
    safetySettings: snapshot.safetySettings,
    warnings: snapshot.warnings,
    latencyMs: snapshot.latencyMs,
    latencyHistory: snapshot.latencyHistory,
    jointTelemetry: snapshot.jointTelemetry,
    toolPose: snapshot.toolPose
  };

  const merged = { ...prev, ...nextPartial } as RobotStoreState;
  nextPartial.commandStreamingEnabled = recomputeStreaming(merged);
  if (!canControl(merged) && merged.deadmanHeld) {
    nextPartial.deadmanHeld = false;
    nextPartial.commandStreamingEnabled = false;
  }
  return nextPartial;
}

export const useRobotStore = create<RobotStoreState>((set, get) => ({
  initialised: false,
  loadingSnapshot: false,
  polling: false,
  connectionStatus: 'Disconnected',
  robotMode: 'Idle',
  servoEnabled: false,
  deadmanHeld: false,
  commandStreamingEnabled: false,
  speedScalingPercent: 50,
  targetPose: DEFAULT_TARGET_POSE,
  safetySettings: DEFAULT_SAFETY,
  warnings: [],
  latencyMs: 0,
  latencyHistory: [],
  jointTelemetry: DEFAULT_JOINTS,
  toolPose: DEFAULT_TOOL_POSE,
  eventLogs: [makeLog('INFO', 'Dashboard ready (mock mode)')],
  logFilters: { INFO: true, WARN: true, ERROR: true },
  eStopModalOpen: false,
  digitalTwinFullscreen: false,

  addLog: (level, message) => {
    set((state) => ({ eventLogs: [makeLog(level, message), ...state.eventLogs].slice(0, 300) }));
  },

  bootstrap: async () => {
    if (get().loadingSnapshot || get().initialised) {
      return;
    }
    set({ loadingSnapshot: true });
    try {
      const snapshot = await robotApi.getDashboardSnapshot();
      set((state) => ({
        ...applySnapshot(snapshot, state),
        initialised: true,
        loadingSnapshot: false,
        eventLogs: [makeLog('INFO', 'Loaded initial mock dashboard snapshot'), ...state.eventLogs].slice(0, 300)
      }));
    } catch (error) {
      set({ loadingSnapshot: false });
      get().addLog('ERROR', `Failed to load mock snapshot: ${String(error)}`);
    }
  },

  pollTelemetry: async () => {
    if (!get().initialised || get().polling) {
      return;
    }
    set({ polling: true });
    try {
      const telemetry = await robotApi.pollTelemetry();
      set((state) => {
        const next = {
          ...state,
          latencyMs: telemetry.latencyMs,
          latencyHistory: telemetry.latencyHistory,
          jointTelemetry: telemetry.jointTelemetry,
          toolPose: telemetry.toolPose,
          warnings: telemetry.warnings,
          polling: false
        } as RobotStoreState;
        return {
          latencyMs: next.latencyMs,
          latencyHistory: next.latencyHistory,
          jointTelemetry: next.jointTelemetry,
          toolPose: next.toolPose,
          warnings: next.warnings,
          polling: false,
          commandStreamingEnabled: recomputeStreaming(next)
        };
      });
    } catch (error) {
      set({ polling: false });
      get().addLog('ERROR', `Telemetry poll failed: ${String(error)}`);
    }
  },

  toggleConnection: async () => {
    const current = get().connectionStatus;
    const next: ConnectionStatus = current === 'Connected' ? 'Disconnected' : 'Connected';
    const result = await robotApi.setConnectionStatus(next);
    if (!result.ok) {
      get().addLog('ERROR', result.message);
      return;
    }
    set((state) => {
      const nextState = {
        ...state,
        connectionStatus: next,
        robotMode: next === 'Disconnected' ? 'Idle' : state.robotMode,
        servoEnabled: next === 'Disconnected' ? false : state.servoEnabled,
        deadmanHeld: false,
        eStopModalOpen: false
      } as RobotStoreState;
      return {
        connectionStatus: nextState.connectionStatus,
        robotMode: nextState.robotMode,
        servoEnabled: nextState.servoEnabled,
        deadmanHeld: nextState.deadmanHeld,
        eStopModalOpen: nextState.eStopModalOpen,
        commandStreamingEnabled: recomputeStreaming(nextState)
      };
    });
    get().addLog('INFO', result.message);
  },

  setRobotMode: async (mode) => {
    const state = get();
    if (state.connectionStatus === 'Disconnected') {
      state.addLog('WARN', 'Connect before changing mode');
      return;
    }
    if (state.robotMode === 'Fault' && mode !== 'Fault') {
      state.addLog('WARN', 'Reset fault before changing mode');
      return;
    }
    const result = await robotApi.setRobotMode(mode);
    if (!result.ok) {
      get().addLog('ERROR', result.message);
      return;
    }
    set((prev) => {
      const nextState = { ...prev, robotMode: mode, servoEnabled: mode === 'Servoing' ? prev.servoEnabled : false } as RobotStoreState;
      return {
        robotMode: nextState.robotMode,
        servoEnabled: nextState.servoEnabled,
        commandStreamingEnabled: recomputeStreaming(nextState)
      };
    });
    get().addLog('INFO', result.message);
  },

  openEstopModal: () => set({ eStopModalOpen: true }),
  closeEstopModal: () => set({ eStopModalOpen: false }),

  confirmEstop: async () => {
    const result = await robotApi.triggerEstop();
    if (!result.ok) {
      get().addLog('ERROR', result.message);
      return;
    }
    set((state) => {
      const nextState = {
        ...state,
        robotMode: 'Fault',
        servoEnabled: false,
        deadmanHeld: false,
        eStopModalOpen: false
      } as RobotStoreState;
      return {
        robotMode: 'Fault',
        servoEnabled: false,
        deadmanHeld: false,
        eStopModalOpen: false,
        commandStreamingEnabled: recomputeStreaming(nextState)
      };
    });
    get().addLog('ERROR', result.message);
  },

  resetFault: async () => {
    if (get().robotMode !== 'Fault') {
      return;
    }
    const result = await robotApi.resetFault();
    if (!result.ok) {
      get().addLog('WARN', result.message);
      return;
    }
    set((state) => {
      const nextState = { ...state, robotMode: 'Idle', deadmanHeld: false } as RobotStoreState;
      return {
        robotMode: 'Idle',
        deadmanHeld: false,
        commandStreamingEnabled: recomputeStreaming(nextState)
      };
    });
    get().addLog('INFO', result.message);
  },

  setServoEnabled: async (enabled) => {
    const state = get();
    if (!canControl(state)) {
      state.addLog('WARN', 'Servo enable blocked by connection/fault state');
      return;
    }
    const result = await robotApi.setServoEnabled(enabled);
    if (!result.ok) {
      get().addLog('WARN', result.message);
      return;
    }
    set((prev) => {
      const nextMode: RobotMode = enabled ? 'Servoing' : prev.robotMode === 'Servoing' ? 'Idle' : prev.robotMode;
      const nextState = {
        ...prev,
        servoEnabled: enabled,
        robotMode: nextMode,
        deadmanHeld: enabled ? prev.deadmanHeld : false
      } as RobotStoreState;
      return {
        servoEnabled: nextState.servoEnabled,
        robotMode: nextState.robotMode,
        deadmanHeld: nextState.deadmanHeld,
        commandStreamingEnabled: recomputeStreaming(nextState)
      };
    });
    get().addLog('INFO', result.message);
  },

  setDeadmanHeld: (held) => {
    set((state) => {
      const allowed = held ? canControl(state) && state.servoEnabled : false;
      const nextState = { ...state, deadmanHeld: held ? allowed : false } as RobotStoreState;
      return {
        deadmanHeld: nextState.deadmanHeld,
        commandStreamingEnabled: recomputeStreaming(nextState)
      };
    });
  },

  sendZeroCommand: async () => {
    const state = get();
    if (!canControl(state)) {
      state.addLog('WARN', 'Zero command blocked by connection/fault state');
      return;
    }
    const result = await robotApi.sendZeroCommand();
    if (result.ok) {
      get().addLog('INFO', result.message);
    } else {
      get().addLog('ERROR', result.message);
    }
  },

  setSpeedScalingPercent: (value) => {
    set({ speedScalingPercent: Math.round(value) });
    void robotApi.setSpeedScalingPercent(value).then((result) => {
      if (!result.ok) {
        get().addLog('ERROR', result.message);
      }
    });
  },

  updateTargetPoseField: (field, value) => {
    const nextPose = { ...get().targetPose, [field]: value } as TargetPose;
    set({ targetPose: nextPose });
    void robotApi.setTargetPose(nextPose).then((result) => {
      if (!result.ok) {
        get().addLog('ERROR', result.message);
      }
    });
  },

  planTarget: async () => {
    const state = get();
    if (!canControl(state)) {
      state.addLog('WARN', 'Planning unavailable while disconnected/faulted');
      return;
    }
    const result = await robotApi.planTarget();
    if (!result.ok) {
      get().addLog('ERROR', result.message);
      return;
    }
    set({ robotMode: 'Planning' });
    get().addLog('INFO', result.message);
  },

  executePlan: async () => {
    const state = get();
    if (!canControl(state)) {
      state.addLog('WARN', 'Execution unavailable while disconnected/faulted');
      return;
    }
    const result = await robotApi.executePlan();
    if (!result.ok) {
      get().addLog('ERROR', result.message);
      return;
    }
    set({ robotMode: 'Idle' });
    get().addLog('INFO', result.message);
  },

  setTargetFromClickedPointPlaceholder: () => {
    get().addLog('INFO', 'Placeholder: set target from clicked point (ROS hook to be added later)');
  },

  toggleSafetySetting: (field) => {
    const next = {
      ...get().safetySettings,
      [field]: !get().safetySettings[field]
    } as SafetySettings;
    set({ safetySettings: next });
    void robotApi.setSafetySettings(next).then((result) => {
      get().addLog(result.ok ? 'INFO' : 'ERROR', result.message);
    });
  },

  setDigitalTwinFullscreen: (value) => {
    set({ digitalTwinFullscreen: value });
    get().addLog('INFO', value ? 'Digital Twin View expanded' : 'Digital Twin View restored');
  },

  toggleLogFilter: (level) => {
    set((state) => ({
      logFilters: {
        ...state.logFilters,
        [level]: !state.logFilters[level]
      }
    }));
  }
}));
