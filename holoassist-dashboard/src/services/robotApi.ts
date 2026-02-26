import type {
  CommandResult,
  ConnectionStatus,
  DashboardSnapshot,
  LatencySample,
  RobotMode,
  SafetySettings,
  TargetPose,
  TelemetryPacket,
  ToolPose,
  WarningItem
} from '../types/robot';

const JOINT_COUNT = 6;
const LATENCY_WINDOW = 36;

interface MockRobotBackendState {
  connectionStatus: ConnectionStatus;
  robotMode: RobotMode;
  servoEnabled: boolean;
  speedScalingPercent: number;
  targetPose: TargetPose;
  safetySettings: SafetySettings;
  jointPositions: number[];
  jointVelocities: number[];
  toolPose: ToolPose;
  latencyHistory: LatencySample[];
  tick: number;
}

const state: MockRobotBackendState = {
  connectionStatus: 'Connected',
  robotMode: 'Idle',
  servoEnabled: false,
  speedScalingPercent: 45,
  targetPose: { x: 0.32, y: -0.08, z: 0.26, roll: 3.14, pitch: 0.0, yaw: 0.0 },
  safetySettings: {
    jointLimits: true,
    speedLimits: true,
    collisionChecking: true
  },
  jointPositions: [0.0, -1.35, 1.85, -1.65, -1.55, 0.05],
  jointVelocities: [0, 0, 0, 0, 0, 0],
  toolPose: { x: 0.31, y: -0.07, z: 0.24, roll: 3.14, pitch: -0.02, yaw: 0.01 },
  latencyHistory: [],
  tick: 0
};

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value));
}

function sleep(ms = 120): Promise<void> {
  return new Promise((resolve) => setTimeout(resolve, ms));
}

function nowStamp(): number {
  return Date.now();
}

function buildWarnings(): WarningItem[] {
  const warnings: WarningItem[] = [];

  warnings.push({
    id: 'collision',
    label: 'Collision checking',
    severity: state.safetySettings.collisionChecking ? 'ok' : 'warn',
    message: state.safetySettings.collisionChecking
      ? 'Collision checking active'
      : 'Collision checking disabled'
  });

  warnings.push({
    id: 'speed',
    label: 'Speed scaling',
    severity: state.speedScalingPercent < 20 ? 'warn' : 'ok',
    message:
      state.speedScalingPercent < 20
        ? `Speed scaling low (${state.speedScalingPercent}%)`
        : `Speed scaling ${state.speedScalingPercent}%`
  });

  if (state.connectionStatus === 'Disconnected') {
    warnings.push({
      id: 'link',
      label: 'Link status',
      severity: 'error',
      message: 'Robot link disconnected'
    });
  }

  if (state.robotMode === 'Fault') {
    warnings.push({
      id: 'fault',
      label: 'Robot mode',
      severity: 'error',
      message: 'Fault latched. Reset required.'
    });
  }

  if (state.robotMode !== 'Fault' && (state.tick % 80) > 62) {
    warnings.push({
      id: 'near-singularity',
      label: 'Kinematics',
      severity: 'warn',
      message: 'Mock: approaching wrist singularity region'
    });
  }

  return warnings;
}

function updateMotionModel(): void {
  state.tick += 1;
  const t = state.tick / 10;

  const connected = state.connectionStatus === 'Connected';
  const active = connected && state.robotMode !== 'Fault';
  const moving = active && (state.robotMode === 'Servoing' || state.robotMode === 'Planning');
  const amplitude = moving ? 0.012 : 0.003;
  const velocityScale = moving ? 0.18 : 0.04;

  for (let i = 0; i < JOINT_COUNT; i += 1) {
    const wave = Math.sin(t * (0.7 + i * 0.08) + i * 0.9);
    const delta = amplitude * wave;
    state.jointPositions[i] = clamp(state.jointPositions[i] + delta, -Math.PI, Math.PI);
    state.jointVelocities[i] = connected ? velocityScale * Math.cos(t * 1.6 + i) : 0;
  }

  const targetBias = state.servoEnabled && state.robotMode === 'Servoing' ? 0.008 : 0.0015;
  state.toolPose = {
    x: clamp(state.toolPose.x + targetBias * Math.sin(t * 0.9), 0.15, 0.5),
    y: clamp(state.toolPose.y + targetBias * Math.cos(t * 0.7), -0.35, 0.35),
    z: clamp(state.toolPose.z + targetBias * Math.sin(t * 0.5 + 0.2), 0.08, 0.45),
    roll: 3.14,
    pitch: clamp(-0.08 + 0.06 * Math.sin(t * 0.4), -0.3, 0.3),
    yaw: clamp(0.12 * Math.sin(t * 0.35), -0.4, 0.4)
  };

  const baseLatency = connected ? 18 : 0;
  const jitter = connected ? Math.round(6 + 18 * Math.abs(Math.sin(t * 0.6))) : 0;
  const modePenalty = state.robotMode === 'Servoing' ? 8 : state.robotMode === 'Planning' ? 5 : 0;
  const latencyMs = connected ? baseLatency + jitter + modePenalty : 0;

  const sample: LatencySample = { t: nowStamp(), ms: latencyMs };
  state.latencyHistory = [...state.latencyHistory, sample].slice(-LATENCY_WINDOW);
}

function snapshotTelemetry(): TelemetryPacket {
  updateMotionModel();
  const latestLatency = state.latencyHistory[state.latencyHistory.length - 1]?.ms ?? 0;

  return {
    latencyMs: latestLatency,
    latencyHistory: [...state.latencyHistory],
    jointTelemetry: {
      positions: [...state.jointPositions],
      velocities: [...state.jointVelocities]
    },
    toolPose: { ...state.toolPose },
    warnings: buildWarnings()
  };
}

function modeAllowed(mode: RobotMode): boolean {
  if (state.connectionStatus === 'Disconnected') {
    return false;
  }
  if (state.robotMode === 'Fault' && mode !== 'Fault') {
    return false;
  }
  return true;
}

export async function getDashboardSnapshot(): Promise<DashboardSnapshot> {
  await sleep(120);
  const telemetry = snapshotTelemetry();
  return {
    connectionStatus: state.connectionStatus,
    robotMode: state.robotMode,
    servoEnabled: state.servoEnabled,
    speedScalingPercent: state.speedScalingPercent,
    targetPose: { ...state.targetPose },
    safetySettings: { ...state.safetySettings },
    ...telemetry
  };
}

export async function pollTelemetry(): Promise<TelemetryPacket> {
  await sleep(30);
  return snapshotTelemetry();
}

export async function setConnectionStatus(next: ConnectionStatus): Promise<CommandResult> {
  await sleep(90);
  state.connectionStatus = next;
  if (next === 'Disconnected') {
    state.robotMode = 'Idle';
    state.servoEnabled = false;
    state.jointVelocities = new Array(JOINT_COUNT).fill(0);
  }
  return {
    ok: true,
    message: next === 'Connected' ? 'Connection established (mock)' : 'Connection closed (mock)'
  };
}

export async function setRobotMode(mode: RobotMode): Promise<CommandResult> {
  await sleep(80);
  if (!modeAllowed(mode)) {
    return { ok: false, message: 'Mode change blocked by connection/fault state' };
  }
  state.robotMode = mode;
  if (mode !== 'Servoing') {
    state.servoEnabled = false;
  }
  return { ok: true, message: `Robot mode set to ${mode}` };
}

export async function triggerEstop(): Promise<CommandResult> {
  await sleep(70);
  state.robotMode = 'Fault';
  state.servoEnabled = false;
  state.jointVelocities = new Array(JOINT_COUNT).fill(0);
  return { ok: true, message: 'E-stop triggered. Fault latched.' };
}

export async function resetFault(): Promise<CommandResult> {
  await sleep(140);
  if (state.connectionStatus === 'Disconnected') {
    return { ok: false, message: 'Reconnect before fault reset' };
  }
  state.robotMode = 'Idle';
  return { ok: true, message: 'Fault reset complete (mock)' };
}

export async function setServoEnabled(enabled: boolean): Promise<CommandResult> {
  await sleep(75);
  if (state.connectionStatus === 'Disconnected' || state.robotMode === 'Fault') {
    return { ok: false, message: 'Servo enable blocked' };
  }
  state.servoEnabled = enabled;
  if (enabled) {
    state.robotMode = 'Servoing';
  } else if (state.robotMode === 'Servoing') {
    state.robotMode = 'Idle';
  }
  return { ok: true, message: enabled ? 'Servo enabled (mock)' : 'Servo disabled (mock)' };
}

export async function setSpeedScalingPercent(value: number): Promise<CommandResult> {
  await sleep(40);
  state.speedScalingPercent = clamp(Math.round(value), 0, 100);
  return { ok: true, message: `Speed scaling ${state.speedScalingPercent}%` };
}

export async function setTargetPose(pose: TargetPose): Promise<CommandResult> {
  await sleep(50);
  state.targetPose = { ...pose };
  return { ok: true, message: 'Target pose updated' };
}

export async function setSafetySettings(settings: SafetySettings): Promise<CommandResult> {
  await sleep(50);
  state.safetySettings = { ...settings };
  return { ok: true, message: 'Safety settings updated' };
}

export async function sendZeroCommand(): Promise<CommandResult> {
  await sleep(45);
  state.jointVelocities = new Array(JOINT_COUNT).fill(0);
  return { ok: true, message: 'Zero command sent (mock)' };
}

export async function planTarget(): Promise<CommandResult> {
  await sleep(220);
  if (state.connectionStatus === 'Disconnected' || state.robotMode === 'Fault') {
    return { ok: false, message: 'Planning unavailable' };
  }
  state.robotMode = 'Planning';
  return { ok: true, message: 'Plan generated (mock)' };
}

export async function executePlan(): Promise<CommandResult> {
  await sleep(260);
  if (state.connectionStatus === 'Disconnected' || state.robotMode === 'Fault') {
    return { ok: false, message: 'Execution unavailable' };
  }
  state.robotMode = 'Idle';
  state.toolPose = {
    x: state.targetPose.x,
    y: state.targetPose.y,
    z: state.targetPose.z,
    roll: state.targetPose.roll,
    pitch: state.targetPose.pitch,
    yaw: state.targetPose.yaw
  };
  return { ok: true, message: 'Plan executed (mock)' };
}
