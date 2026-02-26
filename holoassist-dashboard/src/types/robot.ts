export type ConnectionStatus = 'Connected' | 'Disconnected';
export type RobotMode = 'Idle' | 'Planning' | 'Servoing' | 'Fault';
export type LogLevel = 'INFO' | 'WARN' | 'ERROR';

export interface TargetPose {
  x: number;
  y: number;
  z: number;
  roll: number;
  pitch: number;
  yaw: number;
}

export interface ToolPose {
  x: number;
  y: number;
  z: number;
  roll: number;
  pitch: number;
  yaw: number;
}

export interface JointTelemetry {
  positions: number[];
  velocities: number[];
}

export interface LatencySample {
  t: number;
  ms: number;
}

export interface WarningItem {
  id: string;
  label: string;
  severity: 'ok' | 'warn' | 'error';
  message: string;
}

export interface SafetySettings {
  jointLimits: boolean;
  speedLimits: boolean;
  collisionChecking: boolean;
}

export interface EventLogEntry {
  id: string;
  timestamp: string;
  level: LogLevel;
  message: string;
}

export interface TelemetryPacket {
  latencyMs: number;
  latencyHistory: LatencySample[];
  jointTelemetry: JointTelemetry;
  toolPose: ToolPose;
  warnings: WarningItem[];
}

export interface DashboardSnapshot extends TelemetryPacket {
  connectionStatus: ConnectionStatus;
  robotMode: RobotMode;
  servoEnabled: boolean;
  speedScalingPercent: number;
  targetPose: TargetPose;
  safetySettings: SafetySettings;
}

export interface CommandResult {
  ok: boolean;
  message: string;
}
