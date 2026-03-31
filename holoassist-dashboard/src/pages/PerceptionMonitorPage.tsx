import { useEffect, useMemo, useState } from 'react';

import { Badge } from '../components/ui/badge';
import { Button } from '../components/ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../components/ui/card';

interface RelayStatus {
  server_time_unix_ms: number;
  bridge: {
    bind_host: string;
    bind_port: number;
    uptime_s: number;
    stale_timeout_s: number;
    max_image_fps: number;
    jpeg_quality: number;
    rate_window_s: number;
  };
  topics: {
    debug_image_topic: string;
    fallback_rgb_topic?: string;
    bbox_topic: string;
    pointcloud_topic: string;
    obstacle_topic: string;
    joint_states_topic: string;
    target_pose_topic: string;
    twist_topic: string;
    clicked_point_topic: string;
    unity_map_loaded_topic: string;
  };
  source: {
    mode: 'depth_tracker' | 'rgb_fallback' | 'no_data';
    active_image_topic: string;
    latest_image_source: string;
  };
  capabilities: {
    depth_supported: boolean;
    pointcloud_supported: boolean;
    obstacle_supported: boolean;
    message: string;
  };
  tracker_connection: {
    connected: boolean;
    pipeline_connected: boolean;
    stale: boolean;
    last_debug_age_s: number;
    last_rgb_age_s: number;
    last_bbox_age_s: number;
    last_pointcloud_age_s: number;
    last_obstacle_age_s: number;
  };
  rates_hz: Record<string, number>;
  counters: {
    debug_image_rx_count: number;
    raw_image_rx_count: number;
    bbox_rx_count: number;
    pointcloud_rx_count: number;
    obstacle_rx_count: number;
    joint_state_rx_count: number;
    target_pose_rx_count: number;
    twist_cmd_rx_count: number;
    clicked_point_rx_count: number;
    last_point_count: number;
  };
  latest: {
    obstacle_active: boolean;
    bbox_valid: boolean;
    bbox_raw: number[];
    bbox: {
      x_min_px: number;
      y_min_px: number;
      x_max_px: number;
      y_max_px: number;
      cx_px: number;
      cy_px: number;
      median_depth_m: number;
      area_px: number;
    } | null;
    jpeg_available: boolean;
    latest_debug_frame_id: string;
    image_source: string;
  };
  robot_state: {
    enabled: boolean;
    joint_state_available: boolean;
    joint_state_age_s: number;
    joint_count: number;
    joints: Array<{
      name: string;
      position_rad: number | null;
      velocity_rad_s: number | null;
      effort: number | null;
    }>;
    eef_pose_available: boolean;
    eef_pose_age_s: number;
    eef_pose: {
      frame: string;
      x: number;
      y: number;
      z: number;
      qx: number;
      qy: number;
      qz: number;
      qw: number;
    } | null;
    command_frame: string;
    eef_frame: string;
    command_metrics: {
      last_target_age_s: number;
      last_twist_age_s: number;
      last_clicked_point_age_s: number;
      last_target_transport_latency_ms: number | null;
      avg_target_transport_latency_ms: number | null;
      last_ee_response_latency_ms: number | null;
      pending_goal: boolean;
      eef_motion_threshold_m: number;
    };
  };
  unity_integration: {
    tcp_host: string;
    tcp_port: number;
    tcp_endpoint_reachable: boolean;
    last_probe_age_s: number;
    last_success_age_s: number;
    message: string;
    map_signal_received: boolean;
    map_signal_age_s: number;
    map_loaded: boolean | null;
    map_signal_count: number;
  };
}

const API_BASE = import.meta.env.VITE_PERCEPTION_API_BASE ?? '';
const STATUS_POLL_MS = 300;
const IMAGE_REFRESH_MS = 250;

type TabId = 'overview' | 'perception' | 'robot' | 'unity';

function apiPath(path: string): string {
  return `${API_BASE}${path}`;
}

function formatAge(ageSeconds: number): string {
  if (ageSeconds < 0) {
    return 'n/a';
  }
  return `${ageSeconds.toFixed(2)} s`;
}

function formatMaybe(value: number | null | undefined, digits = 3): string {
  if (value === null || value === undefined) {
    return 'n/a';
  }
  return value.toFixed(digits);
}

export function PerceptionMonitorPage() {
  const [status, setStatus] = useState<RelayStatus | null>(null);
  const [bridgeOnline, setBridgeOnline] = useState(false);
  const [errorText, setErrorText] = useState<string | null>(null);
  const [imageTick, setImageTick] = useState(0);
  const [activeTab, setActiveTab] = useState<TabId>('overview');

  useEffect(() => {
    let cancelled = false;
    let polling = false;

    const pollStatus = async () => {
      if (polling) {
        return;
      }
      polling = true;
      try {
        const response = await fetch(apiPath('/api/perception/status'), {
          cache: 'no-store'
        });
        if (!response.ok) {
          throw new Error(`HTTP ${response.status}`);
        }
        const payload = (await response.json()) as RelayStatus;
        if (cancelled) {
          return;
        }
        setStatus(payload);
        setBridgeOnline(true);
        setErrorText(null);
      } catch (error) {
        if (cancelled) {
          return;
        }
        setBridgeOnline(false);
        setErrorText(error instanceof Error ? error.message : String(error));
      } finally {
        polling = false;
      }
    };

    void pollStatus();
    const pollId = window.setInterval(() => {
      void pollStatus();
    }, STATUS_POLL_MS);

    return () => {
      cancelled = true;
      window.clearInterval(pollId);
    };
  }, []);

  useEffect(() => {
    if (!bridgeOnline || !status?.latest.jpeg_available) {
      return;
    }

    const imageId = window.setInterval(() => {
      setImageTick((value) => value + 1);
    }, IMAGE_REFRESH_MS);

    return () => {
      window.clearInterval(imageId);
    };
  }, [bridgeOnline, status?.latest.jpeg_available]);

  const imageUrl = useMemo(
    () => `${apiPath('/api/perception/debug.jpg')}?tick=${imageTick}`,
    [imageTick]
  );

  const sourceMode = status?.source.mode ?? 'no_data';
  const pipelineConnected = status?.tracker_connection.pipeline_connected ?? false;
  const robotConnected = status?.robot_state.joint_state_available ?? false;
  const unityReachable = status?.unity_integration.tcp_endpoint_reachable ?? false;

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100">
      <main className="mx-auto max-w-[1700px] px-4 py-5">
        <div className="mb-4 flex flex-wrap items-center justify-between gap-3">
          <div>
            <h1 className="text-xl font-semibold tracking-wide">HoloAssist System Dashboard</h1>
            <p className="text-sm text-slate-400">Perception + Robot State + Unity Integration</p>
          </div>
          <div className="flex flex-wrap items-center gap-2">
            <Badge tone={bridgeOnline ? 'success' : 'error'}>
              {bridgeOnline ? 'Relay reachable' : 'Relay unreachable'}
            </Badge>
            <Badge tone={pipelineConnected ? 'success' : 'warn'}>
              {pipelineConnected ? 'Perception live' : 'Perception stale'}
            </Badge>
            <Badge tone={robotConnected ? 'success' : 'warn'}>
              {robotConnected ? 'Robot state live' : 'Robot state stale'}
            </Badge>
            <Badge tone={unityReachable ? 'success' : 'warn'}>
              {unityReachable ? 'Unity bridge reachable' : 'Unity bridge unreachable'}
            </Badge>
            <Badge tone={sourceMode === 'depth_tracker' ? 'info' : sourceMode === 'rgb_fallback' ? 'warn' : 'neutral'}>
              {sourceMode === 'depth_tracker' ? 'Depth mode' : sourceMode === 'rgb_fallback' ? 'RGB fallback mode' : 'No stream'}
            </Badge>
          </div>
        </div>

        {!bridgeOnline ? (
          <Card className="mb-4 border-red-900/70 bg-red-950/20">
            <CardContent className="p-4 text-sm text-red-200">
              Dashboard relay request failed.
              <span className="ml-2 font-mono text-xs text-red-300">{errorText ?? 'Unknown error'}</span>
            </CardContent>
          </Card>
        ) : null}

        <Card className="mb-4">
          <CardContent className="flex flex-wrap gap-2 p-3">
            <TabButton label="Overview" active={activeTab === 'overview'} onClick={() => setActiveTab('overview')} />
            <TabButton label="Perception" active={activeTab === 'perception'} onClick={() => setActiveTab('perception')} />
            <TabButton label="Robot State" active={activeTab === 'robot'} onClick={() => setActiveTab('robot')} />
            <TabButton label="Unity Bridge" active={activeTab === 'unity'} onClick={() => setActiveTab('unity')} />
          </CardContent>
        </Card>

        {activeTab === 'overview' ? (
          <OverviewTab status={status} imageUrl={imageUrl} />
        ) : null}
        {activeTab === 'perception' ? (
          <PerceptionTab status={status} imageUrl={imageUrl} />
        ) : null}
        {activeTab === 'robot' ? <RobotTab status={status} /> : null}
        {activeTab === 'unity' ? <UnityTab status={status} /> : null}
      </main>
    </div>
  );
}

function OverviewTab({ status, imageUrl }: { status: RelayStatus | null; imageUrl: string }) {
  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-[1.3fr_1fr]">
      <Card>
        <CardHeader>
          <CardTitle>Live Camera Feed</CardTitle>
          <CardDescription>Active topic: {status?.source.active_image_topic || 'n/a'}</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="relative overflow-hidden rounded-lg border border-slate-800 bg-slate-950/80">
            {status?.latest.jpeg_available ? (
              <img src={imageUrl} alt="HoloAssist camera stream" className="h-auto min-h-[24rem] w-full object-contain" />
            ) : (
              <div className="flex min-h-[24rem] items-center justify-center text-sm text-slate-400">
                Waiting for image stream...
              </div>
            )}
          </div>
        </CardContent>
      </Card>

      <div className="space-y-4">
        <Card>
          <CardHeader>
            <CardTitle>Subsystem Summary</CardTitle>
            <CardDescription>Current operating mode</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2 text-sm">
            <Metric label="Source mode" value={status?.source.mode || 'n/a'} />
            <Metric label="Depth support" value={status?.capabilities.depth_supported ? 'Yes' : 'No'} />
            <Metric label="Pointcloud support" value={status?.capabilities.pointcloud_supported ? 'Yes' : 'No'} />
            <Metric label="Obstacle active" value={status?.latest.obstacle_active ? 'True' : 'False'} />
            <Metric label="Joint states available" value={status?.robot_state.joint_state_available ? 'True' : 'False'} />
            <Metric label="Unity endpoint reachable" value={status?.unity_integration.tcp_endpoint_reachable ? 'True' : 'False'} />
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>Live Rates (Hz)</CardTitle>
            <CardDescription>Measured over relay rolling window</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2 text-sm">
            <Metric label="debug_image" value={formatMaybe(status?.rates_hz.debug_image, 2)} />
            <Metric label="pointcloud" value={formatMaybe(status?.rates_hz.pointcloud, 2)} />
            <Metric label="joint_states" value={formatMaybe(status?.rates_hz.joint_states, 2)} />
            <Metric label="target_pose" value={formatMaybe(status?.rates_hz.target_pose, 2)} />
            <Metric label="twist_cmd" value={formatMaybe(status?.rates_hz.twist_cmd, 2)} />
            <Metric label="eef_pose" value={formatMaybe(status?.rates_hz.eef_pose, 2)} />
          </CardContent>
        </Card>
      </div>
    </div>
  );
}

function PerceptionTab({ status, imageUrl }: { status: RelayStatus | null; imageUrl: string }) {
  const pointcloudSupported = status?.capabilities.pointcloud_supported ?? false;

  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-[1.5fr_1fr]">
      <Card>
        <CardHeader>
          <div>
            <CardTitle>Perception Stream</CardTitle>
            <CardDescription>Image source: {status?.source.active_image_topic || 'n/a'}</CardDescription>
          </div>
          <Badge tone={status?.latest.jpeg_available ? 'success' : 'warn'}>
            {status?.latest.jpeg_available ? 'Streaming' : 'Waiting'}
          </Badge>
        </CardHeader>
        <CardContent>
          <div className="relative overflow-hidden rounded-lg border border-slate-800 bg-slate-950/80">
            {status?.latest.jpeg_available ? (
              <img src={imageUrl} alt="Perception stream" className="h-auto min-h-[24rem] w-full object-contain" />
            ) : (
              <div className="flex min-h-[24rem] items-center justify-center text-sm text-slate-400">
                Waiting for image stream...
              </div>
            )}
          </div>
        </CardContent>
      </Card>

      <div className="space-y-4">
        <Card>
          <CardHeader>
            <CardTitle>Perception Health</CardTitle>
            <CardDescription>Freshness and counters</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2 text-sm">
            <Metric label="Debug frame age" value={formatAge(status?.tracker_connection.last_debug_age_s ?? -1)} />
            <Metric label="RGB frame age" value={formatAge(status?.tracker_connection.last_rgb_age_s ?? -1)} />
            <Metric label="BBox age" value={formatAge(status?.tracker_connection.last_bbox_age_s ?? -1)} />
            <Metric label="Pointcloud age" value={formatAge(status?.tracker_connection.last_pointcloud_age_s ?? -1)} />
            <Metric label="Obstacle age" value={formatAge(status?.tracker_connection.last_obstacle_age_s ?? -1)} />
            <Metric label="debug_image rate" value={formatMaybe(status?.rates_hz.debug_image, 2)} />
            <Metric label="pointcloud rate" value={formatMaybe(status?.rates_hz.pointcloud, 2)} />
            <Metric label="bbox count" value={String(status?.counters.bbox_rx_count ?? 0)} />
            <Metric label="pointcloud count" value={String(status?.counters.pointcloud_rx_count ?? 0)} />
            <Metric label="last point count" value={pointcloudSupported ? String(status?.counters.last_point_count ?? 0) : 'Unsupported'} />
            {!pointcloudSupported ? (
              <div className="rounded-md border border-amber-900/70 bg-amber-950/20 p-3 text-amber-200">
                {status?.capabilities.message || 'Depth unavailable; pointcloud and obstacle outputs are unsupported.'}
              </div>
            ) : null}
          </CardContent>
        </Card>

        <Card>
          <CardHeader>
            <CardTitle>Current BBox</CardTitle>
            <CardDescription>Topic: {status?.topics.bbox_topic ?? 'n/a'}</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2 text-sm">
            {!pointcloudSupported ? (
              <div className="rounded-md border border-slate-800 bg-slate-900/60 p-3 text-slate-400">
                RGB fallback mode active. No depth-derived bbox.
              </div>
            ) : status?.latest.bbox ? (
              <>
                <Metric label="x_min / y_min" value={`${status.latest.bbox.x_min_px.toFixed(1)} / ${status.latest.bbox.y_min_px.toFixed(1)} px`} />
                <Metric label="x_max / y_max" value={`${status.latest.bbox.x_max_px.toFixed(1)} / ${status.latest.bbox.y_max_px.toFixed(1)} px`} />
                <Metric label="cx / cy" value={`${status.latest.bbox.cx_px.toFixed(1)} / ${status.latest.bbox.cy_px.toFixed(1)} px`} />
                <Metric label="Median depth" value={`${status.latest.bbox.median_depth_m.toFixed(3)} m`} />
                <Metric label="Blob area" value={`${status.latest.bbox.area_px.toFixed(0)} px`} />
              </>
            ) : (
              <div className="rounded-md border border-slate-800 bg-slate-900/60 p-3 text-slate-400">
                No valid obstacle blob in configured depth band.
              </div>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  );
}

function RobotTab({ status }: { status: RelayStatus | null }) {
  const joints = status?.robot_state.joints ?? [];

  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-[1fr_1fr]">
      <Card>
        <CardHeader>
          <CardTitle>Robot State</CardTitle>
          <CardDescription>Joint telemetry and end-effector pose</CardDescription>
        </CardHeader>
        <CardContent className="space-y-2 text-sm">
          <Metric label="Joint state age" value={formatAge(status?.robot_state.joint_state_age_s ?? -1)} />
          <Metric label="Joint state rate" value={formatMaybe(status?.rates_hz.joint_states, 2)} />
          <Metric label="Joint count" value={String(status?.robot_state.joint_count ?? 0)} />
          <Metric label="EEF pose age" value={formatAge(status?.robot_state.eef_pose_age_s ?? -1)} />
          <Metric label="EEF pose rate" value={formatMaybe(status?.rates_hz.eef_pose, 2)} />
          {status?.robot_state.eef_pose ? (
            <>
              <Metric label="EEF frame" value={`${status.robot_state.command_frame} <- ${status.robot_state.eef_frame}`} />
              <Metric label="EEF xyz" value={`${status.robot_state.eef_pose.x.toFixed(3)}, ${status.robot_state.eef_pose.y.toFixed(3)}, ${status.robot_state.eef_pose.z.toFixed(3)} m`} />
              <Metric label="EEF quaternion" value={`${status.robot_state.eef_pose.qx.toFixed(3)}, ${status.robot_state.eef_pose.qy.toFixed(3)}, ${status.robot_state.eef_pose.qz.toFixed(3)}, ${status.robot_state.eef_pose.qw.toFixed(3)}`} />
            </>
          ) : (
            <div className="rounded-md border border-slate-800 bg-slate-900/60 p-3 text-slate-400">
              End-effector pose unavailable. Check TF frames and robot publisher.
            </div>
          )}
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Command and Latency Metrics</CardTitle>
          <CardDescription>Goal transport and response estimates</CardDescription>
        </CardHeader>
        <CardContent className="space-y-2 text-sm">
          <Metric label="Target pose age" value={formatAge(status?.robot_state.command_metrics.last_target_age_s ?? -1)} />
          <Metric label="Twist command age" value={formatAge(status?.robot_state.command_metrics.last_twist_age_s ?? -1)} />
          <Metric label="Clicked-point age" value={formatAge(status?.robot_state.command_metrics.last_clicked_point_age_s ?? -1)} />
          <Metric label="target_pose rate" value={formatMaybe(status?.rates_hz.target_pose, 2)} />
          <Metric label="twist_cmd rate" value={formatMaybe(status?.rates_hz.twist_cmd, 2)} />
          <Metric label="clicked_point rate" value={formatMaybe(status?.rates_hz.clicked_point, 2)} />
          <Metric label="Last target transport latency" value={`${formatMaybe(status?.robot_state.command_metrics.last_target_transport_latency_ms, 2)} ms`} />
          <Metric label="Average target transport latency" value={`${formatMaybe(status?.robot_state.command_metrics.avg_target_transport_latency_ms, 2)} ms`} />
          <Metric label="EEF response latency" value={`${formatMaybe(status?.robot_state.command_metrics.last_ee_response_latency_ms, 2)} ms`} />
          <Metric label="Pending goal" value={status?.robot_state.command_metrics.pending_goal ? 'True' : 'False'} />
          <Metric label="Motion threshold" value={`${formatMaybe(status?.robot_state.command_metrics.eef_motion_threshold_m, 3)} m`} />
        </CardContent>
      </Card>

      <Card className="xl:col-span-2">
        <CardHeader>
          <CardTitle>Joint Table</CardTitle>
          <CardDescription>Position / velocity / effort</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="overflow-x-auto rounded-lg border border-slate-800">
            <table className="w-full min-w-[720px] border-collapse text-sm">
              <thead className="bg-slate-900/80 text-slate-300">
                <tr>
                  <th className="border-b border-slate-800 px-3 py-2 text-left">Joint</th>
                  <th className="border-b border-slate-800 px-3 py-2 text-left">Position (rad)</th>
                  <th className="border-b border-slate-800 px-3 py-2 text-left">Velocity (rad/s)</th>
                  <th className="border-b border-slate-800 px-3 py-2 text-left">Effort</th>
                </tr>
              </thead>
              <tbody>
                {joints.length === 0 ? (
                  <tr>
                    <td className="px-3 py-3 text-slate-400" colSpan={4}>
                      No joint data received yet.
                    </td>
                  </tr>
                ) : (
                  joints.map((joint) => (
                    <tr key={joint.name} className="border-b border-slate-800/60">
                      <td className="px-3 py-2 font-mono text-xs text-slate-300">{joint.name}</td>
                      <td className="px-3 py-2 text-slate-200">{joint.position_rad === null ? 'n/a' : joint.position_rad.toFixed(4)}</td>
                      <td className="px-3 py-2 text-slate-200">{joint.velocity_rad_s === null ? 'n/a' : joint.velocity_rad_s.toFixed(4)}</td>
                      <td className="px-3 py-2 text-slate-200">{joint.effort === null ? 'n/a' : joint.effort.toFixed(4)}</td>
                    </tr>
                  ))
                )}
              </tbody>
            </table>
          </div>
        </CardContent>
      </Card>
    </div>
  );
}

function UnityTab({ status }: { status: RelayStatus | null }) {
  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-[1fr_1fr]">
      <Card>
        <CardHeader>
          <CardTitle>Unity Bridge Health</CardTitle>
          <CardDescription>ROS-TCP endpoint and map signal</CardDescription>
        </CardHeader>
        <CardContent className="space-y-2 text-sm">
          <Metric label="TCP host" value={status ? `${status.unity_integration.tcp_host}:${status.unity_integration.tcp_port}` : 'n/a'} />
          <Metric label="TCP endpoint reachable" value={status?.unity_integration.tcp_endpoint_reachable ? 'True' : 'False'} />
          <Metric label="Last probe age" value={formatAge(status?.unity_integration.last_probe_age_s ?? -1)} />
          <Metric label="Last success age" value={formatAge(status?.unity_integration.last_success_age_s ?? -1)} />
          <Metric label="Map signal received" value={status?.unity_integration.map_signal_received ? 'True' : 'False'} />
          <Metric label="Map loaded" value={status?.unity_integration.map_loaded === null ? 'Unknown' : status?.unity_integration.map_loaded ? 'True' : 'False'} />
          <Metric label="Map signal age" value={formatAge(status?.unity_integration.map_signal_age_s ?? -1)} />
          <Metric label="Map signal count" value={String(status?.unity_integration.map_signal_count ?? 0)} />
          <div className="rounded-md border border-slate-800 bg-slate-900/60 p-3 text-slate-300">
            {status?.unity_integration.message ?? 'No unity integration status available.'}
          </div>
        </CardContent>
      </Card>

      <Card>
        <CardHeader>
          <CardTitle>Digital Twin Alignment Signals</CardTitle>
          <CardDescription>Goal-to-motion timing indicators</CardDescription>
        </CardHeader>
        <CardContent className="space-y-2 text-sm">
          <Metric label="Target topic" value={status?.topics.target_pose_topic ?? 'n/a'} mono />
          <Metric label="Twist topic" value={status?.topics.twist_topic ?? 'n/a'} mono />
          <Metric label="Joint topic" value={status?.topics.joint_states_topic ?? 'n/a'} mono />
          <Metric label="Target transport latency" value={`${formatMaybe(status?.robot_state.command_metrics.last_target_transport_latency_ms, 2)} ms`} />
          <Metric label="EEF response latency" value={`${formatMaybe(status?.robot_state.command_metrics.last_ee_response_latency_ms, 2)} ms`} />
          <Metric label="target_pose rate" value={formatMaybe(status?.rates_hz.target_pose, 2)} />
          <Metric label="twist_cmd rate" value={formatMaybe(status?.rates_hz.twist_cmd, 2)} />
          <Metric label="eef_pose rate" value={formatMaybe(status?.rates_hz.eef_pose, 2)} />
          <Metric label="Joint state age" value={formatAge(status?.robot_state.joint_state_age_s ?? -1)} />
          <Metric label="EEF pose age" value={formatAge(status?.robot_state.eef_pose_age_s ?? -1)} />
        </CardContent>
      </Card>
    </div>
  );
}

function TabButton({
  label,
  active,
  onClick
}: {
  label: string;
  active: boolean;
  onClick: () => void;
}) {
  return (
    <Button variant={active ? 'default' : 'outline'} size="sm" onClick={onClick}>
      {label}
    </Button>
  );
}

function Metric({
  label,
  value,
  mono = false
}: {
  label: string;
  value: string;
  mono?: boolean;
}) {
  return (
    <div className="flex items-center justify-between gap-3 rounded-md border border-slate-800 bg-slate-900/50 px-3 py-2">
      <span className="text-slate-400">{label}</span>
      <span className={mono ? 'font-mono text-xs text-slate-200' : 'font-medium text-slate-200'}>{value}</span>
    </div>
  );
}
