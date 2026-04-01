import { useEffect, useMemo, useState } from 'react';
import {
  CartesianGrid,
  Legend,
  Line,
  LineChart,
  ReferenceArea,
  ReferenceLine,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis
} from 'recharts';

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

interface RateHistorySample {
  ts_unix_ms: number;
  bbox_hz: number;
  obstacle_hz: number;
  debug_age_s: number;
  point_count: number;
}

type BadgeTone = 'neutral' | 'success' | 'warn' | 'error' | 'info';
type TabId = 'overview' | 'perception' | 'robot' | 'unity';

const API_BASE = import.meta.env.VITE_PERCEPTION_API_BASE ?? '';
const STATUS_POLL_MS = 300;
const IMAGE_REFRESH_MS = 250;
const RATE_HISTORY_WINDOW_MS = 60_000;

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

function formatClockLabel(unixMs: number): string {
  if (!Number.isFinite(unixMs) || unixMs <= 0) {
    return 'n/a';
  }
  const date = new Date(unixMs);
  const hh = String(date.getHours()).padStart(2, '0');
  const mm = String(date.getMinutes()).padStart(2, '0');
  const ss = String(date.getSeconds()).padStart(2, '0');
  return `${hh}:${mm}:${ss}`;
}

function trimRateHistory(samples: RateHistorySample[], nowMs: number): RateHistorySample[] {
  const cutoff = nowMs - RATE_HISTORY_WINDOW_MS;
  return samples.filter((sample) => sample.ts_unix_ms >= cutoff);
}

function formatPoints(value: number): string {
  if (!Number.isFinite(value)) {
    return '0';
  }
  return Math.round(value).toLocaleString();
}

export function PerceptionMonitorPage() {
  const [status, setStatus] = useState<RelayStatus | null>(null);
  const [bridgeOnline, setBridgeOnline] = useState(false);
  const [errorText, setErrorText] = useState<string | null>(null);
  const [imageTick, setImageTick] = useState(0);
  const [activeTab, setActiveTab] = useState<TabId>('overview');
  const [rateHistory, setRateHistory] = useState<RateHistorySample[]>([]);

  useEffect(() => {
    let cancelled = false;
    let polling = false;

    const pollStatus = async () => {
      if (polling) {
        return;
      }
      polling = true;

      try {
        const response = await fetch(apiPath('/api/perception/status'), { cache: 'no-store' });
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
        setRateHistory((previous) => {
          const nowMs = Number.isFinite(payload.server_time_unix_ms) ? payload.server_time_unix_ms : Date.now();
          const sample: RateHistorySample = {
            ts_unix_ms: nowMs,
            bbox_hz: Number(payload.rates_hz.bbox ?? 0),
            obstacle_hz: Number(payload.rates_hz.obstacle ?? 0),
            debug_age_s: Number(payload.tracker_connection.last_debug_age_s ?? -1),
            point_count: Number(payload.counters.last_point_count ?? 0)
          };

          const deduped =
            previous.length > 0 && previous[previous.length - 1].ts_unix_ms === sample.ts_unix_ms
              ? [...previous.slice(0, -1), sample]
              : [...previous, sample];
          return trimRateHistory(deduped, nowMs);
        });
      } catch (error) {
        if (cancelled) {
          return;
        }
        setBridgeOnline(false);
        setErrorText(error instanceof Error ? error.message : String(error));
        setRateHistory((previous) => trimRateHistory(previous, Date.now()));
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

  const imageUrl = useMemo(() => `${apiPath('/api/perception/debug.jpg')}?tick=${imageTick}`, [imageTick]);

  const sourceMode = status?.source.mode ?? 'no_data';
  const pipelineConnected = status?.tracker_connection.pipeline_connected ?? false;
  const robotConnected = status?.robot_state.joint_state_available ?? false;
  const unityReachable = status?.unity_integration.tcp_endpoint_reachable ?? false;
  const staleTimeout = status?.bridge.stale_timeout_s ?? 1.5;
  const historyWindowSeconds = Math.round(RATE_HISTORY_WINDOW_MS / 1000);
  const sourceLabel =
    sourceMode === 'depth_tracker' ? 'RealSense depth mode' : sourceMode === 'rgb_fallback' ? 'RGB fallback mode' : 'No stream';

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100">
      <main className="mx-auto max-w-[1800px] px-5 py-6">
        <div className="mb-4 rounded-2xl border border-slate-800 bg-slate-900/70 p-5 shadow-panel backdrop-blur-sm">
          <div className="flex flex-wrap items-start justify-between gap-4">
            <div>
              <h1 className="text-3xl font-semibold tracking-tight text-slate-100">HoloAssist Evidence Dashboard</h1>
              <p className="mt-1 text-base text-slate-300">Perception, robot telemetry, and bridge health for presentation capture</p>
            </div>
            <div className="rounded-xl border border-slate-700/80 bg-slate-950/70 px-4 py-3 text-right">
              <div className="text-xs uppercase tracking-wide text-slate-400">Logging window</div>
              <div className="text-lg font-semibold text-slate-100">{historyWindowSeconds}s rolling</div>
              <div className="mt-1 text-xs text-slate-400">Last update: {status ? formatClockLabel(status.server_time_unix_ms) : 'n/a'}</div>
            </div>
          </div>

          <div className="mt-4 flex flex-wrap gap-2">
            <StatusBadge label="Relay" value={bridgeOnline ? 'Reachable' : 'Unreachable'} tone={bridgeOnline ? 'success' : 'error'} />
            <StatusBadge
              label="Tracker"
              value={pipelineConnected ? 'Live' : 'Stale'}
              tone={pipelineConnected ? 'success' : 'warn'}
            />
            <StatusBadge
              label="Robot State"
              value={robotConnected ? 'Live' : 'Stale'}
              tone={robotConnected ? 'success' : 'warn'}
            />
            <StatusBadge
              label="Unity Bridge"
              value={unityReachable ? 'Reachable' : 'Unreachable'}
              tone={unityReachable ? 'success' : 'warn'}
            />
            <StatusBadge
              label="Camera Source"
              value={sourceLabel}
              tone={sourceMode === 'depth_tracker' ? 'info' : sourceMode === 'rgb_fallback' ? 'warn' : 'neutral'}
            />
            <StatusBadge
              label="Obstacle"
              value={status?.latest.obstacle_active ? 'Active' : 'Inactive'}
              tone={status?.latest.obstacle_active ? 'success' : 'neutral'}
            />
          </div>

          <div className="mt-4 grid grid-cols-1 gap-3 sm:grid-cols-2 xl:grid-cols-4">
            <EvidenceStat
              label="BBox Publish Rate"
              value={`${formatMaybe(status?.rates_hz.bbox, 2)} Hz`}
              note="Primary evidence metric"
            />
            <EvidenceStat
              label="Obstacle Marker Rate"
              value={`${formatMaybe(status?.rates_hz.obstacle, 2)} Hz`}
              note="Target observed around 4.0-4.5 Hz"
            />
            <EvidenceStat
              label="Debug Frame Age"
              value={formatAge(status?.tracker_connection.last_debug_age_s ?? -1)}
              note={`Stale threshold ${staleTimeout.toFixed(2)} s`}
            />
            <EvidenceStat
              label="Last Point Count"
              value={formatPoints(status?.counters.last_point_count ?? 0)}
              note="Context only; not primary rate evidence"
            />
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

        {bridgeOnline && sourceMode !== 'depth_tracker' ? (
          <Card className="mb-4 border-amber-900/70 bg-amber-950/20">
            <CardContent className="p-4 text-sm text-amber-200">
              {status?.capabilities.message || 'Depth source unavailable, pointcloud and boundary features are disabled.'}
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

        {activeTab === 'overview' ? <OverviewTab status={status} imageUrl={imageUrl} /> : null}
        {activeTab === 'perception' ? (
          <PerceptionTab status={status} imageUrl={imageUrl} rateHistory={rateHistory} staleTimeout={staleTimeout} />
        ) : null}
        {activeTab === 'robot' ? <RobotTab status={status} /> : null}
        {activeTab === 'unity' ? <UnityTab status={status} /> : null}
      </main>
    </div>
  );
}

function OverviewTab({ status, imageUrl }: { status: RelayStatus | null; imageUrl: string }) {
  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-[1.35fr_1fr]">
      <Card>
        <CardHeader>
          <CardTitle className="text-lg">Live Camera Feed</CardTitle>
          <CardDescription className="text-sm">Active topic: {status?.source.active_image_topic || 'n/a'}</CardDescription>
        </CardHeader>
        <CardContent>
          <div className="relative overflow-hidden rounded-lg border border-slate-800 bg-slate-950/80">
            {status?.latest.jpeg_available ? (
              <img src={imageUrl} alt="HoloAssist camera stream" className="h-auto min-h-[26rem] w-full object-contain" />
            ) : (
              <div className="flex min-h-[24rem] items-center justify-center text-sm text-slate-400">Waiting for image stream...</div>
            )}
          </div>
        </CardContent>
      </Card>

      <div className="space-y-4">
        <Card>
          <CardHeader>
            <CardTitle className="text-lg">Subsystem Summary</CardTitle>
            <CardDescription className="text-sm">Current operating mode and evidence snapshot</CardDescription>
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
            <CardTitle className="text-lg">Presentation Rates (Hz)</CardTitle>
            <CardDescription className="text-sm">Measured over relay rolling window</CardDescription>
          </CardHeader>
          <CardContent className="space-y-2 text-sm">
            <Metric label="bbox (primary)" value={formatMaybe(status?.rates_hz.bbox, 2)} />
            <Metric label="obstacle (primary)" value={formatMaybe(status?.rates_hz.obstacle, 2)} />
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

function PerceptionTab({
  status,
  imageUrl,
  rateHistory,
  staleTimeout
}: {
  status: RelayStatus | null;
  imageUrl: string;
  rateHistory: RateHistorySample[];
  staleTimeout: number;
}) {
  const pointcloudSupported = status?.capabilities.pointcloud_supported ?? false;

  return (
    <div className="space-y-4">
      <div className="grid grid-cols-1 gap-4 xl:grid-cols-[1.5fr_1fr]">
        <Card>
          <CardHeader>
            <div>
              <CardTitle className="text-lg">Perception Stream</CardTitle>
              <CardDescription className="text-sm">Image source: {status?.source.active_image_topic || 'n/a'}</CardDescription>
            </div>
            <Badge tone={status?.latest.jpeg_available ? 'success' : 'warn'} className="px-3 py-1 text-sm">
              {status?.latest.jpeg_available ? 'Streaming' : 'Waiting'}
            </Badge>
          </CardHeader>
          <CardContent>
            <div className="relative overflow-hidden rounded-lg border border-slate-800 bg-slate-950/80">
              {status?.latest.jpeg_available ? (
                <img src={imageUrl} alt="Perception stream" className="h-auto min-h-[26rem] w-full object-contain" />
              ) : (
                <div className="flex min-h-[24rem] items-center justify-center text-sm text-slate-400">Waiting for image stream...</div>
              )}
            </div>
          </CardContent>
        </Card>

        <div className="space-y-4">
          <Card>
            <CardHeader>
              <CardTitle className="text-lg">Perception Health</CardTitle>
              <CardDescription className="text-sm">Freshness and counters</CardDescription>
            </CardHeader>
            <CardContent className="space-y-2 text-sm">
              <Metric label="Debug frame age" value={formatAge(status?.tracker_connection.last_debug_age_s ?? -1)} />
              <Metric label="RGB frame age" value={formatAge(status?.tracker_connection.last_rgb_age_s ?? -1)} />
              <Metric label="BBox age" value={formatAge(status?.tracker_connection.last_bbox_age_s ?? -1)} />
              <Metric label="Pointcloud age" value={formatAge(status?.tracker_connection.last_pointcloud_age_s ?? -1)} />
              <Metric label="Obstacle age" value={formatAge(status?.tracker_connection.last_obstacle_age_s ?? -1)} />
              <Metric label="bbox rate (primary)" value={formatMaybe(status?.rates_hz.bbox, 2)} />
              <Metric label="obstacle rate (primary)" value={formatMaybe(status?.rates_hz.obstacle, 2)} />
              <Metric label="bbox count" value={String(status?.counters.bbox_rx_count ?? 0)} />
              <Metric label="obstacle count" value={String(status?.counters.obstacle_rx_count ?? 0)} />
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
              <CardTitle className="text-lg">Current BBox</CardTitle>
              <CardDescription className="text-sm">Topic: {status?.topics.bbox_topic ?? 'n/a'}</CardDescription>
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

      <PerceptionRateTrendCard samples={rateHistory} />
      <PerceptionFreshnessCard samples={rateHistory} staleTimeout={staleTimeout} />
    </div>
  );
}

function PerceptionRateTrendCard({ samples }: { samples: RateHistorySample[] }) {
  const enoughData = samples.length >= 2;

  return (
    <Card>
      <CardHeader>
        <div>
          <CardTitle className="text-lg">Perception Output Rates - Rolling 60s</CardTitle>
          <CardDescription className="text-sm">
            Time-series log of bbox and obstacle marker publish rates for presentation evidence
          </CardDescription>
        </div>
        <Badge tone="info" className="px-3 py-1 text-sm">
          BBox and obstacle rates
        </Badge>
      </CardHeader>
      <CardContent>
        {!enoughData ? (
          <EmptyChartState text="Collecting rate history. Wait a few seconds for the graph to populate." />
        ) : (
          <div className="h-[22rem] w-full rounded-lg border border-slate-800 bg-slate-950/70 p-2">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={samples} margin={{ top: 18, right: 28, bottom: 22, left: 16 }}>
                <CartesianGrid stroke="rgba(148,163,184,0.2)" strokeDasharray="4 4" />
                <XAxis
                  dataKey="ts_unix_ms"
                  type="number"
                  domain={['dataMin', 'dataMax']}
                  tickFormatter={(value: number) => formatClockLabel(value)}
                  tick={{ fill: '#cbd5e1', fontSize: 12 }}
                  stroke="#94a3b8"
                  label={{ value: 'Time (hh:mm:ss)', position: 'insideBottom', offset: -12, fill: '#94a3b8' }}
                />
                <YAxis
                  yAxisId="rate"
                  tick={{ fill: '#cbd5e1', fontSize: 12 }}
                  stroke="#94a3b8"
                  width={52}
                  label={{ value: 'Rate (Hz)', angle: -90, position: 'insideLeft', fill: '#94a3b8' }}
                />
                <Tooltip
                  contentStyle={{
                    backgroundColor: 'rgba(15,23,42,0.95)',
                    border: '1px solid rgba(71,85,105,0.8)',
                    borderRadius: '0.75rem',
                    color: '#e2e8f0'
                  }}
                  labelFormatter={(value: number | string) => `Time ${formatClockLabel(Number(value))}`}
                  formatter={(value: number | string, name: string) => [`${Number(value).toFixed(2)} Hz`, name]}
                />
                <Legend
                  verticalAlign="top"
                  wrapperStyle={{
                    color: '#cbd5e1',
                    fontSize: '12px',
                    paddingBottom: '8px'
                  }}
                />
                <ReferenceArea yAxisId="rate" y1={4.0} y2={4.6} fill="rgba(34,197,94,0.13)" />
                <ReferenceLine yAxisId="rate" y={4.0} stroke="#22c55e" strokeDasharray="6 6" />
                <ReferenceLine yAxisId="rate" y={4.6} stroke="#22c55e" strokeDasharray="6 6" />
                <Line
                  yAxisId="rate"
                  type="monotone"
                  dataKey="bbox_hz"
                  name="BBox publish rate"
                  stroke="#38bdf8"
                  strokeWidth={3}
                  dot={false}
                  isAnimationActive={false}
                />
                <Line
                  yAxisId="rate"
                  type="monotone"
                  dataKey="obstacle_hz"
                  name="Obstacle marker rate"
                  stroke="#f97316"
                  strokeWidth={3}
                  dot={false}
                  isAnimationActive={false}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        )}
      </CardContent>
    </Card>
  );
}

function PerceptionFreshnessCard({ samples, staleTimeout }: { samples: RateHistorySample[]; staleTimeout: number }) {
  const enoughData = samples.length >= 2;

  return (
    <Card>
      <CardHeader>
        <div>
          <CardTitle className="text-lg">Freshness and Point Count - Rolling 60s</CardTitle>
          <CardDescription className="text-sm">Debug frame freshness and point density context</CardDescription>
        </div>
      </CardHeader>
      <CardContent>
        {!enoughData ? (
          <EmptyChartState text="Collecting freshness history. Keep the tracker running to build the timeline." />
        ) : (
          <div className="h-[20rem] w-full rounded-lg border border-slate-800 bg-slate-950/70 p-2">
            <ResponsiveContainer width="100%" height="100%">
              <LineChart data={samples} margin={{ top: 18, right: 28, bottom: 22, left: 16 }}>
                <CartesianGrid stroke="rgba(148,163,184,0.2)" strokeDasharray="4 4" />
                <XAxis
                  dataKey="ts_unix_ms"
                  type="number"
                  domain={['dataMin', 'dataMax']}
                  tickFormatter={(value: number) => formatClockLabel(value)}
                  tick={{ fill: '#cbd5e1', fontSize: 12 }}
                  stroke="#94a3b8"
                  label={{ value: 'Time (hh:mm:ss)', position: 'insideBottom', offset: -12, fill: '#94a3b8' }}
                />
                <YAxis
                  yAxisId="age"
                  tick={{ fill: '#cbd5e1', fontSize: 12 }}
                  stroke="#94a3b8"
                  width={56}
                  label={{ value: 'Age (s)', angle: -90, position: 'insideLeft', fill: '#94a3b8' }}
                />
                <YAxis
                  yAxisId="points"
                  orientation="right"
                  tick={{ fill: '#cbd5e1', fontSize: 12 }}
                  stroke="#94a3b8"
                  width={70}
                  label={{ value: 'Points', angle: 90, position: 'insideRight', fill: '#94a3b8' }}
                />
                <Tooltip
                  contentStyle={{
                    backgroundColor: 'rgba(15,23,42,0.95)',
                    border: '1px solid rgba(71,85,105,0.8)',
                    borderRadius: '0.75rem',
                    color: '#e2e8f0'
                  }}
                  labelFormatter={(value: number | string) => `Time ${formatClockLabel(Number(value))}`}
                  formatter={(value: number | string, name: string) => {
                    if (name === 'Last point count') {
                      return [formatPoints(Number(value)), name];
                    }
                    return [`${Number(value).toFixed(2)} s`, name];
                  }}
                />
                <Legend
                  verticalAlign="top"
                  wrapperStyle={{
                    color: '#cbd5e1',
                    fontSize: '12px',
                    paddingBottom: '8px'
                  }}
                />
                <ReferenceLine
                  yAxisId="age"
                  y={staleTimeout}
                  stroke="#f59e0b"
                  strokeDasharray="6 6"
                  label={{ value: 'Stale threshold', fill: '#fbbf24', fontSize: 11 }}
                />
                <Line
                  yAxisId="age"
                  type="monotone"
                  dataKey="debug_age_s"
                  name="Debug frame age"
                  stroke="#f59e0b"
                  strokeWidth={2.5}
                  dot={false}
                  isAnimationActive={false}
                />
                <Line
                  yAxisId="points"
                  type="monotone"
                  dataKey="point_count"
                  name="Last point count"
                  stroke="#a78bfa"
                  strokeWidth={2.5}
                  dot={false}
                  isAnimationActive={false}
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        )}
      </CardContent>
    </Card>
  );
}

function EmptyChartState({ text }: { text: string }) {
  return (
    <div className="flex h-[16rem] items-center justify-center rounded-lg border border-slate-800 bg-slate-950/60 text-sm text-slate-400">
      {text}
    </div>
  );
}

function RobotTab({ status }: { status: RelayStatus | null }) {
  const joints = status?.robot_state.joints ?? [];

  return (
    <div className="grid grid-cols-1 gap-4 xl:grid-cols-[1fr_1fr]">
      <Card>
        <CardHeader>
          <CardTitle className="text-lg">Robot State</CardTitle>
          <CardDescription className="text-sm">Joint telemetry and end-effector pose</CardDescription>
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
          <CardTitle className="text-lg">Command and Latency Metrics</CardTitle>
          <CardDescription className="text-sm">Goal transport and response estimates</CardDescription>
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
          <CardTitle className="text-lg">Joint Table</CardTitle>
          <CardDescription className="text-sm">Position / velocity / effort</CardDescription>
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
          <CardTitle className="text-lg">Unity Bridge Health</CardTitle>
          <CardDescription className="text-sm">ROS-TCP endpoint and map signal</CardDescription>
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
          <CardTitle className="text-lg">Digital Twin Alignment Signals</CardTitle>
          <CardDescription className="text-sm">Goal-to-motion timing indicators</CardDescription>
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
    <Button variant={active ? 'default' : 'outline'} size="sm" onClick={onClick} className="min-w-[8.5rem]">
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
    <div className="flex items-center justify-between gap-3 rounded-md border border-slate-800 bg-slate-900/50 px-3 py-2.5">
      <span className="text-sm text-slate-300">{label}</span>
      <span className={mono ? 'font-mono text-sm text-slate-100' : 'text-sm font-semibold text-slate-100'}>{value}</span>
    </div>
  );
}

function StatusBadge({ label, value, tone }: { label: string; value: string; tone: BadgeTone }) {
  const dotColorByTone: Record<BadgeTone, string> = {
    neutral: 'bg-slate-400',
    success: 'bg-emerald-400',
    warn: 'bg-amber-400',
    error: 'bg-red-400',
    info: 'bg-sky-400'
  };

  return (
    <Badge tone={tone} className="px-3 py-1 text-sm">
      <span className={`mr-2 inline-block h-2 w-2 rounded-full ${dotColorByTone[tone]}`} />
      <span className="font-semibold">{label}:</span>
      <span className="ml-1">{value}</span>
    </Badge>
  );
}

function EvidenceStat({ label, value, note }: { label: string; value: string; note: string }) {
  return (
    <div className="rounded-xl border border-slate-700/80 bg-slate-950/75 p-3">
      <div className="text-xs uppercase tracking-wide text-slate-400">{label}</div>
      <div className="mt-1 text-2xl font-semibold text-slate-100">{value}</div>
      <div className="mt-1 text-xs text-slate-400">{note}</div>
    </div>
  );
}
