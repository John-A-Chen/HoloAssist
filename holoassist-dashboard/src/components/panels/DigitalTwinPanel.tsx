import { useMemo } from 'react';

import { formatNumber } from '../../lib/format';
import { useRobotStore } from '../../store/useRobotStore';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';

export function DigitalTwinPanel() {
  const toolPose = useRobotStore((s) => s.toolPose);
  const jointTelemetry = useRobotStore((s) => s.jointTelemetry);
  const robotMode = useRobotStore((s) => s.robotMode);
  const digitalTwinFullscreen = useRobotStore((s) => s.digitalTwinFullscreen);
  const setDigitalTwinFullscreen = useRobotStore((s) => s.setDigitalTwinFullscreen);

  const miniMapPoint = useMemo(() => {
    const x = ((toolPose.x - 0.15) / (0.5 - 0.15)) * 100;
    const y = ((toolPose.y + 0.35) / 0.7) * 100;
    return { x: Math.max(0, Math.min(100, x)), y: Math.max(0, Math.min(100, y)) };
  }, [toolPose.x, toolPose.y]);

  return (
    <>
      {digitalTwinFullscreen ? (
        <div className="fixed inset-0 z-40 bg-slate-950/70 backdrop-blur-sm" onClick={() => setDigitalTwinFullscreen(false)} />
      ) : null}
      <Card className={digitalTwinFullscreen ? 'fixed inset-4 z-50 flex flex-col' : 'h-full min-h-[32rem]'}>
        <CardHeader className="pb-3">
          <div>
            <CardTitle>Digital Twin View</CardTitle>
            <CardDescription>Placeholder visualisation panel (3D slot ready)</CardDescription>
          </div>
          <div className="flex items-center gap-2">
            <Badge tone={robotMode === 'Fault' ? 'error' : robotMode === 'Servoing' ? 'info' : 'neutral'}>{robotMode}</Badge>
            <Button
              variant="outline"
              size="sm"
              onClick={() => setDigitalTwinFullscreen(!digitalTwinFullscreen)}
            >
              {digitalTwinFullscreen ? 'Exit full screen' : 'Full screen'}
            </Button>
          </div>
        </CardHeader>

        <CardContent className="grid flex-1 grid-cols-1 gap-4 lg:grid-cols-[1.7fr_1fr]">
          <div className="relative flex min-h-[20rem] flex-col rounded-xl border border-slate-800 bg-slate-950/60 p-4 panel-grid">
            <div className="absolute inset-x-0 top-0 h-24 bg-gradient-to-b from-sky-500/10 to-transparent" />
            <div className="relative z-10 mb-4 flex items-center justify-between">
              <div>
                <div className="text-xs uppercase tracking-wider text-slate-500">Visualisation slot</div>
                <div className="text-sm font-medium text-slate-200">Digital twin placeholder (no ROS / no 3D engine yet)</div>
              </div>
              <div className="rounded-lg border border-slate-700 bg-slate-900/70 px-3 py-2 text-xs text-slate-300">
                Scene FPS (mock): 60
              </div>
            </div>

            <div className="relative z-10 mt-auto grid gap-3 sm:grid-cols-2">
              <div className="rounded-lg border border-slate-800 bg-slate-900/60 p-3">
                <div className="mb-2 text-xs uppercase tracking-wider text-slate-500">Tool pose</div>
                <div className="grid grid-cols-2 gap-x-4 gap-y-1 text-sm">
                  <PoseValue label="X" value={toolPose.x} />
                  <PoseValue label="Y" value={toolPose.y} />
                  <PoseValue label="Z" value={toolPose.z} />
                  <PoseValue label="Roll" value={toolPose.roll} />
                  <PoseValue label="Pitch" value={toolPose.pitch} />
                  <PoseValue label="Yaw" value={toolPose.yaw} />
                </div>
              </div>

              <div className="rounded-lg border border-slate-800 bg-slate-900/60 p-3">
                <div className="mb-2 text-xs uppercase tracking-wider text-slate-500">Mini-map (top view)</div>
                <div className="relative h-28 overflow-hidden rounded-md border border-slate-800 mini-grid bg-slate-950/60">
                  <div className="absolute left-1/2 top-0 bottom-0 w-px bg-slate-700/80" />
                  <div className="absolute top-1/2 left-0 right-0 h-px bg-slate-700/80" />
                  <div
                    className="absolute h-3 w-3 -translate-x-1/2 -translate-y-1/2 rounded-full border border-sky-200 bg-sky-400 shadow-[0_0_20px_rgba(56,189,248,0.45)]"
                    style={{ left: `${miniMapPoint.x}%`, top: `${miniMapPoint.y}%` }}
                  />
                </div>
                <div className="mt-2 text-xs text-slate-500">Workspace projection placeholder for tool motion preview</div>
              </div>
            </div>
          </div>

          <div className="rounded-xl border border-slate-800 bg-slate-950/40 p-4">
            <div className="mb-3 flex items-center justify-between">
              <div className="text-sm font-medium text-slate-200">Joint activity bars</div>
              <div className="text-xs text-slate-500">Read-only mock</div>
            </div>
            <div className="space-y-3">
              {jointTelemetry.positions.map((position, index) => {
                const pct = ((position + Math.PI) / (2 * Math.PI)) * 100;
                return (
                  <div key={`joint-bar-${index}`} className="space-y-1">
                    <div className="flex items-center justify-between text-xs text-slate-400">
                      <span>J{index + 1}</span>
                      <span>{formatNumber(position, 2)} rad</span>
                    </div>
                    <div className="h-2 overflow-hidden rounded-full bg-slate-800">
                      <div className="h-full rounded-full bg-sky-400/80" style={{ width: `${pct}%` }} />
                    </div>
                  </div>
                );
              })}
            </div>
          </div>
        </CardContent>
      </Card>
    </>
  );
}

function PoseValue({ label, value }: { label: string; value: number }) {
  return (
    <div className="flex items-center justify-between gap-3">
      <span className="text-slate-500">{label}</span>
      <span className="font-mono text-slate-200">{formatNumber(value, 3)}</span>
    </div>
  );
}
