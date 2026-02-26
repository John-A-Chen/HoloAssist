import { formatNumber } from '../../lib/format';
import { useRobotStore } from '../../store/useRobotStore';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';

export function JointStateWidget() {
  const jointTelemetry = useRobotStore((s) => s.jointTelemetry);

  return (
    <Card>
      <CardHeader>
        <div>
          <CardTitle>Joint state</CardTitle>
          <CardDescription>Positions and velocities (10 Hz mock)</CardDescription>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="space-y-3">
          {jointTelemetry.positions.map((position, index) => {
            const pct = ((position + Math.PI) / (2 * Math.PI)) * 100;
            return (
              <div key={`pos-${index}`} className="space-y-1">
                <div className="flex items-center justify-between text-xs text-slate-400">
                  <span>J{index + 1} position</span>
                  <span className="font-mono text-slate-200">{formatNumber(position, 3)} rad</span>
                </div>
                <input
                  type="range"
                  min={-Math.PI}
                  max={Math.PI}
                  step={0.001}
                  value={position}
                  readOnly
                  className="range-accent h-2 w-full cursor-default rounded-lg bg-slate-800"
                  style={{ backgroundSize: `${pct}% 100%` }}
                />
              </div>
            );
          })}
        </div>

        <div className="grid grid-cols-2 gap-2">
          {jointTelemetry.velocities.map((velocity, index) => (
            <div key={`vel-${index}`} className="rounded-md border border-slate-800 bg-slate-950/40 p-2">
              <div className="text-xs uppercase tracking-wider text-slate-500">J{index + 1} vel</div>
              <div className="font-mono text-sm text-slate-100">{formatNumber(velocity, 3)} rad/s</div>
            </div>
          ))}
        </div>
      </CardContent>
    </Card>
  );
}
