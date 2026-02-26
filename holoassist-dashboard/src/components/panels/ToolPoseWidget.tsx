import { formatNumber } from '../../lib/format';
import { useRobotStore } from '../../store/useRobotStore';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';

export function ToolPoseWidget() {
  const toolPose = useRobotStore((s) => s.toolPose);

  return (
    <Card>
      <CardHeader>
        <div>
          <CardTitle>Tool pose</CardTitle>
          <CardDescription>Position and orientation display (10 Hz mock)</CardDescription>
        </div>
      </CardHeader>
      <CardContent className="grid grid-cols-2 gap-2">
        <PoseCell label="X" unit="m" value={toolPose.x} />
        <PoseCell label="Y" unit="m" value={toolPose.y} />
        <PoseCell label="Z" unit="m" value={toolPose.z} />
        <PoseCell label="Roll" unit="rad" value={toolPose.roll} />
        <PoseCell label="Pitch" unit="rad" value={toolPose.pitch} />
        <PoseCell label="Yaw" unit="rad" value={toolPose.yaw} />
      </CardContent>
    </Card>
  );
}

function PoseCell({ label, value, unit }: { label: string; value: number; unit: string }) {
  return (
    <div className="rounded-md border border-slate-800 bg-slate-950/40 p-2">
      <div className="text-xs uppercase tracking-wider text-slate-500">{label}</div>
      <div className="font-mono text-sm text-slate-100">{formatNumber(value, 3)} {unit}</div>
    </div>
  );
}
