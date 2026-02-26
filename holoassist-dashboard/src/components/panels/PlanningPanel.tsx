import { useRobotStore } from '../../store/useRobotStore';
import type { TargetPose } from '../../types/robot';
import { Button } from '../ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';
import { Input } from '../ui/input';

const poseFields: Array<{ key: keyof TargetPose; label: string; step: string }> = [
  { key: 'x', label: 'X', step: '0.001' },
  { key: 'y', label: 'Y', step: '0.001' },
  { key: 'z', label: 'Z', step: '0.001' },
  { key: 'roll', label: 'R', step: '0.01' },
  { key: 'pitch', label: 'P', step: '0.01' },
  { key: 'yaw', label: 'Yaw', step: '0.01' }
];

export function PlanningPanel() {
  const connectionStatus = useRobotStore((s) => s.connectionStatus);
  const robotMode = useRobotStore((s) => s.robotMode);
  const targetPose = useRobotStore((s) => s.targetPose);
  const planTarget = useRobotStore((s) => s.planTarget);
  const executePlan = useRobotStore((s) => s.executePlan);
  const updateTargetPoseField = useRobotStore((s) => s.updateTargetPoseField);
  const setTargetFromClickedPointPlaceholder = useRobotStore((s) => s.setTargetFromClickedPointPlaceholder);

  const disabled = connectionStatus === 'Disconnected' || robotMode === 'Fault';

  return (
    <Card>
      <CardHeader>
        <div>
          <CardTitle>Planning</CardTitle>
          <CardDescription>Mock MoveIt plan/execute workflow</CardDescription>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="grid grid-cols-2 gap-2">
          <Button disabled={disabled} onClick={() => void planTarget()}>
            Plan
          </Button>
          <Button variant="outline" disabled={disabled} onClick={() => void executePlan()}>
            Execute
          </Button>
        </div>

        <div className="rounded-lg border border-slate-800 bg-slate-950/40 p-3">
          <div className="mb-3 text-sm font-medium text-slate-200">Target pose</div>
          <div className="grid grid-cols-2 gap-2">
            {poseFields.map((field) => (
              <label key={`${field.key}-${field.label}`} className="space-y-1">
                <span className="text-xs uppercase tracking-wider text-slate-500">{field.label}</span>
                <Input
                  type="number"
                  step={field.step}
                  value={targetPose[field.key]}
                  disabled={disabled}
                  onChange={(event) => updateTargetPoseField(field.key, Number(event.currentTarget.value))}
                />
              </label>
            ))}
          </div>
        </div>

        <Button variant="outline" disabled={disabled} className="w-full" onClick={setTargetFromClickedPointPlaceholder}>
          Set target from clicked point (placeholder)
        </Button>
      </CardContent>
    </Card>
  );
}
