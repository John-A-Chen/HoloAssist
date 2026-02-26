import { useRobotStore } from '../../store/useRobotStore';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';
import { Slider } from '../ui/slider';
import { Switch } from '../ui/switch';

export function ServoPanel() {
  const connectionStatus = useRobotStore((s) => s.connectionStatus);
  const robotMode = useRobotStore((s) => s.robotMode);
  const servoEnabled = useRobotStore((s) => s.servoEnabled);
  const deadmanHeld = useRobotStore((s) => s.deadmanHeld);
  const commandStreamingEnabled = useRobotStore((s) => s.commandStreamingEnabled);
  const speedScalingPercent = useRobotStore((s) => s.speedScalingPercent);
  const setServoEnabled = useRobotStore((s) => s.setServoEnabled);
  const setDeadmanHeld = useRobotStore((s) => s.setDeadmanHeld);
  const setSpeedScalingPercent = useRobotStore((s) => s.setSpeedScalingPercent);
  const sendZeroCommand = useRobotStore((s) => s.sendZeroCommand);

  const disconnected = connectionStatus === 'Disconnected';
  const faulted = robotMode === 'Fault';
  const controlsDisabled = disconnected || faulted;

  return (
    <Card>
      <CardHeader>
        <div>
          <CardTitle>Servo</CardTitle>
          <CardDescription>Real-time jogging control with deadman gating</CardDescription>
        </div>
        <Badge tone={commandStreamingEnabled ? 'success' : deadmanHeld ? 'info' : 'neutral'}>
          {commandStreamingEnabled ? 'Streaming' : deadmanHeld ? 'Armed' : 'Standby'}
        </Badge>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="flex items-center justify-between rounded-lg border border-slate-800 bg-slate-950/40 p-3">
          <div>
            <div className="text-sm font-medium text-slate-200">Enable Servo</div>
            <div className="text-xs text-slate-400">Disabled in Fault mode</div>
          </div>
          <Switch
            checked={servoEnabled}
            disabled={controlsDisabled}
            ariaLabel="Enable servo"
            onCheckedChange={(checked) => void setServoEnabled(checked)}
          />
        </div>

        <div className="space-y-2 rounded-lg border border-slate-800 bg-slate-950/40 p-3">
          <div className="flex items-center justify-between text-sm">
            <span className="text-slate-300">Speed scaling</span>
            <span className="font-medium text-slate-100">{speedScalingPercent}%</span>
          </div>
          <Slider
            value={speedScalingPercent}
            disabled={controlsDisabled}
            onChange={(event) => setSpeedScalingPercent(Number(event.currentTarget.value))}
          />
        </div>

        <div className="space-y-2 rounded-lg border border-slate-800 bg-slate-950/40 p-3">
          <div className="flex items-center justify-between text-sm">
            <span className="text-slate-300">Deadman switch</span>
            <Badge tone={deadmanHeld ? 'success' : 'neutral'}>{deadmanHeld ? 'Held' : 'Released'}</Badge>
          </div>
          <Button
            variant={deadmanHeld ? 'success' : 'outline'}
            disabled={controlsDisabled || !servoEnabled}
            className="w-full"
            onPointerDown={() => setDeadmanHeld(true)}
            onPointerUp={() => setDeadmanHeld(false)}
            onPointerLeave={() => setDeadmanHeld(false)}
            onPointerCancel={() => setDeadmanHeld(false)}
          >
            Hold to Enable Command Streaming (Space)
          </Button>
          <div className="text-xs text-slate-500">Command streaming only stays active while held.</div>
        </div>

        <Button variant="outline" className="w-full" disabled={controlsDisabled} onClick={() => void sendZeroCommand()}>
          Send zero command
        </Button>
      </CardContent>
    </Card>
  );
}
