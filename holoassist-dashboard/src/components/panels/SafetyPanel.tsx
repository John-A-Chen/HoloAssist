import { useRobotStore } from '../../store/useRobotStore';
import { Badge } from '../ui/badge';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';
import { Switch } from '../ui/switch';

export function SafetyPanel() {
  const safetySettings = useRobotStore((s) => s.safetySettings);
  const warnings = useRobotStore((s) => s.warnings);
  const connectionStatus = useRobotStore((s) => s.connectionStatus);
  const robotMode = useRobotStore((s) => s.robotMode);
  const toggleSafetySetting = useRobotStore((s) => s.toggleSafetySetting);

  const disabled = connectionStatus === 'Disconnected' || robotMode === 'Fault';

  return (
    <Card>
      <CardHeader>
        <div>
          <CardTitle>Safety</CardTitle>
          <CardDescription>Software checks and mock warning feed</CardDescription>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="space-y-3 rounded-lg border border-slate-800 bg-slate-950/40 p-3">
          <ToggleRow
            label="Joint limits"
            checked={safetySettings.jointLimits}
            disabled={disabled}
            onChange={() => toggleSafetySetting('jointLimits')}
          />
          <ToggleRow
            label="Speed limits"
            checked={safetySettings.speedLimits}
            disabled={disabled}
            onChange={() => toggleSafetySetting('speedLimits')}
          />
          <ToggleRow
            label="Collision checking"
            checked={safetySettings.collisionChecking}
            disabled={disabled}
            onChange={() => toggleSafetySetting('collisionChecking')}
          />
        </div>

        <div className="space-y-2 rounded-lg border border-slate-800 bg-slate-950/40 p-3">
          <div className="text-sm font-medium text-slate-200">Status</div>
          <div className="space-y-2">
            {warnings.map((warning) => (
              <div key={warning.id} className="flex items-start justify-between gap-3 rounded-md border border-slate-800 bg-slate-900/50 p-2">
                <div>
                  <div className="text-xs font-medium uppercase tracking-wider text-slate-400">{warning.label}</div>
                  <div className="text-sm text-slate-200">{warning.message}</div>
                </div>
                <Badge tone={warning.severity === 'ok' ? 'success' : warning.severity === 'warn' ? 'warn' : 'error'}>
                  {warning.severity.toUpperCase()}
                </Badge>
              </div>
            ))}
          </div>
        </div>
      </CardContent>
    </Card>
  );
}

function ToggleRow({
  label,
  checked,
  disabled,
  onChange
}: {
  label: string;
  checked: boolean;
  disabled: boolean;
  onChange: () => void;
}) {
  return (
    <div className="flex items-center justify-between gap-3">
      <div className="text-sm text-slate-200">{label}</div>
      <Switch checked={checked} disabled={disabled} onCheckedChange={onChange} ariaLabel={label} />
    </div>
  );
}
