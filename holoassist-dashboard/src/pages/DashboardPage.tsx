import { TopBar } from '../components/layout/TopBar';
import { DigitalTwinPanel } from '../components/panels/DigitalTwinPanel';
import { EventLogWidget } from '../components/panels/EventLogWidget';
import { JointStateWidget } from '../components/panels/JointStateWidget';
import { PlanningPanel } from '../components/panels/PlanningPanel';
import { SafetyPanel } from '../components/panels/SafetyPanel';
import { ServoPanel } from '../components/panels/ServoPanel';
import { ToolPoseWidget } from '../components/panels/ToolPoseWidget';
import { Badge } from '../components/ui/badge';
import { useDashboardPolling } from '../hooks/useDashboardPolling';
import { useKeyboardShortcuts } from '../hooks/useKeyboardShortcuts';
import { useRobotStore } from '../store/useRobotStore';

export function DashboardPage() {
  useDashboardPolling();
  useKeyboardShortcuts();

  const initialised = useRobotStore((s) => s.initialised);
  const loadingSnapshot = useRobotStore((s) => s.loadingSnapshot);
  const connectionStatus = useRobotStore((s) => s.connectionStatus);
  const robotMode = useRobotStore((s) => s.robotMode);
  const commandStreamingEnabled = useRobotStore((s) => s.commandStreamingEnabled);

  return (
    <div className="min-h-screen bg-slate-950 text-slate-100">
      <TopBar />

      <main className="mx-auto max-w-[1800px] px-4 py-4">
        <div className="mb-4 flex flex-wrap items-center gap-2">
          {loadingSnapshot && !initialised ? <Badge tone="info">Initialising mock data...</Badge> : null}
          <Badge tone={connectionStatus === 'Connected' ? 'success' : 'error'}>
            {connectionStatus === 'Connected' ? 'Link active' : 'Link inactive'}
          </Badge>
          <Badge tone={robotMode === 'Fault' ? 'error' : robotMode === 'Servoing' ? 'info' : 'neutral'}>
            Mode: {robotMode}
          </Badge>
          <Badge tone={commandStreamingEnabled ? 'success' : 'neutral'}>
            {commandStreamingEnabled ? 'Command streaming enabled' : 'Command streaming gated'}
          </Badge>
        </div>

        <div className="grid grid-cols-1 gap-4 xl:grid-cols-[360px_minmax(0,1fr)_400px]">
          <div className="space-y-4">
            <ServoPanel />
            <PlanningPanel />
            <SafetyPanel />
          </div>

          <div className="min-w-0">
            <DigitalTwinPanel />
          </div>

          <div className="space-y-4">
            <JointStateWidget />
            <ToolPoseWidget />
            <EventLogWidget />
          </div>
        </div>
      </main>
    </div>
  );
}
