import { useMemo } from 'react';

import { useRobotStore } from '../../store/useRobotStore';
import type { RobotMode } from '../../types/robot';
import { Badge } from '../ui/badge';
import { Button } from '../ui/button';
import { ConfirmModal } from '../ui/modal';
import { Select } from '../ui/select';
import { LatencySparkline } from '../visualisation/LatencySparkline';

const robotModes: RobotMode[] = ['Idle', 'Planning', 'Servoing', 'Fault'];

export function TopBar() {
  const connectionStatus = useRobotStore((s) => s.connectionStatus);
  const robotMode = useRobotStore((s) => s.robotMode);
  const latencyMs = useRobotStore((s) => s.latencyMs);
  const latencyHistory = useRobotStore((s) => s.latencyHistory);
  const eStopModalOpen = useRobotStore((s) => s.eStopModalOpen);
  const toggleConnection = useRobotStore((s) => s.toggleConnection);
  const setRobotMode = useRobotStore((s) => s.setRobotMode);
  const openEstopModal = useRobotStore((s) => s.openEstopModal);
  const closeEstopModal = useRobotStore((s) => s.closeEstopModal);
  const confirmEstop = useRobotStore((s) => s.confirmEstop);
  const resetFault = useRobotStore((s) => s.resetFault);

  const modeDisabled = connectionStatus === 'Disconnected' || robotMode === 'Fault';
  const estopDisabled = connectionStatus === 'Disconnected';
  const latencyTone = useMemo(() => {
    if (connectionStatus === 'Disconnected') {
      return 'neutral' as const;
    }
    if (latencyMs > 45) {
      return 'warn' as const;
    }
    return 'success' as const;
  }, [connectionStatus, latencyMs]);

  return (
    <>
      <header className="sticky top-0 z-30 flex flex-wrap items-center justify-between gap-4 border-b border-slate-800/90 bg-slate-950/90 px-4 py-3 backdrop-blur">
        <div className="flex min-w-0 items-center gap-3">
          <div>
            <div className="text-xs uppercase tracking-[0.18em] text-slate-500">RS2 HoloAssist</div>
            <div className="text-lg font-semibold text-slate-100">Robot Ops Dashboard</div>
          </div>
          <Button
            variant={connectionStatus === 'Connected' ? 'success' : 'outline'}
            size="sm"
            onClick={() => void toggleConnection()}
            className="min-w-32"
          >
            {connectionStatus === 'Connected' ? 'Disconnect' : 'Connect'}
          </Button>
          <Badge tone={connectionStatus === 'Connected' ? 'success' : 'error'}>
            {connectionStatus}
          </Badge>
        </div>

        <div className="flex flex-1 flex-wrap items-center justify-end gap-3">
          <div className="flex items-center gap-2 rounded-lg border border-slate-800 bg-slate-900/80 px-3 py-2">
            <label htmlFor="robot-mode" className="text-xs font-medium uppercase tracking-wider text-slate-400">
              Mode
            </label>
            <Select
              id="robot-mode"
              value={robotMode}
              disabled={modeDisabled}
              onChange={(event) => void setRobotMode(event.target.value as RobotMode)}
              className="h-8 w-36"
            >
              {robotModes.map((mode) => (
                <option key={mode} value={mode}>
                  {mode}
                </option>
              ))}
            </Select>
          </div>

          <div className="flex items-center gap-3 rounded-lg border border-slate-800 bg-slate-900/80 px-3 py-2">
            <div>
              <div className="text-[10px] uppercase tracking-wider text-slate-500">Latency</div>
              <div className="flex items-center gap-2">
                <span className="text-sm font-semibold text-slate-100">{latencyMs} ms</span>
                <Badge tone={latencyTone}>{latencyMs === 0 ? 'No link' : 'Live'}</Badge>
              </div>
            </div>
            <LatencySparkline samples={latencyHistory} />
          </div>

          {robotMode === 'Fault' ? (
            <Button variant="outline" onClick={() => void resetFault()}>
              Reset Fault (R)
            </Button>
          ) : null}

          <Button
            variant="destructive"
            size="lg"
            onClick={openEstopModal}
            disabled={estopDisabled}
            className="min-w-32 font-semibold"
          >
            E-Stop (E)
          </Button>
        </div>
      </header>

      <ConfirmModal
        open={eStopModalOpen}
        title="Trigger emergency stop?"
        description="This mock action latches Fault mode and disables servo controls until Reset Fault is pressed."
        confirmLabel="Trigger E-Stop"
        confirmVariant="destructive"
        onCancel={closeEstopModal}
        onConfirm={() => void confirmEstop()}
      />
    </>
  );
}
