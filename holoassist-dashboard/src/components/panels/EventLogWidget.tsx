import { formatLogTime } from '../../lib/format';
import { useRobotStore } from '../../store/useRobotStore';
import type { LogLevel } from '../../types/robot';
import { Badge } from '../ui/badge';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '../ui/card';
import { Chip } from '../ui/chip';

const levels: LogLevel[] = ['INFO', 'WARN', 'ERROR'];

export function EventLogWidget() {
  const eventLogs = useRobotStore((s) => s.eventLogs);
  const logFilters = useRobotStore((s) => s.logFilters);
  const toggleLogFilter = useRobotStore((s) => s.toggleLogFilter);

  const filtered = eventLogs.filter((entry) => logFilters[entry.level]);

  return (
    <Card className="flex min-h-[20rem] flex-col">
      <CardHeader>
        <div>
          <CardTitle>Event log</CardTitle>
          <CardDescription>Actions, mode changes and mock system events</CardDescription>
        </div>
      </CardHeader>
      <CardContent className="flex min-h-0 flex-1 flex-col gap-3">
        <div className="flex flex-wrap gap-2">
          {levels.map((level) => (
            <Chip key={level} active={logFilters[level]} onClick={() => toggleLogFilter(level)}>
              {level}
            </Chip>
          ))}
        </div>

        <div className="scrollbar-thin min-h-0 flex-1 space-y-2 overflow-auto rounded-lg border border-slate-800 bg-slate-950/40 p-2">
          {filtered.length === 0 ? (
            <div className="p-4 text-sm text-slate-500">No log entries match the selected filters.</div>
          ) : (
            filtered.map((entry) => (
              <div key={entry.id} className="rounded-md border border-slate-800 bg-slate-900/60 p-2">
                <div className="mb-1 flex items-center justify-between gap-2">
                  <Badge tone={entry.level === 'INFO' ? 'info' : entry.level === 'WARN' ? 'warn' : 'error'}>
                    {entry.level}
                  </Badge>
                  <span className="font-mono text-xs text-slate-500">{formatLogTime(entry.timestamp)}</span>
                </div>
                <div className="text-sm text-slate-200">{entry.message}</div>
              </div>
            ))
          )}
        </div>
      </CardContent>
    </Card>
  );
}
