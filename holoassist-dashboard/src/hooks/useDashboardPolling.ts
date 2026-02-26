import { useEffect } from 'react';

import { useRobotStore } from '../store/useRobotStore';

export function useDashboardPolling(): void {
  const bootstrap = useRobotStore((state) => state.bootstrap);
  const pollTelemetry = useRobotStore((state) => state.pollTelemetry);

  useEffect(() => {
    void bootstrap();
  }, [bootstrap]);

  useEffect(() => {
    const id = window.setInterval(() => {
      void pollTelemetry();
    }, 100);

    return () => window.clearInterval(id);
  }, [pollTelemetry]);
}
