import { Line, LineChart, ResponsiveContainer } from 'recharts';

import type { LatencySample } from '../../types/robot';

interface LatencySparklineProps {
  samples: LatencySample[];
}

export function LatencySparkline({ samples }: LatencySparklineProps) {
  const data = samples.map((sample, index) => ({ idx: index, ms: sample.ms }));

  return (
    <div className="h-10 w-28">
      <ResponsiveContainer width="100%" height="100%">
        <LineChart data={data}>
          <Line
            type="monotone"
            dataKey="ms"
            stroke="#38bdf8"
            strokeWidth={2}
            dot={false}
            isAnimationActive={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
}
