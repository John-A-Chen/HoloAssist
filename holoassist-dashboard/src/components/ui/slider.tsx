import type { InputHTMLAttributes } from 'react';

import { cn } from '../../lib/cn';

export function Slider(props: InputHTMLAttributes<HTMLInputElement>) {
  return (
    <input
      type="range"
      min={0}
      max={100}
      step={1}
      {...props}
      className={cn('range-accent h-2 w-full cursor-pointer rounded-lg bg-slate-800', props.className)}
    />
  );
}
