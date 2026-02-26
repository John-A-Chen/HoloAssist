import type { ButtonHTMLAttributes } from 'react';

import { cn } from '../../lib/cn';

interface ChipProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  active?: boolean;
}

export function Chip({ active = false, className, ...props }: ChipProps) {
  return (
    <button
      type="button"
      className={cn(
        'inline-flex h-7 items-center rounded-full border px-3 text-xs font-medium transition-colours',
        active
          ? 'border-sky-600 bg-sky-900/30 text-sky-300'
          : 'border-slate-700 bg-slate-900 text-slate-300 hover:border-slate-500',
        className
      )}
      {...props}
    />
  );
}
