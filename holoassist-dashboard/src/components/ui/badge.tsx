import type { HTMLAttributes } from 'react';

import { cn } from '../../lib/cn';

type BadgeTone = 'neutral' | 'success' | 'warn' | 'error' | 'info';

const toneMap: Record<BadgeTone, string> = {
  neutral: 'border-slate-700 bg-slate-800 text-slate-200',
  success: 'border-emerald-700/50 bg-emerald-900/30 text-emerald-300',
  warn: 'border-amber-700/50 bg-amber-900/20 text-amber-300',
  error: 'border-red-700/50 bg-red-900/20 text-red-300',
  info: 'border-sky-700/50 bg-sky-900/20 text-sky-300'
};

interface BadgeProps extends HTMLAttributes<HTMLSpanElement> {
  tone?: BadgeTone;
}

export function Badge({ className, children, tone = 'neutral', ...props }: BadgeProps) {
  return (
    <span
      className={cn('inline-flex items-center rounded-full border px-2 py-0.5 text-xs font-medium', toneMap[tone], className)}
      {...props}
    >
      {children}
    </span>
  );
}
