import type { SelectHTMLAttributes } from 'react';

import { cn } from '../../lib/cn';

export function Select(props: SelectHTMLAttributes<HTMLSelectElement>) {
  return (
    <select
      {...props}
      className={cn(
        'h-9 w-full rounded-md border border-slate-700 bg-slate-950/80 px-3 text-sm text-slate-100 outline-none focus:border-sky-500',
        props.className
      )}
    />
  );
}
