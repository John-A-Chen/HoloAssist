import type { InputHTMLAttributes } from 'react';

import { cn } from '../../lib/cn';

export function Input(props: InputHTMLAttributes<HTMLInputElement>) {
  return (
    <input
      {...props}
      className={cn(
        'h-9 w-full rounded-md border border-slate-700 bg-slate-950/80 px-3 text-sm text-slate-100 outline-none ring-0 placeholder:text-slate-500 focus:border-sky-500',
        props.className
      )}
    />
  );
}
