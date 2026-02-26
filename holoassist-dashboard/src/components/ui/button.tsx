import type { ButtonHTMLAttributes } from 'react';

import { cn } from '../../lib/cn';

type ButtonVariant = 'default' | 'outline' | 'ghost' | 'destructive' | 'success';
type ButtonSize = 'sm' | 'md' | 'lg';

export interface ButtonProps extends ButtonHTMLAttributes<HTMLButtonElement> {
  variant?: ButtonVariant;
  size?: ButtonSize;
}

const variantClasses: Record<ButtonVariant, string> = {
  default:
    'bg-slate-100 text-slate-950 hover:bg-white disabled:bg-slate-800 disabled:text-slate-500',
  outline:
    'border border-slate-700 bg-slate-900 text-slate-100 hover:border-slate-500 hover:bg-slate-800 disabled:opacity-40',
  ghost:
    'bg-transparent text-slate-200 hover:bg-slate-800/70 disabled:opacity-40',
  destructive:
    'bg-red-600 text-white hover:bg-red-500 disabled:bg-red-950 disabled:text-red-300/60',
  success:
    'bg-emerald-600 text-white hover:bg-emerald-500 disabled:bg-emerald-950 disabled:text-emerald-300/60'
};

const sizeClasses: Record<ButtonSize, string> = {
  sm: 'h-8 px-3 text-sm',
  md: 'h-10 px-4 text-sm',
  lg: 'h-11 px-5 text-base'
};

export function Button({ className, variant = 'default', size = 'md', type = 'button', ...props }: ButtonProps) {
  return (
    <button
      type={type}
      className={cn(
        'inline-flex items-center justify-center rounded-md font-medium transition-colors focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-sky-400 focus-visible:ring-offset-2 focus-visible:ring-offset-slate-950 disabled:cursor-not-allowed',
        variantClasses[variant],
        sizeClasses[size],
        className
      )}
      {...props}
    />
  );
}
