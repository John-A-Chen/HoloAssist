import { cn } from '../../lib/cn';

interface SwitchProps {
  checked: boolean;
  disabled?: boolean;
  onCheckedChange: (checked: boolean) => void;
  ariaLabel: string;
}

export function Switch({ checked, disabled, onCheckedChange, ariaLabel }: SwitchProps) {
  return (
    <button
      type="button"
      role="switch"
      aria-checked={checked}
      aria-label={ariaLabel}
      disabled={disabled}
      onClick={() => onCheckedChange(!checked)}
      className={cn(
        'relative inline-flex h-6 w-11 items-center rounded-full border transition-colours focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-sky-400 disabled:cursor-not-allowed disabled:opacity-40',
        checked ? 'border-sky-500 bg-sky-500/20' : 'border-slate-700 bg-slate-800'
      )}
    >
      <span
        className={cn(
          'inline-block h-4 w-4 rounded-full transition-transform',
          checked ? 'translate-x-6 bg-sky-300' : 'translate-x-1 bg-slate-300'
        )}
      />
    </button>
  );
}
