export function formatNumber(value: number, digits = 3): string {
  return value.toFixed(digits);
}

export function formatDegrees(valueRad: number): string {
  return `${(valueRad * 180 / Math.PI).toFixed(1)}°`;
}

export function formatLogTime(iso: string): string {
  const date = new Date(iso);
  const hh = String(date.getHours()).padStart(2, '0');
  const mm = String(date.getMinutes()).padStart(2, '0');
  const ss = String(date.getSeconds()).padStart(2, '0');
  const ms = String(date.getMilliseconds()).padStart(3, '0');
  return `${hh}:${mm}:${ss}.${ms}`;
}
