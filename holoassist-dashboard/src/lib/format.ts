export function formatNumber(value: number, digits = 3): string {
  return value.toFixed(digits);
}

export function formatDegrees(valueRad: number): string {
  return `${(valueRad * 180 / Math.PI).toFixed(1)}°`;
}

export function formatLogTime(iso: string): string {
  const date = new Date(iso);
  return new Intl.DateTimeFormat('en-GB', {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    fractionalSecondDigits: 3,
    hour12: false
  }).format(date);
}
