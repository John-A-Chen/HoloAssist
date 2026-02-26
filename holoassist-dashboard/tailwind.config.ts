import type { Config } from 'tailwindcss';

export default {
  content: ['./index.html', './src/**/*.{ts,tsx}'],
  theme: {
    extend: {
      boxShadow: {
        panel: '0 0 0 1px rgb(255 255 255 / 0.04), 0 12px 28px rgb(0 0 0 / 0.28)'
      },
      keyframes: {
        pulseSoft: {
          '0%, 100%': { opacity: '0.55' },
          '50%': { opacity: '1' }
        }
      },
      animation: {
        pulseSoft: 'pulseSoft 1.8s ease-in-out infinite'
      }
    }
  },
  plugins: []
} satisfies Config;
