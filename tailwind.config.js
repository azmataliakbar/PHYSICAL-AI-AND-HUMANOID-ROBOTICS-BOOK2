/** @type {import('tailwindcss').Config} */
export default {
  content: [
    "./index.html",
    "./src/**/*.{js,ts,jsx,tsx}",
  ],
  theme: {
    // ADD CUSTOM BREAKPOINTS
    screens: {
      'xs': '375px',    // Extra small devices (small phones)
      'sm': '640px',    // Small devices (phones)
      'md': '768px',    // Medium devices (tablets)
      'lg': '1024px',   // Large devices (laptops)
      'xl': '1280px',   // Extra large devices (desktops)
      '2xl': '1536px',  // 2X large devices (large desktops)
    },
    extend: {
      colors: {
        'primary': {
          DEFAULT: '#4CAF50',
          dark: '#2E7D32',
          light: '#81C784',
        },
      },
      boxShadow: {
        'green-sm': '0 2px 4px rgba(46, 125, 50, 0.1)',
        'green-md': '0 4px 8px rgba(46, 125, 50, 0.2)',
        'green-lg': '0 10px 20px rgba(46, 125, 50, 0.3)',
      },
      // ADD CUSTOM ANIMATIONS
      keyframes: {
        'slide-up': {
          '0%': { 
            opacity: '0', 
            transform: 'translateY(20px)' 
          },
          '100%': { 
            opacity: '1', 
            transform: 'translateY(0)' 
          },
        },
        'fade-in': {
          '0%': { opacity: '0' },
          '100%': { opacity: '1' },
        },
      },
      animation: {
        'slide-up': 'slide-up 0.3s ease-out',
        'fade-in': 'fade-in 0.3s ease-out',
      },
      // ADD CUSTOM SPACING FOR CHAT WIDGET
      spacing: {
        '13': '3.25rem',
        '15': '3.75rem',
      },
    },
  },
  plugins: [],
}