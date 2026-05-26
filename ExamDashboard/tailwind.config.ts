import type { Config } from "tailwindcss";

const config: Config = {
  content: ["./src/**/*.{js,ts,jsx,tsx,mdx}"],
  theme: {
    extend: {
      colors: {
        background: "hsl(0 0% 100%)",
        foreground: "hsl(220 15% 15%)",
        card: "hsl(0 0% 100%)",
        "card-foreground": "hsl(220 15% 15%)",
        popover: "hsl(0 0% 100%)",
        "popover-foreground": "hsl(220 15% 15%)",
        primary: "hsl(210 100% 40%)",
        "primary-foreground": "hsl(0 0% 100%)",
        secondary: "hsl(220 15% 95%)",
        "secondary-foreground": "hsl(220 15% 20%)",
        muted: "hsl(220 15% 95%)",
        "muted-foreground": "hsl(220 15% 45%)",
        accent: "hsl(220 15% 93%)",
        "accent-foreground": "hsl(220 15% 20%)",
        destructive: "hsl(0 80% 45%)",
        "destructive-foreground": "hsl(0 0% 100%)",
        border: "hsl(220 15% 85%)",
        input: "hsl(220 15% 90%)",
        ring: "hsl(210 100% 40%)",
        success: "hsl(140 60% 40%)",
        warning: "hsl(40 100% 45%)",
        "electric-blue": "hsl(210 100% 40%)",
        "motor-purple": "hsl(270 60% 50%)",
        "terminal-bg": "hsl(220 15% 95%)",
        "terminal-border": "hsl(220 15% 85%)",
        "terminal-text": "hsl(140 60% 30%)",
      },
    },
  },
  plugins: [require("@tailwindcss/typography")],
};
export default config;
