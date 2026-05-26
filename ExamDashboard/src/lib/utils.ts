import { clsx, type ClassValue } from "clsx";
import { twMerge } from "tailwind-merge";

export function cn(...inputs: ClassValue[]) {
  return twMerge(clsx(inputs));
}

export function renderLatex(text: string): string {
  return text
    .replace(/\$\$(.+?)\$\$/g, "$$$$$1$$$$")
    .replace(/\$(.+?)\$/g, "$$$1$");
}

export function difficultyStars(level: number): string {
  return "★".repeat(Math.max(1, Math.min(5, Math.round(level))));
}

export function difficultyLabel(level: number): string {
  const clamped = Math.max(1, Math.min(5, Math.round(level)));
  if (clamped <= 2) return "入门级";
  if (clamped <= 4) return "工程级";
  return "专家级";
}
