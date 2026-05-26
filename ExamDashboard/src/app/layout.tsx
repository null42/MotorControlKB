import type { Metadata } from "next";
import "highlight.js/styles/github.css";
import "./globals.css";

export const metadata: Metadata = {
  title: "电机控制知识学习系统",
  description: "电机控制知识学习与追踪平台",
};

export default function RootLayout({
  children,
}: {
  children: React.ReactNode;
}) {
  return (
    <html lang="zh-CN">
      <body>{children}</body>
    </html>
  );
}
