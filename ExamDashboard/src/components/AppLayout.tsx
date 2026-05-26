"use client";

import Link from "next/link";
import { usePathname } from "next/navigation";
import {
  LayoutDashboard,
  BookOpen,
  FileText,
  GitGraph,
  BookMarked,
  Menu,
  X,
  Pencil,
} from "lucide-react";
import { useEffect, useState } from "react";
import { cn } from "@/lib/utils";

const navItems = [
  { href: "/dashboard", label: "学习仪表盘", icon: LayoutDashboard },
  { href: "/quiz", label: "灵活刷题", icon: BookOpen },
  { href: "/knowledge-graph", label: "知识图谱", icon: GitGraph },
  { href: "/pdf-reader", label: "知识库文档", icon: FileText },
  { href: "/notes", label: "我的笔记", icon: Pencil },
];

export default function AppLayout({ children }: { children: React.ReactNode }) {
  const pathname = usePathname();
  const [sidebarOpen, setSidebarOpen] = useState(false);
  const [totalNotes, setTotalNotes] = useState(0);

  useEffect(() => {
    fetch("/api/annotations")
      .then((res) => res.json())
      .then((data) => {
        setTotalNotes(data.annotations?.length || 0);
      })
      .catch(() => {});
  }, []);

  return (
    <div className="flex h-screen bg-background">
      {/* 移动端遮罩 */}
      {sidebarOpen && (
        <div
          className="fixed inset-0 z-40 bg-black/50 lg:hidden"
          onClick={() => setSidebarOpen(false)}
        />
      )}

      {/* 侧边栏 */}
      <aside
        className={cn(
          "fixed inset-y-0 left-0 z-50 w-56 bg-card border-r border-border flex flex-col transition-transform duration-200 lg:relative lg:translate-x-0",
          sidebarOpen ? "translate-x-0" : "-translate-x-full"
        )}
      >
        <div className="flex items-center gap-2 px-5 py-4 border-b border-border">
          <BookMarked className="h-6 w-6 text-primary" />
          <span className="text-lg font-bold tracking-tight">电控知识库</span>
          <button
            className="ml-auto lg:hidden text-muted-foreground"
            onClick={() => setSidebarOpen(false)}
          >
            <X className="h-5 w-5" />
          </button>
        </div>

        <nav className="flex-1 px-3 py-4 space-y-1">
          {navItems.map((item) => {
            const Icon = item.icon;
            const active = pathname.startsWith(item.href);
            return (
              <Link
                key={item.href}
                href={item.href}
                onClick={() => setSidebarOpen(false)}
                className={cn(
                  "flex items-center gap-3 px-3 py-2.5 rounded-lg text-sm font-medium transition-colors",
                  active
                    ? "bg-primary/15 text-primary"
                    : "text-muted-foreground hover:bg-accent hover:text-foreground"
                )}
              >
                <Icon className="h-4.5 w-4.5" />
                {item.label}
                {item.href === "/notes" && totalNotes > 0 && (
                  <span className="ml-auto text-[10px] bg-primary/20 text-primary px-1.5 py-0.5 rounded-full">{totalNotes}</span>
                )}
              </Link>
            );
          })}
        </nav>

        <div className="px-4 py-3 border-t border-border text-xs text-muted-foreground">
          v1.0 · FOC电机控制学习系统
        </div>
      </aside>

      {/* 主内容 */}
      <div className="flex-1 flex flex-col min-w-0">
        {/* 顶部栏 */}
        <header className="h-14 border-b border-border flex items-center px-4 gap-3 bg-card/50">
          <button
            className="lg:hidden text-muted-foreground"
            onClick={() => setSidebarOpen(true)}
          >
            <Menu className="h-5 w-5" />
          </button>
          <div className="flex-1" />
          <span className="text-sm text-muted-foreground">
            {new Date().toLocaleDateString("zh-CN", {
              year: "numeric", month: "long", day: "numeric", weekday: "long",
            })}
          </span>
        </header>

        {/* 页面内容 */}
        <main className="flex-1 overflow-auto scrollbar-thin">{children}</main>
      </div>
    </div>
  );
}
