"use client";

import { useEffect, useState, useCallback } from "react";
import AppLayout from "@/components/AppLayout";
import {
  Pencil,
  BookMarked,
  Plus,
  Trash2,
  ChevronRight,
  ExternalLink,
  Loader2,
  Save,
  X,
} from "lucide-react";
import Link from "next/link";

interface SectionAnnotation {
  sectionId: string;
  sectionTitle: string;
  content: string;
  updatedAt: string;
  docPath?: string;
}

interface FreeNote {
  id: string;
  title: string;
  content: string;
  createdAt: string;
  updatedAt: string;
}

interface KnowledgeModule {
  id: string;
  title: string;
  path: string;
}

interface KnowledgeSection {
  id: string;
  title: string;
  description: string;
  modules: KnowledgeModule[];
}

const REFERENCE_APPENDIX_SUBGROUPS: { prefix: string; label: string }[] = [
  { prefix: "control-theory/", label: "控制理论" },
  { prefix: "electronics-basics/", label: "电子基础" },
  { prefix: "power-path/", label: "电源路径" },
];

const DATE_FORMAT: Intl.DateTimeFormatOptions = {
  year: "numeric",
  month: "long",
  day: "numeric",
  hour: "2-digit",
  minute: "2-digit",
};

export default function NotesPage() {
  const [activeTab, setActiveTab] = useState<"chapters" | "free">("chapters");
  const [chapterAnnotations, setChapterAnnotations] = useState<
    (SectionAnnotation & { docPath: string })[]
  >([]);
  const [knowledgeSections, setKnowledgeSections] = useState<KnowledgeSection[]>([]);
  const [freeNotes, setFreeNotes] = useState<FreeNote[]>([]);
  const [loading, setLoading] = useState(true);

  const [editingNoteId, setEditingNoteId] = useState<string | null>(null);
  const [editTitle, setEditTitle] = useState("");
  const [editContent, setEditContent] = useState("");
  const [creatingNew, setCreatingNew] = useState(false);
  const [newTitle, setNewTitle] = useState("");
  const [newContent, setNewContent] = useState("");
  const [saving, setSaving] = useState(false);

  const [expandedSections, setExpandedSections] = useState<Set<string>>(
    new Set()
  );

  useEffect(() => {
    async function loadData() {
      try {
        const [annotationsRes, sectionsRes, notesRes] = await Promise.all([
          fetch("/api/annotations"),
          fetch("/api/knowledge-base/list"),
          fetch("/api/notes"),
        ]);

        const annotationsData = await annotationsRes.json();
        const sectionsData = await sectionsRes.json();
        const notesData = await notesRes.json();

        setChapterAnnotations(
          annotationsData.annotations ||
            annotationsData ||
            []
        );
        setKnowledgeSections(sectionsData.sections || []);
        setFreeNotes(notesData.notes || []);
      } catch {
        setChapterAnnotations([]);
        setKnowledgeSections([]);
        setFreeNotes([]);
      } finally {
        setLoading(false);
      }
    }
    loadData();
  }, []);

  const toggleSection = useCallback((sectionId: string) => {
    setExpandedSections((prev) => {
      const next = new Set(prev);
      if (next.has(sectionId)) {
        next.delete(sectionId);
      } else {
        next.add(sectionId);
      }
      return next;
    });
  }, []);

  const annotationsByDocPath = new Map<string, (SectionAnnotation & { docPath: string })[]>();
  for (const ann of chapterAnnotations) {
    const dp = ann.docPath || "";
    if (!annotationsByDocPath.has(dp)) {
      annotationsByDocPath.set(dp, []);
    }
    annotationsByDocPath.get(dp)!.push(ann);
  }

  const refreshFreeNotes = useCallback(async () => {
    try {
      const res = await fetch("/api/notes");
      const data = await res.json();
      setFreeNotes(data.notes || []);
    } catch {}
  }, []);

  const handleCreateNote = useCallback(async () => {
    if (!newTitle.trim() || !newContent.trim()) return;
    setSaving(true);
    try {
      const res = await fetch("/api/notes", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ title: newTitle.trim(), content: newContent.trim() }),
      });
      if (res.ok) {
        await refreshFreeNotes();
        setCreatingNew(false);
        setNewTitle("");
        setNewContent("");
      }
    } catch {} finally {
      setSaving(false);
    }
  }, [newTitle, newContent, refreshFreeNotes]);

  const handleEditNote = useCallback(
    (note: FreeNote) => {
      setEditingNoteId(note.id);
      setEditTitle(note.title);
      setEditContent(note.content);
    },
    []
  );

  const handleSaveEdit = useCallback(async () => {
    if (!editingNoteId) return;
    setSaving(true);
    try {
      const res = await fetch("/api/notes", {
        method: "PUT",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          id: editingNoteId,
          title: editTitle.trim(),
          content: editContent.trim(),
        }),
      });
      if (res.ok) {
        setEditingNoteId(null);
        setEditTitle("");
        setEditContent("");
        await refreshFreeNotes();
      }
    } catch {} finally {
      setSaving(false);
    }
  }, [editingNoteId, editTitle, editContent, refreshFreeNotes]);

  const handleCancelEdit = useCallback(() => {
    setEditingNoteId(null);
    setEditTitle("");
    setEditContent("");
  }, []);

  const handleDeleteNote = useCallback(
    async (id: string) => {
      if (!window.confirm("确定删除这条笔记吗？")) return;
      try {
        await fetch("/api/notes", {
          method: "DELETE",
          headers: { "Content-Type": "application/json" },
          body: JSON.stringify({ id }),
        });
        await refreshFreeNotes();
      } catch {}
    },
    [refreshFreeNotes]
  );

  const handleCancelCreate = useCallback(() => {
    setCreatingNew(false);
    setNewTitle("");
    setNewContent("");
  }, []);

  if (loading) {
    return (
      <AppLayout>
        <div className="flex items-center justify-center h-full">
          <Loader2 className="h-8 w-8 animate-spin text-muted-foreground" />
        </div>
      </AppLayout>
    );
  }

  return (
    <AppLayout>
      <div className="p-6 space-y-6 max-w-5xl mx-auto">
        <div className="flex items-center justify-between">
          <div>
            <h1 className="text-2xl font-bold flex items-center gap-2">
              <BookMarked className="h-6 w-6 text-primary" />
              我的笔记
            </h1>
            <p className="text-muted-foreground mt-1">
              章节注释与自由笔记，沉淀学习心得
            </p>
          </div>
        </div>

        <div className="flex items-center gap-1">
          <button
            onClick={() => setActiveTab("chapters")}
            className={`px-3 py-1.5 rounded-lg text-sm font-medium border transition-colors ${
              activeTab === "chapters"
                ? "bg-accent border-primary/30 text-foreground"
                : "border-transparent text-muted-foreground hover:text-foreground"
            }`}
          >
            章节注释
          </button>
          <button
            onClick={() => setActiveTab("free")}
            className={`px-3 py-1.5 rounded-lg text-sm font-medium border transition-colors ${
              activeTab === "free"
                ? "bg-accent border-primary/30 text-foreground"
                : "border-transparent text-muted-foreground hover:text-foreground"
            }`}
          >
            自由笔记
          </button>
        </div>

        {activeTab === "chapters" ? (
          <div className="space-y-3">
            {knowledgeSections.length === 0 ? (
              <div className="bg-card border border-border rounded-xl p-8 text-center">
                <p className="text-muted-foreground">
                  暂无知识库章节数据
                </p>
              </div>
            ) : (
              knowledgeSections.map((section) => {
                const isExpanded = expandedSections.has(section.id);
                const isReferenceAppendix = section.id === "reference-appendix";

                if (isReferenceAppendix) {
                  const subgrouped = new Map<string, KnowledgeModule[]>();
                  for (const mod of section.modules) {
                    const match = REFERENCE_APPENDIX_SUBGROUPS.find((sg) =>
                      mod.path.startsWith(sg.prefix)
                    );
                    const key = match ? match.label : "其他";
                    if (!subgrouped.has(key)) subgrouped.set(key, []);
                    subgrouped.get(key)!.push(mod);
                  }

                  const hasAnyAnnotation = Array.from(subgrouped.values())
                    .flat()
                    .some((mod) => annotationsByDocPath.has(mod.path));

                  return (
                    <div
                      key={section.id}
                      className="bg-card border border-border rounded-xl overflow-hidden"
                    >
                      <button
                        onClick={() => toggleSection(section.id)}
                        className="w-full px-5 py-4 flex items-center gap-3 hover:bg-accent/40 transition-colors text-left"
                      >
                        <ChevronRight
                          className={`h-4 w-4 text-muted-foreground shrink-0 transition-transform ${
                            isExpanded ? "rotate-90" : ""
                          }`}
                        />
                        <div className="flex-1 min-w-0">
                          <h3 className="text-base font-semibold">
                            {section.title}
                          </h3>
                          {section.description && (
                            <p className="text-xs text-muted-foreground mt-0.5">
                              {section.description}
                            </p>
                          )}
                        </div>
                        {hasAnyAnnotation && (
                          <span className="text-[10px] bg-primary/20 text-primary px-2 py-0.5 rounded-full shrink-0">
                            有笔记
                          </span>
                        )}
                      </button>

                      {isExpanded && (
                        <div className="px-5 pb-4 space-y-4">
                          {Array.from(subgrouped.entries()).map(
                            ([label, mods]) => (
                              <div key={label}>
                                <h4 className="text-xs font-semibold text-muted-foreground uppercase tracking-wider mb-2 pl-2 border-l-2 border-border">
                                  {label}
                                </h4>
                                <div className="space-y-2">
                                  {mods.map((mod) => {
                                    const anns =
                                      annotationsByDocPath.get(mod.path) ||
                                      [];
                                    return (
                                      <div
                                        key={mod.path}
                                        className="pl-2"
                                      >
                                        <div className="flex items-center gap-2 py-1">
                                          <span className="text-sm font-medium truncate flex-1">
                                            {mod.title}
                                          </span>
                                          {anns.length > 0 && (
                                            <span className="text-[10px] text-muted-foreground shrink-0">
                                              {anns.length} 条注释
                                            </span>
                                          )}
                                        </div>
                                        {anns.length > 0 && (
                                          <div className="ml-4 mt-1 space-y-1.5 border-l border-border/60 pl-3">
                                            {anns.map((ann) => (
                                              <Link
                                                key={ann.sectionId}
                                                href={`/pdf-reader?doc=${encodeURIComponent(ann.docPath)}&sectionId=${encodeURIComponent(ann.sectionId)}`}
                                                className="block p-2.5 rounded-lg bg-accent/30 hover:bg-accent/50 transition-colors border border-border/50"
                                              >
                                                <div className="flex items-start gap-2">
                                                  <span className="text-xs font-medium text-foreground truncate flex-1">
                                                    {ann.sectionTitle || "未命名小节"}
                                                  </span>
                                                  <ExternalLink className="h-3 w-3 text-muted-foreground shrink-0 mt-0.5" />
                                                </div>
                                                <p className="text-xs text-muted-foreground mt-1 line-clamp-2">
                                                  {ann.content.slice(0, 80) +
                                                    (ann.content.length > 80
                                                      ? "..."
                                                      : "")}
                                                </p>
                                                <p className="text-[10px] text-muted-foreground mt-1.5">
                                                  {new Date(
                                                    ann.updatedAt
                                                  ).toLocaleDateString(
                                                    "zh-CN",
                                                    DATE_FORMAT
                                                  )}
                                                </p>
                                              </Link>
                                            ))}
                                          </div>
                                        )}
                                        {anns.length === 0 && (
                                          <p className="ml-4 text-[11px] text-muted-foreground pl-3">
                                            暂无注释
                                          </p>
                                        )}
                                      </div>
                                    );
                                  })}
                                </div>
                              </div>
                            )
                          )}
                        </div>
                      )}
                    </div>
                  );
                }

                const hasAnyAnnotation = section.modules.some((mod) =>
                  annotationsByDocPath.has(mod.path)
                );

                return (
                  <div
                    key={section.id}
                    className="bg-card border border-border rounded-xl overflow-hidden"
                  >
                    <button
                      onClick={() => toggleSection(section.id)}
                      className="w-full px-5 py-4 flex items-center gap-3 hover:bg-accent/40 transition-colors text-left"
                    >
                      <ChevronRight
                        className={`h-4 w-4 text-muted-foreground shrink-0 transition-transform ${
                          isExpanded ? "rotate-90" : ""
                        }`}
                      />
                      <div className="flex-1 min-w-0">
                        <h3 className="text-base font-semibold">
                          {section.title}
                        </h3>
                        {section.description && (
                          <p className="text-xs text-muted-foreground mt-0.5">
                            {section.description}
                          </p>
                        )}
                      </div>
                      {hasAnyAnnotation && (
                        <span className="text-[10px] bg-primary/20 text-primary px-2 py-0.5 rounded-full shrink-0">
                          有笔记
                        </span>
                      )}
                    </button>

                    {isExpanded && (
                      <div className="px-5 pb-4 space-y-1.5">
                        {section.modules.map((mod) => {
                          const anns =
                            annotationsByDocPath.get(mod.path) || [];
                          return (
                            <div key={mod.path} className="pl-2">
                              <div className="flex items-center gap-2 py-1.5">
                                <span className="text-sm font-medium truncate flex-1">
                                  {mod.title}
                                </span>
                                {anns.length > 0 && (
                                  <span className="text-[10px] text-muted-foreground shrink-0">
                                    {anns.length} 条注释
                                  </span>
                                )}
                              </div>
                              {anns.length > 0 && (
                                <div className="ml-4 mt-1 space-y-1.5 border-l border-border/60 pl-3">
                                  {anns.map((ann) => (
                                    <Link
                                      key={ann.sectionId}
                                      href={`/pdf-reader?doc=${encodeURIComponent(ann.docPath)}&sectionId=${encodeURIComponent(ann.sectionId)}`}
                                      className="block p-2.5 rounded-lg bg-accent/30 hover:bg-accent/50 transition-colors border border-border/50"
                                    >
                                      <div className="flex items-start gap-2">
                                        <span className="text-xs font-medium text-foreground truncate flex-1">
                                          {ann.sectionTitle || "未命名小节"}
                                        </span>
                                        <ExternalLink className="h-3 w-3 text-muted-foreground shrink-0 mt-0.5" />
                                      </div>
                                      <p className="text-xs text-muted-foreground mt-1 line-clamp-2">
                                        {ann.content.slice(0, 80) +
                                          (ann.content.length > 80
                                            ? "..."
                                            : "")}
                                      </p>
                                      <p className="text-[10px] text-muted-foreground mt-1.5">
                                        {new Date(
                                          ann.updatedAt
                                        ).toLocaleDateString(
                                          "zh-CN",
                                          DATE_FORMAT
                                        )}
                                      </p>
                                    </Link>
                                  ))}
                                </div>
                              )}
                              {anns.length === 0 && (
                                <p className="ml-4 text-[11px] text-muted-foreground pl-3">
                                  暂无注释
                                </p>
                              )}
                            </div>
                          );
                        })}
                      </div>
                    )}
                  </div>
                );
              })
            )}

            {chapterAnnotations.length === 0 && knowledgeSections.length > 0 && (
              <div className="bg-card border border-border rounded-xl p-8 text-center">
                <BookMarked className="h-10 w-10 text-muted-foreground mx-auto mb-3 opacity-40" />
                <p className="text-muted-foreground">
                  暂无章节注释。在学习知识库文档时，点击小节旁的 📝 按钮添加注释。
                </p>
              </div>
            )}
          </div>
        ) : (
          <div className="space-y-4">
            {!creatingNew && (
              <button
                onClick={() => setCreatingNew(true)}
                className="inline-flex items-center gap-2 px-4 py-2.5 rounded-lg text-sm font-medium bg-primary/10 border border-primary/30 text-primary hover:bg-primary/20 transition-colors"
              >
                <Plus className="h-4 w-4" />
                新建笔记
              </button>
            )}

            {creatingNew && (
              <div className="bg-card border border-border rounded-xl p-5 space-y-3">
                <input
                  type="text"
                  value={newTitle}
                  onChange={(e) => setNewTitle(e.target.value)}
                  placeholder="笔记标题"
                  className="w-full px-3 py-2 rounded-lg bg-accent/40 border border-border text-sm text-foreground placeholder:text-muted-foreground focus:outline-none focus:border-primary/50"
                />
                <textarea
                  value={newContent}
                  onChange={(e) => setNewContent(e.target.value)}
                  placeholder="输入笔记内容...支持 Markdown"
                  rows={5}
                  className="w-full px-3 py-2 rounded-lg bg-accent/40 border border-border text-sm text-foreground placeholder:text-muted-foreground focus:outline-none focus:border-primary/50 resize-y"
                />
                <div className="flex items-center gap-2">
                  <button
                    onClick={handleCreateNote}
                    disabled={saving || !newTitle.trim() || !newContent.trim()}
                    className="inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-medium bg-primary text-primary-foreground hover:bg-primary/90 disabled:opacity-40 transition-colors"
                  >
                    {saving ? (
                      <Loader2 className="h-3.5 w-3.5 animate-spin" />
                    ) : (
                      <Save className="h-3.5 w-3.5" />
                    )}
                    保存
                  </button>
                  <button
                    onClick={handleCancelCreate}
                    className="inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-medium border border-border text-muted-foreground hover:text-foreground transition-colors"
                  >
                    <X className="h-3.5 w-3.5" />
                    取消
                  </button>
                </div>
              </div>
            )}

            {freeNotes.length === 0 && !creatingNew ? (
              <div className="bg-card border border-border rounded-xl p-8 text-center">
                <Pencil className="h-10 w-10 text-muted-foreground mx-auto mb-3 opacity-40" />
                <p className="text-muted-foreground">
                  暂无自由笔记，点击上方「新建笔记」开始记录
                </p>
              </div>
            ) : (
              <div className="grid grid-cols-1 sm:grid-cols-2 gap-4">
                {freeNotes.map((note) => (
                  <div
                    key={note.id}
                    className="bg-card border border-border rounded-xl overflow-hidden"
                  >
                    {editingNoteId === note.id ? (
                      <div className="p-4 space-y-3">
                        <input
                          type="text"
                          value={editTitle}
                          onChange={(e) => setEditTitle(e.target.value)}
                          className="w-full px-3 py-2 rounded-lg bg-accent/40 border border-border text-sm text-foreground placeholder:text-muted-foreground focus:outline-none focus:border-primary/50"
                        />
                        <textarea
                          value={editContent}
                          onChange={(e) => setEditContent(e.target.value)}
                          rows={4}
                          className="w-full px-3 py-2 rounded-lg bg-accent/40 border border-border text-sm text-foreground placeholder:text-muted-foreground focus:outline-none focus:border-primary/50 resize-y"
                        />
                        <div className="flex items-center gap-2">
                          <button
                            onClick={handleSaveEdit}
                            disabled={saving || !editTitle.trim()}
                            className="inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-medium bg-primary text-primary-foreground hover:bg-primary/90 disabled:opacity-40 transition-colors"
                          >
                            {saving ? (
                              <Loader2 className="h-3.5 w-3.5 animate-spin" />
                            ) : (
                              <Save className="h-3.5 w-3.5" />
                            )}
                            保存
                          </button>
                          <button
                            onClick={handleCancelEdit}
                            className="inline-flex items-center gap-1.5 px-3 py-1.5 rounded-lg text-xs font-medium border border-border text-muted-foreground hover:text-foreground transition-colors"
                          >
                            <X className="h-3.5 w-3.5" />
                            取消
                          </button>
                          <button
                            onClick={() => handleDeleteNote(note.id)}
                            className="ml-auto inline-flex items-center gap-1 px-2 py-1.5 rounded-lg text-xs font-medium text-red-400 hover:bg-red-500/10 transition-colors"
                          >
                            <Trash2 className="h-3.5 w-3.5" />
                            删除
                          </button>
                        </div>
                      </div>
                    ) : (
                      <div
                        className="p-4 cursor-pointer hover:bg-accent/30 transition-colors"
                        onClick={() => handleEditNote(note)}
                      >
                        <h3 className="text-sm font-bold truncate">
                          {note.title}
                        </h3>
                        <p className="text-xs text-muted-foreground mt-1.5 line-clamp-3">
                          {note.content.slice(0, 100) +
                            (note.content.length > 100 ? "..." : "")}
                        </p>
                        <div className="flex items-center gap-3 mt-3 pt-2 border-t border-border/50">
                          <span className="text-[10px] text-muted-foreground">
                            {new Date(note.createdAt).toLocaleDateString(
                              "zh-CN",
                              DATE_FORMAT
                            )}
                          </span>
                          <div className="flex items-center gap-1 ml-auto">
                            <button
                              onClick={(e) => {
                                e.stopPropagation();
                                handleEditNote(note);
                              }}
                              className="p-1 rounded hover:bg-accent text-muted-foreground hover:text-foreground transition-colors"
                            >
                              <Pencil className="h-3.5 w-3.5" />
                            </button>
                            <button
                              onClick={(e) => {
                                e.stopPropagation();
                                handleDeleteNote(note.id);
                              }}
                              className="p-1 rounded hover:bg-red-500/10 text-muted-foreground hover:text-red-400 transition-colors"
                            >
                              <Trash2 className="h-3.5 w-3.5" />
                            </button>
                          </div>
                        </div>
                      </div>
                    )}
                  </div>
                ))}
              </div>
            )}
          </div>
        )}
      </div>
    </AppLayout>
  );
}