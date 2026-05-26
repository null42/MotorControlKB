"use client";

import { Suspense, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { useSearchParams } from "next/navigation";
import AppLayout from "@/components/AppLayout";
import { mcOutline, PdfOutlineItem } from "@/data/mc-outline.generated";
import { renderMarkdown } from "@/lib/markdown";
import "katex/dist/katex.min.css";
import {
  FileText,
  BookOpen,
  Upload,
  ChevronDown,
  ChevronRight,
  ZoomIn,
  ZoomOut,
  RotateCcw,
  Loader2,
  Pencil,
  Check,
  X,
} from "lucide-react";
import { useAnnotations } from "@/lib/annotation-store";

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

const OUTLINE_DOC_MAP: Record<string, string> = {
  "第1章 FOC理论基础": "algorithm/ALG-01-FOC-Theory.md",
  "1.1 FOC理论基础": "algorithm/ALG-01-FOC-Theory.md",
  "1.2 前沿研究": "algorithm/ALG-06-Advanced-Research.md",
  "第2章 有感FOC实现": "algorithm/ALG-02-Sensored-FOC.md",
  "2.1 有感FOC实现": "algorithm/ALG-02-Sensored-FOC.md",
  "第3章 无感FOC观测器": "algorithm/ALG-03-Sensorless-Observers.md",
  "3.1 无感FOC观测器": "algorithm/ALG-03-Sensorless-Observers.md",
  "第4章 高频注入法": "algorithm/ALG-04-High-Frequency-Injection.md",
  "4.1 高频注入法": "algorithm/ALG-04-High-Frequency-Injection.md",
  "第5章 保护与优化": "algorithm/ALG-05-Protection-Optimization.md",
  "5.1 保护与优化": "algorithm/ALG-05-Protection-Optimization.md",
  "附录: 硬件基础": "hardware/HW-01-Motor-Basics.md",
  "附.1 电机本体基础": "hardware/HW-01-Motor-Basics.md",
  "附.2 电流采样电路": "hardware/HW-02-Current-Sensing.md",
  "附.3 位置传感器接口": "hardware/HW-03-Position-Sensor.md",
  "附.4 MCU外设与通信": "hardware/HW-04-MCU-Peripherals.md",
  "附.5 功率器件与栅极驱动": "hardware/HW-05-Power-Devices-Gate-Drivers.md",
  "附.6 电源管理与保护": "hardware/HW-06-Power-Management-Protection.md",
  "附.7 热设计与EMC": "hardware/HW-07-Thermal-EMC-Design.md",
};

type ViewMode = "knowledge-base" | "pdf-upload";

function DocReaderInner() {
  const searchParams = useSearchParams();
  const docParam = searchParams.get("doc");

  const [viewMode, setViewMode] = useState<ViewMode>("knowledge-base");
  const [docContent, setDocContent] = useState<string>("");
  const [docTitle, setDocTitle] = useState<string>("");
  const [activeDoc, setActiveDoc] = useState<string>("");
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string>("");

  const [pdfFile, setPdfFile] = useState<File | null>(null);
  const [currentPage, setCurrentPage] = useState(1);
  const [numPages, setNumPages] = useState(1);
  const [scale, setScale] = useState(1.3);
  const [outlineExpanded, setOutlineExpanded] = useState<Record<string, boolean>>({});

  const [knowledgeSections, setKnowledgeSections] = useState<KnowledgeSection[]>([]);

  const [activeAnnotationId, setActiveAnnotationId] = useState<string | null>(null)
  const [annotationDraft, setAnnotationDraft] = useState<string>("")
  const [annotationDirty, setAnnotationDirty] = useState(false)
  const [annotationEditMode, setAnnotationEditMode] = useState(false)
  const [savedToast, setSavedToast] = useState(false)
  const contentRef = useRef<HTMLDivElement>(null)
  const annotationDirtyRef = useRef(false)
  const handleAnnotationClickRef = useRef<(sectionId: string) => void>(() => {})
  const { hasAnnotation, saveAnnotation, deleteAnnotation, annotations } = useAnnotations(activeDoc)

  useEffect(() => {
    annotationDirtyRef.current = annotationDirty
  }, [annotationDirty])

  useEffect(() => {
    fetch("/api/knowledge-base/list")
      .then((r) => r.json())
      .then((data) => setKnowledgeSections(data.sections || []))
      .catch(() => setKnowledgeSections([]));
  }, []);

  const loadDocument = useCallback(async (docPath: string) => {
    if (annotationDirtyRef.current) {
      if (!window.confirm("你有未保存的注释，确定放弃修改吗？")) return
      setAnnotationDirty(false)
      setActiveAnnotationId(null)
    }
    setLoading(true);
    setError("");
    setActiveDoc(docPath);
    try {
      const res = await fetch(`/api/knowledge-base/doc/${docPath}`);
      if (!res.ok) throw new Error("文档加载失败");
      const data = await res.json();
      setDocContent(data.content);
      setDocTitle(data.title);
    } catch (e) {
      setError(e instanceof Error ? e.message : "加载失败");
      setDocContent("");
    }
    setLoading(false);
  }, []);

  const handleAnnotationClick = (sectionId: string) => {
    if (annotationDirtyRef.current && activeAnnotationId !== sectionId) {
      if (!window.confirm("你有未保存的注释，确定放弃修改吗？")) return
    }
    if (activeAnnotationId === sectionId) {
      setActiveAnnotationId(null)
      setAnnotationDraft("")
      setAnnotationDirty(false)
      setAnnotationEditMode(false)
      return
    }
    setActiveAnnotationId(sectionId)
    const existing = annotations[sectionId]
    if (existing && existing.content) {
      setAnnotationDraft(existing.content)
      setAnnotationDirty(false)
      setAnnotationEditMode(false)
    } else {
      setAnnotationDraft("")
      setAnnotationDirty(false)
      setAnnotationEditMode(true)
    }
  }

  useEffect(() => {
    handleAnnotationClickRef.current = handleAnnotationClick
  })

  const handleSaveAnnotation = async () => {
    if (!activeAnnotationId) return
    const heading = contentRef.current?.querySelector(`#${CSS.escape(activeAnnotationId)}`)
    const sectionTitle = heading?.textContent?.replace(/📝/g, "").trim() || activeAnnotationId
    if (annotationDraft.trim() === "") {
      await deleteAnnotation(activeAnnotationId)
    } else {
      await saveAnnotation(activeAnnotationId, sectionTitle, annotationDraft)
    }
    setActiveAnnotationId(null)
    setAnnotationDraft("")
    setAnnotationDirty(false)
    setAnnotationEditMode(false)
    setSavedToast(true)
    setTimeout(() => setSavedToast(false), 2000)
  }

  const handleCancelAnnotation = () => {
    if (annotationDirty) {
      if (!window.confirm("你有未保存的注释，确定放弃修改吗？")) return
    }
    setActiveAnnotationId(null)
    setAnnotationDraft("")
    setAnnotationDirty(false)
    setAnnotationEditMode(false)
  }

  const handleDeleteAnnotation = async () => {
    if (!window.confirm("确定删除这条注释吗？")) return
    if (!activeAnnotationId) return
    await deleteAnnotation(activeAnnotationId)
    setActiveAnnotationId(null)
    setAnnotationDraft("")
    setAnnotationDirty(false)
    setAnnotationEditMode(false)
  }

  useEffect(() => {
    const sectionId = searchParams.get("sectionId")
    if (sectionId && docContent) {
      setActiveAnnotationId(sectionId)
      const existing = annotations[sectionId]
      if (existing && existing.content) {
        setAnnotationDraft(existing.content)
        setAnnotationDirty(false)
        setAnnotationEditMode(false)
      }
    }
  }, [docContent, annotations, searchParams])

  useEffect(() => {
    if (docParam) {
      loadDocument(docParam);
    } else {
      loadDocument("algorithm/ALG-01-FOC-Theory.md");
    }
  }, [docParam, loadDocument]);

  const toggleOutline = (label: string) => {
    setOutlineExpanded((prev) => ({ ...prev, [label]: !prev[label] }));
  };

  const jumpToPage = (page: number) => {
    setCurrentPage(Math.max(1, Math.min(page, numPages)));
  };

  const handleFileDrop = (e: React.DragEvent) => {
    e.preventDefault();
    const file = e.dataTransfer.files[0];
    if (file && file.type === "application/pdf") {
      setPdfFile(file);
      setViewMode("pdf-upload");
    }
  };

  const renderedContent = useMemo(() => {
    if (!docContent) return "";
    return renderMarkdown(docContent);
  }, [docContent]);

  useEffect(() => {
    if (!contentRef.current) return
    const headings = contentRef.current.querySelectorAll("h2, h3")
    headings.forEach((heading) => {
      const existing = heading.querySelector(".annotation-btn")
      if (existing) existing.remove()
      const id = heading.getAttribute("id") || ""
      if (!id) return
      const btn = document.createElement("button")
      btn.className = "annotation-btn inline-flex items-center ml-2 text-muted-foreground hover:text-primary transition-colors"
      btn.innerHTML = "📝"
      btn.title = hasAnnotation(id) ? "编辑注释" : "添加注释"
      btn.style.opacity = hasAnnotation(id) ? "1" : "0.3"
      btn.onclick = (e) => {
        e.stopPropagation()
        handleAnnotationClickRef.current(id)
      }
      heading.appendChild(btn)
    })
  }, [renderedContent, activeDoc, hasAnnotation])

  const handleFileInput = (e: React.ChangeEvent<HTMLInputElement>) => {
    const file = e.target.files?.[0];
    if (file) {
      setPdfFile(file);
      setViewMode("pdf-upload");
    }
  };

  const findDocForOutlineLabel = (label: string): string | null => {
    return OUTLINE_DOC_MAP[label] || null;
  };

  const isItemActive = (item: PdfOutlineItem): boolean => {
    const docPath = findDocForOutlineLabel(item.label);
    if (docPath) return activeDoc === docPath;
    if (item.children) {
      return item.children.some((child) => isItemActive(child));
    }
    return false;
  };

  const renderOutlineItem = (
    item: PdfOutlineItem,
    depth: number = 0
  ) => {
    const expanded = outlineExpanded[item.label] !== false;
    const hasChildren = item.children && item.children.length > 0;
    const docPath = findDocForOutlineLabel(item.label);
    const active = isItemActive(item);

    return (
      <div key={item.label} style={{ paddingLeft: depth * 16 }}>
        <div
          className={`flex items-center gap-1 py-1.5 px-2 rounded text-xs cursor-pointer transition-colors ${
            active
              ? "bg-primary/15 text-primary font-medium"
              : "text-muted-foreground hover:text-foreground hover:bg-accent"
          }`}
          onClick={() => {
            if (hasChildren) toggleOutline(item.label);
            if (viewMode === "knowledge-base" && docPath) {
              loadDocument(docPath);
            } else if (viewMode === "pdf-upload") {
              jumpToPage(item.page || 1);
            }
          }}
        >
          {hasChildren ? (
            expanded ? (
              <ChevronDown className="h-3 w-3 shrink-0" />
            ) : (
              <ChevronRight className="h-3 w-3 shrink-0" />
            )
          ) : (
            <span className="w-3 shrink-0" />
          )}
          <span className="truncate">{item.label}</span>
          {viewMode === "pdf-upload" && (
            <span className="ml-auto text-[10px] opacity-40">{item.page}</span>
          )}
        </div>
        {hasChildren && expanded && item.children!.map((child) => renderOutlineItem(child, depth + 1))}
      </div>
    );
  };

  const renderKnowledgeSections = () => {
    if (knowledgeSections.length === 0) return null;
    return knowledgeSections.map((section) => {
      const expanded = outlineExpanded[section.id] !== false;
      return (
        <div key={section.id}>
          <div
            className="flex items-center gap-1 py-1.5 px-2 rounded text-xs cursor-pointer text-muted-foreground hover:text-foreground hover:bg-accent"
            onClick={() => toggleOutline(section.id)}
          >
            {expanded ? (
              <ChevronDown className="h-3 w-3 shrink-0" />
            ) : (
              <ChevronRight className="h-3 w-3 shrink-0" />
            )}
            <span className="font-medium truncate">{section.title}</span>
          </div>
          {expanded &&
            section.modules.map((mod) => {
              const active = activeDoc === mod.path;
              return (
                <div
                  key={mod.id}
                  className={`flex items-center gap-1 py-1.5 pl-8 pr-2 rounded text-xs cursor-pointer transition-colors ${
                    active
                      ? "bg-primary/15 text-primary font-medium"
                      : "text-muted-foreground hover:text-foreground hover:bg-accent"
                  }`}
                  onClick={() => loadDocument(mod.path)}
                >
                  <span className="truncate">{mod.title}</span>
                </div>
              );
            })}
        </div>
      );
    });
  };

  return (
    <div className="h-full flex">
      <aside className="w-56 border-r border-border bg-card overflow-auto shrink-0 scrollbar-thin flex flex-col">
        <div className="px-3 py-3 border-b border-border text-sm font-semibold flex items-center gap-1.5">
          <FileText className="h-4 w-4 text-primary" /> 知识库目录
        </div>
        <div className="py-2 flex-1 overflow-auto">
          
          {renderKnowledgeSections()}
          <div className="px-3 py-1.5 mt-2">
            <div className="border-t border-border pt-2">
              <span className="text-[10px] font-medium text-muted-foreground uppercase tracking-wider">
                章节快速导航
              </span>
            </div>
          </div>
          {mcOutline.map((item) => renderOutlineItem(item))}
        </div>
      </aside>

      <div
        className="flex-1 flex flex-col bg-background"
        onDragOver={(e) => e.preventDefault()}
        onDrop={handleFileDrop}
      >
        <div className="px-4 py-2 border-b border-border flex items-center gap-3 bg-card/50">
          <div className="flex rounded-lg border border-border overflow-hidden">
            <button
              onClick={() => setViewMode("knowledge-base")}
              className={`px-3 py-1.5 text-xs font-medium flex items-center gap-1.5 transition-colors ${
                viewMode === "knowledge-base"
                  ? "bg-accent text-foreground"
                  : "text-muted-foreground hover:text-foreground"
              }`}
            >
              <BookOpen className="h-3.5 w-3.5" />
              知识库文档
            </button>
            <button
              onClick={() => setViewMode("pdf-upload")}
              className={`px-3 py-1.5 text-xs font-medium flex items-center gap-1.5 transition-colors border-l border-border ${
                viewMode === "pdf-upload"
                  ? "bg-accent text-foreground"
                  : "text-muted-foreground hover:text-foreground"
              }`}
            >
              <Upload className="h-3.5 w-3.5" />
              上传PDF
            </button>
          </div>

          {viewMode === "pdf-upload" && (
            <>
              <div className="flex items-center gap-1 text-sm ml-auto">
                <button
                  onClick={() => jumpToPage(currentPage - 1)}
                  className="p-1 rounded hover:bg-accent text-muted-foreground"
                  title="上一页"
                >
                  ←
                </button>
                <input
                  type="number"
                  value={currentPage}
                  onChange={(e) => jumpToPage(parseInt(e.target.value) || 1)}
                  className="w-12 bg-transparent border border-border rounded px-1.5 py-0.5 text-center text-xs"
                />
                <span className="text-muted-foreground text-xs">/ {numPages}</span>
                <button
                  onClick={() => jumpToPage(currentPage + 1)}
                  className="p-1 rounded hover:bg-accent text-muted-foreground"
                  title="下一页"
                >
                  →
                </button>
              </div>
              <div className="flex items-center gap-1">
                <button
                  onClick={() => setScale((s) => Math.max(0.5, s - 0.2))}
                  className="p-1.5 rounded hover:bg-accent text-muted-foreground"
                  title="缩小"
                >
                  <ZoomOut className="h-4 w-4" />
                </button>
                <span className="text-xs text-muted-foreground w-10 text-center">
                  {Math.round(scale * 100)}%
                </span>
                <button
                  onClick={() => setScale((s) => Math.min(3, s + 0.2))}
                  className="p-1.5 rounded hover:bg-accent text-muted-foreground"
                  title="放大"
                >
                  <ZoomIn className="h-4 w-4" />
                </button>
                <button
                  onClick={() => setScale(1.3)}
                  className="p-1.5 rounded hover:bg-accent text-muted-foreground ml-1"
                  title="重置"
                >
                  <RotateCcw className="h-4 w-4" />
                </button>
                <label className="ml-2 px-2 py-1 rounded-lg border border-border text-xs cursor-pointer hover:bg-accent text-muted-foreground">
                  选择文件
                  <input
                    type="file"
                    accept=".pdf"
                    className="hidden"
                    onChange={handleFileInput}
                  />
                </label>
              </div>
            </>
          )}
        </div>

        <div className="flex-1 overflow-auto bg-background">
          {viewMode === "knowledge-base" ? (
            <div className="max-w-3xl mx-auto px-8 py-8">
              {savedToast && (
                <div className="fixed top-4 right-4 z-50 px-4 py-2 rounded-lg bg-green-500/90 text-white text-sm font-medium shadow-lg animate-in fade-in slide-in-from-top-2">
                  ✅ 注释已保存
                </div>
              )}
              {loading && (
                <div className="flex items-center justify-center py-20">
                  <Loader2 className="h-6 w-6 animate-spin text-muted-foreground" />
                  <span className="ml-2 text-sm text-muted-foreground">加载文档中...</span>
                </div>
              )}
              {error && (
                <div className="flex items-center justify-center py-20 text-sm text-red-400">
                  {error}
                </div>
              )}
              {!loading && !error && docContent && (
                <article className="prose prose-slate max-w-none
                  prose-headings:scroll-mt-20
                  prose-headings:font-bold prose-headings:tracking-tight
                  prose-h1:text-3xl prose-h1:mt-0 prose-h1:mb-6 prose-h1:pb-3 prose-h1:border-b-2 prose-h1:border-primary/20
                  prose-h2:text-2xl prose-h2:mt-10 prose-h2:mb-4 prose-h2:pb-2 prose-h2:border-b prose-h2:border-border
                  prose-h3:text-xl prose-h3:mt-8 prose-h3:mb-3 prose-h3:text-primary/80
                  prose-h4:text-lg prose-h4:mt-6 prose-h4:mb-2 prose-h4:text-muted-foreground
                  prose-p:text-foreground/85 prose-p:leading-relaxed prose-p:my-4
                  prose-strong:text-foreground prose-strong:font-bold
                  prose-code:text-primary prose-code:bg-primary/5 prose-code:px-1.5 prose-code:py-0.5 prose-code:rounded prose-code:text-sm prose-code:font-medium prose-code:before:content-none prose-code:after:content-none
                  prose-pre:bg-slate-50 prose-pre:border prose-pre:border-border prose-pre:rounded-xl prose-pre:shadow-sm prose-pre:my-6
                  prose-pre:code:bg-transparent prose-pre:code:text-sm prose-pre:code:leading-relaxed
                  prose-blockquote:border-l-4 prose-blockquote:border-primary/40 prose-blockquote:bg-primary/[0.03] prose-blockquote:py-3 prose-blockquote:px-5 prose-blockquote:rounded-r-lg prose-blockquote:my-6 prose-blockquote:not-italic
                  prose-blockquote:text-foreground/75
                  prose-a:text-primary prose-a:font-medium prose-a:no-underline hover:prose-a:underline prose-a:decoration-primary/30 hover:prose-a:decoration-primary/60 prose-a:underline-offset-2
                  prose-table:text-sm prose-table:my-6
                  prose-th:bg-slate-50 prose-th:border prose-th:border-border prose-th:px-4 prose-th:py-3 prose-th:text-xs prose-th:font-semibold prose-th:text-foreground prose-th:uppercase prose-th:tracking-wider
                  prose-td:border prose-td:border-border prose-td:px-4 prose-td:py-2.5 prose-td:text-sm prose-td:text-foreground/80
                  prose-tr:even:bg-slate-50/50
                  prose-li:text-foreground/80 prose-li:my-1.5 prose-li:leading-relaxed
                  prose-ul:my-4 prose-ol:my-4
                  prose-hr:border-border prose-hr:my-8
                  prose-img:rounded-xl prose-img:shadow-md prose-img:my-6
                  prose-figure:my-6
                  katex-content
                ">
                  <div
                    ref={contentRef}
                    dangerouslySetInnerHTML={{ __html: renderedContent }}
                  />
                  {activeAnnotationId && (
                    <div className="mt-2 p-4 rounded-lg bg-primary/5 border border-primary/20 animate-in fade-in slide-in-from-top-2">
                      <div className="flex items-center justify-between mb-2">
                        <span className="text-xs font-medium text-primary">注释</span>
                        <div className="flex items-center gap-1">
                          {annotationEditMode ? (
                            <>
                              <button onClick={handleSaveAnnotation} className="inline-flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium bg-primary text-primary-foreground hover:bg-primary/90 transition-colors">
                                <Check className="h-3 w-3" /> 保存
                              </button>
                              <button onClick={handleCancelAnnotation} className="inline-flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium border border-border text-muted-foreground hover:text-foreground transition-colors">
                                <X className="h-3 w-3" /> 取消
                              </button>
                            </>
                          ) : (
                            <>
                              <button onClick={() => { setAnnotationEditMode(true); setAnnotationDirty(false) }} className="inline-flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium border border-border text-muted-foreground hover:text-foreground transition-colors">
                                <Pencil className="h-3 w-3" /> 编辑
                              </button>
                              <button onClick={handleDeleteAnnotation} className="inline-flex items-center gap-1 px-2 py-1 rounded text-[11px] font-medium border border-red-500/30 text-red-400 hover:bg-red-500/10 transition-colors">
                                <X className="h-3 w-3" /> 删除
                              </button>
                            </>
                          )}
                        </div>
                      </div>
                      {annotationEditMode ? (
                        <textarea
                          value={annotationDraft}
                          onChange={(e) => { setAnnotationDraft(e.target.value); setAnnotationDirty(true) }}
                          className="w-full min-h-[120px] bg-background border border-border rounded-lg p-3 text-sm text-foreground resize-y focus:outline-none focus:border-primary/50"
                          placeholder="输入你的理解和注释...（支持 Markdown）"
                        />
                      ) : (
                        <div className="text-sm text-muted-foreground leading-relaxed prose-sm dark:prose-invert max-w-none katex-content" dangerouslySetInnerHTML={{ __html: renderMarkdown(annotationDraft) }} />
                      )}
                    </div>
                  )}
                </article>
              )}
              {!loading && !error && !docContent && (
                <div className="flex items-center justify-center py-20 text-sm text-muted-foreground">
                  请从左侧目录选择文档
                </div>
              )}
            </div>
          ) : (
            
            <div className="flex-1 overflow-auto flex justify-center py-8 bg-background">
              <div
                className="bg-white text-gray-900 rounded-lg shadow-xl p-10 whitespace-pre-wrap font-mono text-sm leading-relaxed"
                style={{
                  width: `${210 * 2.5}px`,
                  minHeight: `${297 * 2.5}px`,
                  transform: `scale(${scale})`,
                  transformOrigin: "top center",
                  marginBottom: `${297 * 2.5 * (scale - 1)}px`,
                }}
              >
                {pdfFile ? (
                  <div>
                    <p className="text-center text-gray-500 text-xs mb-4">
                      PDF 文件：{pdfFile.name}（{Math.round(pdfFile.size / 1024)} KB）
                    </p>
                    <p className="text-center text-gray-400 text-xs">
                      PDF 渲染器就绪。实际部署后使用 pdf.js 渲染真实内容。
                    </p>
                  </div>
                ) : (
                  <div>
                    <p className="text-center text-gray-400 text-xs mb-6">
                      —— 拖放 PDF 文件到此区域以加载文档 ——
                    </p>
                    <p className="text-center text-gray-400 text-xs">
                      或点击上方「选择文件」按钮上传 PDF
                    </p>
                  </div>
                )}
              </div>
            </div>
          )}
        </div>
      </div>
    </div>
  );
}

export default function PDFReaderPage() {
  return (
    <AppLayout>
      <Suspense
        fallback={
          <div className="flex items-center justify-center h-full text-muted-foreground">
            <Loader2 className="h-5 w-5 animate-spin mr-2" />
            加载中...
          </div>
        }
      >
        <DocReaderInner />
      </Suspense>
    </AppLayout>
  );
}
