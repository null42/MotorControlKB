import { NextRequest, NextResponse } from "next/server";
import fs from "fs";
import path from "path";

const KB_DIR = path.resolve(process.cwd(), "..", "motor-control-knowledge-base");
const ANNOTATIONS_DIR = path.join(KB_DIR, "_annotations");

interface SectionAnnotation {
  sectionId: string;
  sectionTitle: string;
  content: string;
  updatedAt: string;
}

interface DocAnnotations {
  annotations: SectionAnnotation[];
}

function resolveAnnotationPath(docPath: string): string {
  const fullPath = path.join(ANNOTATIONS_DIR, docPath + ".json");
  const resolved = path.resolve(fullPath);
  const baseResolved = path.resolve(ANNOTATIONS_DIR);
  const relative = path.relative(baseResolved, resolved);
  if (relative.startsWith("..") || path.isAbsolute(relative)) {
    throw new Error("Access denied");
  }
  return resolved;
}

function ensureDir(filePath: string): void {
  const dir = path.dirname(filePath);
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
  }
}

function readAnnotations(docPath: string): DocAnnotations {
  const filePath = resolveAnnotationPath(docPath);
  if (!fs.existsSync(filePath)) {
    return { annotations: [] };
  }
  const raw = fs.readFileSync(filePath, "utf-8");
  return JSON.parse(raw);
}

function writeAnnotations(docPath: string, data: DocAnnotations): void {
  const filePath = resolveAnnotationPath(docPath);
  ensureDir(filePath);
  fs.writeFileSync(filePath, JSON.stringify(data, null, 2), "utf-8");
}

function deleteAnnotationsFile(docPath: string): void {
  const filePath = resolveAnnotationPath(docPath);
  if (fs.existsSync(filePath)) {
    fs.unlinkSync(filePath);
  }
}

function scanDir(dir: string, results: string[]): void {
  if (!fs.existsSync(dir)) {
    return;
  }
  const entries = fs.readdirSync(dir, { withFileTypes: true });
  for (const entry of entries) {
    const fullPath = path.join(dir, entry.name);
    if (entry.isDirectory()) {
      scanDir(fullPath, results);
    } else if (entry.isFile() && entry.name.endsWith(".json") && entry.name !== "_free_notes.json") {
      results.push(fullPath);
    }
  }
}

export async function GET(req: NextRequest) {
  const docPath = req.nextUrl.searchParams.get("docPath");

  if (docPath) {
    try {
      const data = readAnnotations(docPath);
      return NextResponse.json(data);
    } catch {
      return NextResponse.json({ error: "Access denied" }, { status: 403 });
    }
  }

  const allJsonFiles: string[] = [];
  scanDir(ANNOTATIONS_DIR, allJsonFiles);

  const aggregated: Array<SectionAnnotation & { docPath: string }> = [];

  for (const filePath of allJsonFiles) {
    const relativePath = path.relative(ANNOTATIONS_DIR, filePath);
    const extractedDocPath = relativePath.replace(/\.json$/, "");
    const raw = fs.readFileSync(filePath, "utf-8");
    const data: DocAnnotations = JSON.parse(raw);
    for (const annotation of data.annotations) {
      aggregated.push({
        docPath: extractedDocPath,
        ...annotation,
      });
    }
  }

  return NextResponse.json({ annotations: aggregated });
}

export async function PUT(req: NextRequest) {
  try {
    const body = await req.json();
    const { docPath, sectionId, sectionTitle, content } = body;

    if (!docPath || !sectionId) {
      return NextResponse.json({ error: "docPath and sectionId are required" }, { status: 400 });
    }

    const data = readAnnotations(docPath);

    if (content === "") {
      data.annotations = data.annotations.filter((a) => a.sectionId !== sectionId);
    } else {
      const idx = data.annotations.findIndex((a) => a.sectionId === sectionId);
      const entry: SectionAnnotation = {
        sectionId,
        sectionTitle: sectionTitle || "",
        content,
        updatedAt: new Date().toISOString(),
      };
      if (idx >= 0) {
        data.annotations[idx] = entry;
      } else {
        data.annotations.push(entry);
      }
    }

    writeAnnotations(docPath, data);
    return NextResponse.json({ success: true });
  } catch {
    return NextResponse.json({ error: "Access denied" }, { status: 403 });
  }
}

export async function DELETE(req: NextRequest) {
  try {
    const body = await req.json();
    const { docPath, sectionId } = body;

    if (!docPath || !sectionId) {
      return NextResponse.json({ error: "docPath and sectionId are required" }, { status: 400 });
    }

    const data = readAnnotations(docPath);
    data.annotations = data.annotations.filter((a) => a.sectionId !== sectionId);

    if (data.annotations.length === 0) {
      deleteAnnotationsFile(docPath);
    } else {
      writeAnnotations(docPath, data);
    }

    return NextResponse.json({ success: true });
  } catch {
    return NextResponse.json({ error: "Access denied" }, { status: 403 });
  }
}