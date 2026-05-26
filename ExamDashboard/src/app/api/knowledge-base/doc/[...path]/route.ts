import { NextRequest, NextResponse } from "next/server";
import fs from "fs";
import path from "path";

const KB_DIR = path.resolve(
  process.cwd(),
  "..",
  "motor-control-knowledge-base"
);

export async function GET(
  _req: NextRequest,
  { params }: { params: { path: string[] } }
) {
  const filePath = params.path.join("/");

  if (!filePath.endsWith(".md")) {
    return NextResponse.json({ error: "Only markdown files are supported" }, { status: 400 });
  }

  const fullPath = path.join(KB_DIR, filePath);
  const resolved = path.resolve(fullPath);
  const baseResolved = path.resolve(KB_DIR);

  const relative = path.relative(baseResolved, resolved);
  if (relative.startsWith("..") || path.isAbsolute(relative)) {
    return NextResponse.json({ error: "Access denied" }, { status: 403 });
  }

  if (!fs.existsSync(resolved)) {
    return NextResponse.json({ error: "Document not found" }, { status: 404 });
  }

  const content = fs.readFileSync(resolved, "utf-8");
  const title = path.basename(filePath, ".md");

  return NextResponse.json({
    title,
    path: filePath,
    content,
  });
}
