import { NextRequest, NextResponse } from "next/server";
import fs from "fs";
import path from "path";

const NOTES_FILE = path.resolve(
  process.cwd(),
  "..",
  "motor-control-knowledge-base",
  "_annotations",
  "_free_notes.json"
);

interface FreeNote {
  id: string;
  title: string;
  content: string;
  createdAt: string;
  updatedAt: string;
}

function readNotes(): FreeNote[] {
  if (!fs.existsSync(NOTES_FILE)) {
    return [];
  }
  try {
    const raw = fs.readFileSync(NOTES_FILE, "utf-8");
    const data = JSON.parse(raw);
    return data.notes || [];
  } catch {
    return [];
  }
}

function writeNotes(notes: FreeNote[]) {
  const dir = path.dirname(NOTES_FILE);
  if (!fs.existsSync(dir)) {
    fs.mkdirSync(dir, { recursive: true });
  }
  fs.writeFileSync(NOTES_FILE, JSON.stringify({ notes }, null, 2), "utf-8");
}

function generateId(): string {
  return Date.now().toString(36) + Math.random().toString(36).slice(2, 8);
}

export async function GET() {
  const notes = readNotes();
  notes.sort((a, b) => b.createdAt.localeCompare(a.createdAt));
  return NextResponse.json({ notes });
}

export async function POST(req: NextRequest) {
  const body = await req.json();
  const { title, content } = body;

  if (!title || !content) {
    return NextResponse.json(
      { error: "title and content are required" },
      { status: 400 }
    );
  }

  const notes = readNotes();
  const now = new Date().toISOString();
  const note: FreeNote = {
    id: generateId(),
    title,
    content,
    createdAt: now,
    updatedAt: now,
  };

  notes.push(note);
  writeNotes(notes);

  return NextResponse.json({ note });
}

export async function PUT(req: NextRequest) {
  const body = await req.json();
  const { id, title, content } = body;

  if (!id) {
    return NextResponse.json({ error: "id is required" }, { status: 400 });
  }

  const notes = readNotes();
  const index = notes.findIndex((n) => n.id === id);

  if (index === -1) {
    return NextResponse.json({ error: "Note not found" }, { status: 404 });
  }

  if (title !== undefined) {
    notes[index].title = title;
  }
  if (content !== undefined) {
    notes[index].content = content;
  }
  notes[index].updatedAt = new Date().toISOString();

  writeNotes(notes);

  return NextResponse.json({ note: notes[index] });
}

export async function DELETE(req: NextRequest) {
  const body = await req.json();
  const { id } = body;

  if (!id) {
    return NextResponse.json({ error: "id is required" }, { status: 400 });
  }

  const notes = readNotes();
  const index = notes.findIndex((n) => n.id === id);

  if (index === -1) {
    return NextResponse.json({ error: "Note not found" }, { status: 404 });
  }

  notes.splice(index, 1);
  writeNotes(notes);

  return NextResponse.json({ success: true });
}