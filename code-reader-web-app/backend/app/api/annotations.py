from fastapi import APIRouter, HTTPException
from typing import Optional
from pathlib import Path
from pydantic import BaseModel
from datetime import datetime
import sqlite3
import uuid

router = APIRouter()

DB_PATH = Path(__file__).parent.parent.parent.parent / "data" / "db" / "annotations.db"


class HighlightCreate(BaseModel):
    project_id: str
    file_path: str
    start_line: int
    start_column: int
    end_line: int
    end_column: int
    color: str


class AnnotationCreate(BaseModel):
    project_id: str
    file_path: str
    line_number: int
    content: str


class AnnotationUpdate(BaseModel):
    content: str


def get_db():
    DB_PATH.parent.mkdir(parents=True, exist_ok=True)
    conn = sqlite3.connect(str(DB_PATH))
    conn.row_factory = sqlite3.Row
    return conn


def init_db():
    conn = get_db()
    cursor = conn.cursor()
    
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS highlights (
            id TEXT PRIMARY KEY,
            project_id TEXT NOT NULL,
            file_path TEXT NOT NULL,
            start_line INTEGER NOT NULL,
            start_column INTEGER NOT NULL,
            end_line INTEGER NOT NULL,
            end_column INTEGER NOT NULL,
            color TEXT NOT NULL,
            created_at TEXT NOT NULL
        )
    """)
    
    cursor.execute("""
        CREATE TABLE IF NOT EXISTS annotations (
            id TEXT PRIMARY KEY,
            project_id TEXT NOT NULL,
            file_path TEXT NOT NULL,
            line_number INTEGER NOT NULL,
            content TEXT NOT NULL,
            created_at TEXT NOT NULL,
            updated_at TEXT NOT NULL
        )
    """)
    
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_highlights_project ON highlights(project_id)")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_highlights_file ON highlights(project_id, file_path)")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_annotations_project ON annotations(project_id)")
    cursor.execute("CREATE INDEX IF NOT EXISTS idx_annotations_file ON annotations(project_id, file_path)")
    
    try:
        cursor.execute("ALTER TABLE highlights ADD COLUMN start_column INTEGER DEFAULT 1")
    except sqlite3.OperationalError:
        pass
    try:
        cursor.execute("ALTER TABLE highlights ADD COLUMN end_column INTEGER DEFAULT 1")
    except sqlite3.OperationalError:
        pass
    
    conn.commit()
    conn.close()


init_db()


@router.post("/highlights")
async def create_highlight(highlight: HighlightCreate):
    conn = get_db()
    cursor = conn.cursor()
    
    highlight_id = str(uuid.uuid4())[:8]
    created_at = datetime.now().isoformat()
    
    cursor.execute(
        "INSERT INTO highlights VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)",
        (highlight_id, highlight.project_id, highlight.file_path,
         highlight.start_line, highlight.start_column,
         highlight.end_line, highlight.end_column,
         highlight.color, created_at)
    )
    conn.commit()
    conn.close()
    
    return {
        "success": True, 
        "highlight": {
            "id": highlight_id,
            "project_id": highlight.project_id,
            "file_path": highlight.file_path,
            "start_line": highlight.start_line,
            "start_column": highlight.start_column,
            "end_line": highlight.end_line,
            "end_column": highlight.end_column,
            "color": highlight.color,
            "created_at": created_at
        }
    }


@router.get("/highlights/{project_id}")
async def get_highlights(project_id: str, file_path: Optional[str] = None):
    conn = get_db()
    cursor = conn.cursor()
    
    if file_path:
        cursor.execute(
            "SELECT * FROM highlights WHERE project_id = ? AND file_path = ?",
            (project_id, file_path)
        )
    else:
        cursor.execute("SELECT * FROM highlights WHERE project_id = ?", (project_id,))
    
    rows = cursor.fetchall()
    conn.close()
    
    highlights = []
    for row in rows:
        h = dict(row)
        if 'start_column' not in h or h['start_column'] is None:
            h['start_column'] = 1
        if 'end_column' not in h or h['end_column'] is None:
            h['end_column'] = 1
        highlights.append(h)
    
    return {"highlights": highlights}


@router.delete("/highlights/{highlight_id}")
async def delete_highlight(highlight_id: str):
    conn = get_db()
    cursor = conn.cursor()
    
    cursor.execute("DELETE FROM highlights WHERE id = ?", (highlight_id,))
    affected = cursor.rowcount
    conn.commit()
    conn.close()
    
    if affected == 0:
        raise HTTPException(status_code=404, detail="Highlight not found")
    
    return {"success": True}


@router.post("/annotations")
async def create_annotation(annotation: AnnotationCreate):
    conn = get_db()
    cursor = conn.cursor()
    
    annotation_id = str(uuid.uuid4())[:8]
    now = datetime.now().isoformat()
    
    cursor.execute(
        "INSERT INTO annotations VALUES (?, ?, ?, ?, ?, ?, ?)",
        (annotation_id, annotation.project_id, annotation.file_path,
         annotation.line_number, annotation.content, now, now)
    )
    conn.commit()
    conn.close()
    
    return {
        "success": True, 
        "annotation": {
            "id": annotation_id,
            "project_id": annotation.project_id,
            "file_path": annotation.file_path,
            "line_number": annotation.line_number,
            "content": annotation.content,
            "created_at": now,
            "updated_at": now
        }
    }


@router.get("/annotations/{project_id}")
async def get_annotations(project_id: str, file_path: Optional[str] = None):
    conn = get_db()
    cursor = conn.cursor()
    
    if file_path:
        cursor.execute(
            "SELECT * FROM annotations WHERE project_id = ? AND file_path = ?",
            (project_id, file_path)
        )
    else:
        cursor.execute("SELECT * FROM annotations WHERE project_id = ?", (project_id,))
    
    rows = cursor.fetchall()
    conn.close()
    
    annotations = [dict(row) for row in rows]
    return {"annotations": annotations}


@router.put("/annotations/{annotation_id}")
async def update_annotation(annotation_id: str, body: AnnotationUpdate):
    conn = get_db()
    cursor = conn.cursor()
    
    now = datetime.now().isoformat()
    
    cursor.execute(
        "UPDATE annotations SET content = ?, updated_at = ? WHERE id = ?",
        (body.content, now, annotation_id)
    )
    affected = cursor.rowcount
    conn.commit()
    conn.close()
    
    if affected == 0:
        raise HTTPException(status_code=404, detail="Annotation not found")
    
    return {"success": True, "updated_at": now}


@router.delete("/annotations/{annotation_id}")
async def delete_annotation(annotation_id: str):
    conn = get_db()
    cursor = conn.cursor()
    
    cursor.execute("DELETE FROM annotations WHERE id = ?", (annotation_id,))
    affected = cursor.rowcount
    conn.commit()
    conn.close()
    
    if affected == 0:
        raise HTTPException(status_code=404, detail="Annotation not found")
    
    return {"success": True}
