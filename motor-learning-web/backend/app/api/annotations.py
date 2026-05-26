from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from pathlib import Path
import os
import json
import uuid
from datetime import datetime, timezone
from typing import List, Optional

router = APIRouter()

BASE_DIR = Path(__file__).resolve().parent.parent.parent.parent.parent
ANNOTATIONS_DIR = BASE_DIR / "motor-control-knowledge-base" / "_annotations"


def _now_iso() -> str:
    return datetime.now(timezone.utc).isoformat()


def _secure_path(doc_path: str) -> Path:
    sanitized = Path(doc_path)
    file_path = ANNOTATIONS_DIR / sanitized.with_suffix(".json")
    resolved = file_path.resolve()
    base_resolved = ANNOTATIONS_DIR.resolve()
    relative = os.path.relpath(str(resolved), str(base_resolved))
    if relative.startswith("..") or os.path.isabs(relative):
        raise HTTPException(status_code=403, detail="Access denied")
    return file_path


class AnnotationSave(BaseModel):
    docPath: str
    sectionId: str
    sectionTitle: str
    content: str


class AnnotationDelete(BaseModel):
    docPath: str
    sectionId: str


class NoteCreate(BaseModel):
    title: str
    content: str


class NoteUpdate(BaseModel):
    id: str
    title: str
    content: str


class NoteDelete(BaseModel):
    id: str


_FREE_NOTES_FILE = ANNOTATIONS_DIR / "_free_notes.json"


def _read_json(file_path: Path) -> dict:
    if file_path.exists():
        return json.loads(file_path.read_text(encoding="utf-8"))
    return {}


def _write_json(file_path: Path, data: dict):
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


@router.get("/")
async def get_all_annotations(docPath: Optional[str] = None):
    if docPath:
        file_path = _secure_path(docPath)
        data = _read_json(file_path)
        return {"annotations": data.get("annotations", [])}

    ANNOTATIONS_DIR.mkdir(parents=True, exist_ok=True)
    all_annotations = []
    for file_path in ANNOTATIONS_DIR.rglob("*.json"):
        if file_path.name == "_free_notes.json":
            continue
        data = _read_json(file_path)
        relative = file_path.relative_to(ANNOTATIONS_DIR)
        doc_path = str(relative.with_suffix("")).replace("\\", "/")
        for ann in data.get("annotations", []):
            ann["docPath"] = doc_path
            all_annotations.append(ann)

    return {"annotations": all_annotations}


@router.put("/")
async def save_annotation(body: AnnotationSave):
    file_path = _secure_path(body.docPath)
    data = _read_json(file_path)

    annotations: list = data.get("annotations", [])

    if not body.content:
        annotations = [a for a in annotations if a.get("sectionId") != body.sectionId]
    else:
        existing = None
        for a in annotations:
            if a.get("sectionId") == body.sectionId:
                existing = a
                break
        if existing:
            existing["content"] = body.content
            existing["sectionTitle"] = body.sectionTitle
            existing["updatedAt"] = _now_iso()
        else:
            annotations.append({
                "sectionId": body.sectionId,
                "sectionTitle": body.sectionTitle,
                "content": body.content,
                "updatedAt": _now_iso(),
            })

    data["annotations"] = annotations
    _write_json(file_path, data)
    return {"success": True}


@router.delete("/")
async def delete_annotation(body: AnnotationDelete):
    file_path = _secure_path(body.docPath)
    data = _read_json(file_path)

    annotations: list = data.get("annotations", [])
    annotations = [a for a in annotations if a.get("sectionId") != body.sectionId]

    if not annotations:
        if file_path.exists():
            file_path.unlink()
    else:
        data["annotations"] = annotations
        _write_json(file_path, data)

    return {"success": True}


@router.get("/notes")
async def get_all_notes():
    data = _read_json(_FREE_NOTES_FILE)
    notes = data.get("notes", [])
    notes.sort(key=lambda n: n.get("createdAt", ""), reverse=True)
    return {"notes": notes}


@router.post("/notes")
async def create_note(body: NoteCreate):
    data = _read_json(_FREE_NOTES_FILE)
    notes: list = data.get("notes", [])
    now = _now_iso()
    note = {
        "id": str(uuid.uuid4()),
        "title": body.title,
        "content": body.content,
        "createdAt": now,
        "updatedAt": now,
    }
    notes.append(note)
    data["notes"] = notes
    _write_json(_FREE_NOTES_FILE, data)
    return {"note": note}


@router.put("/notes")
async def update_note(body: NoteUpdate):
    data = _read_json(_FREE_NOTES_FILE)
    notes: list = data.get("notes", [])
    for note in notes:
        if note.get("id") == body.id:
            note["title"] = body.title
            note["content"] = body.content
            note["updatedAt"] = _now_iso()
            _write_json(_FREE_NOTES_FILE, data)
            return {"note": note}
    raise HTTPException(status_code=404, detail="Note not found")


@router.delete("/notes")
async def delete_note(body: NoteDelete):
    data = _read_json(_FREE_NOTES_FILE)
    notes: list = data.get("notes", [])
    notes = [n for n in notes if n.get("id") != body.id]
    data["notes"] = notes
    _write_json(_FREE_NOTES_FILE, data)
    return {"success": True}