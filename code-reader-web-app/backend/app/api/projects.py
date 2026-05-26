from fastapi import APIRouter, UploadFile, File, HTTPException, Form
from fastapi.responses import JSONResponse
from typing import Optional
import os
import shutil
import zipfile
import uuid
import json
from pathlib import Path
from pydantic import BaseModel

router = APIRouter()

PROJECTS_DIR = Path(__file__).parent.parent.parent.parent / "data" / "projects"
PROJECTS_DIR.mkdir(parents=True, exist_ok=True)

class ProjectInfo(BaseModel):
    id: str
    name: str
    path: str
    file_count: int
    created_at: str

class ProjectCreate(BaseModel):
    name: str


def count_files(directory: Path) -> int:
    count = 0
    for item in directory.rglob("*"):
        if item.is_file():
            count += 1
    return count


def get_project_info(project_id: str) -> Optional[ProjectInfo]:
    meta_file = PROJECTS_DIR / project_id / ".project_meta.json"
    if meta_file.exists():
        with open(meta_file, "r", encoding="utf-8") as f:
            data = json.load(f)
            return ProjectInfo(**data)
    return None


@router.post("/upload/zip")
async def upload_zip(file: UploadFile = File(...), name: Optional[str] = Form(None)):
    if not file.filename or not file.filename.endswith(".zip"):
        raise HTTPException(status_code=400, detail="Only ZIP files are supported")
    
    project_id = str(uuid.uuid4())[:8]
    project_name = name or Path(file.filename).stem
    project_path = PROJECTS_DIR / project_id
    
    try:
        project_path.mkdir(parents=True, exist_ok=True)
        
        temp_zip = project_path / "temp.zip"
        with open(temp_zip, "wb") as f:
            content = await file.read()
            f.write(content)
        
        with zipfile.ZipFile(temp_zip, "r") as zip_ref:
            zip_ref.extractall(project_path)
        
        temp_zip.unlink()
        
        extracted_items = list(project_path.iterdir())
        if len(extracted_items) == 1 and extracted_items[0].is_dir():
            inner_dir = extracted_items[0]
            for item in inner_dir.iterdir():
                shutil.move(str(item), str(project_path))
            inner_dir.rmdir()
        
        file_count = count_files(project_path)
        
        meta = {
            "id": project_id,
            "name": project_name,
            "path": str(project_path),
            "file_count": file_count,
            "created_at": __import__("datetime").datetime.now().isoformat()
        }
        
        with open(project_path / ".project_meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        
        return {"success": True, "project": meta}
    
    except Exception as e:
        if project_path.exists():
            shutil.rmtree(project_path)
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/upload/folder")
async def upload_folder(files: list[UploadFile] = File(...), name: str = Form(...)):
    project_id = str(uuid.uuid4())[:8]
    project_path = PROJECTS_DIR / project_id
    
    try:
        project_path.mkdir(parents=True, exist_ok=True)
        
        for file in files:
            file_path = project_path / file.filename
            file_path.parent.mkdir(parents=True, exist_ok=True)
            content = await file.read()
            with open(file_path, "wb") as f:
                f.write(content)
        
        file_count = count_files(project_path)
        
        meta = {
            "id": project_id,
            "name": name,
            "path": str(project_path),
            "file_count": file_count,
            "created_at": __import__("datetime").datetime.now().isoformat()
        }
        
        with open(project_path / ".project_meta.json", "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)
        
        return {"success": True, "project": meta}
    
    except Exception as e:
        if project_path.exists():
            shutil.rmtree(project_path)
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/list")
async def list_projects():
    projects = []
    for item in PROJECTS_DIR.iterdir():
        if item.is_dir():
            meta_file = item / ".project_meta.json"
            if meta_file.exists():
                with open(meta_file, "r", encoding="utf-8") as f:
                    projects.append(json.load(f))
    return {"projects": projects}


@router.get("/{project_id}")
async def get_project(project_id: str):
    info = get_project_info(project_id)
    if not info:
        raise HTTPException(status_code=404, detail="Project not found")
    return info


@router.delete("/{project_id}")
async def delete_project(project_id: str):
    project_path = PROJECTS_DIR / project_id
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    shutil.rmtree(project_path)
    return {"success": True, "message": f"Project {project_id} deleted"}
