from fastapi import APIRouter, HTTPException
from typing import List, Optional
from pathlib import Path
import json
from pydantic import BaseModel

router = APIRouter()

PROJECTS_DIR = Path(__file__).parent.parent.parent.parent / "data" / "projects"


class FileNode(BaseModel):
    name: str
    path: str
    type: str
    children: Optional[List["FileNode"]] = None


FileNode.model_rebuild()


class FileContent(BaseModel):
    path: str
    content: str
    language: str


def detect_language(filename: str) -> str:
    ext_map = {
        ".c": "c",
        ".h": "c",
        ".cpp": "cpp",
        ".hpp": "cpp",
        ".cc": "cpp",
        ".cxx": "cpp",
        ".py": "python",
        ".js": "javascript",
        ".ts": "typescript",
        ".json": "json",
        ".xml": "xml",
        ".html": "html",
        ".css": "css",
        ".md": "markdown",
        ".txt": "plaintext",
        ".sh": "shell",
        ".yaml": "yaml",
        ".yml": "yaml",
        ".toml": "toml",
        ".rs": "rust",
        ".go": "go",
        ".java": "java",
        ".kt": "kotlin",
        ".swift": "swift",
    }
    ext = Path(filename).suffix.lower()
    return ext_map.get(ext, "plaintext")


def build_file_tree(directory: Path, base_path: Path) -> List[FileNode]:
    nodes = []
    try:
        items = sorted(directory.iterdir(), key=lambda x: (not x.is_dir(), x.name.lower()))
        for item in items:
            if item.name.startswith("."):
                continue
            relative_path = str(item.relative_to(base_path))
            if item.is_dir():
                children = build_file_tree(item, base_path)
                nodes.append(FileNode(
                    name=item.name,
                    path=relative_path,
                    type="directory",
                    children=children
                ))
            else:
                nodes.append(FileNode(
                    name=item.name,
                    path=relative_path,
                    type="file"
                ))
    except PermissionError:
        pass
    return nodes


@router.get("/{project_id}/tree")
async def get_file_tree(project_id: str):
    project_path = PROJECTS_DIR / project_id
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    tree = build_file_tree(project_path, project_path)
    return {"tree": [node.model_dump() for node in tree]}


@router.get("/{project_id}/file")
async def get_file_content(project_id: str, path: str):
    project_path = PROJECTS_DIR / project_id
    file_path = project_path / path
    
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="File not found")
    
    if not str(file_path.resolve()).startswith(str(project_path.resolve())):
        raise HTTPException(status_code=403, detail="Access denied")
    
    try:
        encodings = ['utf-8', 'gbk', 'gb2312', 'utf-16', 'latin-1']
        content = None
        last_error = None
        
        for encoding in encodings:
            try:
                with open(file_path, "r", encoding=encoding) as f:
                    content = f.read()
                break
            except (UnicodeDecodeError, UnicodeError) as e:
                last_error = e
                continue
        
        if content is None:
            try:
                with open(file_path, "rb") as f:
                    raw_bytes = f.read()
                content = raw_bytes.decode('utf-8', errors='replace')
            except Exception as e:
                raise HTTPException(status_code=400, detail=f"Cannot read file: {str(e)}")
        
        language = detect_language(file_path.name)
        
        return FileContent(
            path=path,
            content=content,
            language=language
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{project_id}/search")
async def search_files(project_id: str, query: str):
    project_path = PROJECTS_DIR / project_id
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    results = []
    query_lower = query.lower()
    
    for file_path in project_path.rglob("*"):
        if file_path.is_file() and not file_path.name.startswith("."):
            if query_lower in file_path.name.lower():
                relative_path = str(file_path.relative_to(project_path))
                results.append({
                    "name": file_path.name,
                    "path": relative_path,
                    "type": "file"
                })
    
    return {"results": results}
