from fastapi import APIRouter, HTTPException
from typing import List, Optional, Dict, Any
from pathlib import Path
from pydantic import BaseModel
import subprocess
import json
import os
import asyncio

router = APIRouter()

PROJECTS_DIR = Path(__file__).parent.parent.parent.parent / "data" / "projects"


class DefinitionRequest(BaseModel):
    project_id: str
    file_path: str
    line: int
    character: int


class ReferenceRequest(BaseModel):
    project_id: str
    file_path: str
    line: int
    character: int


class SymbolSearchRequest(BaseModel):
    project_id: str
    query: str


class Location(BaseModel):
    file_path: str
    line: int
    character: int


class DefinitionResult(BaseModel):
    location: Optional[Location] = None
    found: bool = False


class ReferenceResult(BaseModel):
    locations: List[Location] = []


class SymbolResult(BaseModel):
    name: str
    kind: str
    location: Location


LSP_PROCESSes: Dict[str, Any] = {}


async def get_lsp_client(project_id: str, project_path: Path):
    if project_id not in LSP_PROCESSes:
        has_cpp = any(project_path.rglob("*.c")) or any(project_path.rglob("*.cpp"))
        has_py = any(project_path.rglob("*.py"))
        
        if has_cpp:
            process = await asyncio.create_subprocess_exec(
                "clangd",
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            LSP_PROCESSes[project_id] = {"process": process, "language": "cpp"}
        elif has_py:
            process = await asyncio.create_subprocess_exec(
                "pylsp",
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            LSP_PROCESSes[project_id] = {"process": process, "language": "python"}
        else:
            return None
    
    return LSP_PROCESSes.get(project_id)


@router.post("/definition")
async def get_definition(request: DefinitionRequest):
    project_path = PROJECTS_DIR / request.project_id
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    return {
        "found": False,
        "location": None,
        "message": "LSP integration requires clangd or pylsp to be installed"
    }


@router.post("/references")
async def get_references(request: ReferenceRequest):
    project_path = PROJECTS_DIR / request.project_id
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    return {
        "locations": [],
        "message": "LSP integration requires clangd or pylsp to be installed"
    }


@router.post("/symbols")
async def search_symbols(request: SymbolSearchRequest):
    project_path = PROJECTS_DIR / request.project_id
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    results = []
    query_lower = request.query.lower()
    
    text_extensions = {".c", ".h", ".cpp", ".hpp", ".py", ".js", ".ts", ".java", ".go", ".rs"}
    
    for file_path in project_path.rglob("*"):
        if file_path.is_file() and file_path.suffix.lower() in text_extensions:
            try:
                with open(file_path, "r", encoding="utf-8") as f:
                    lines = f.readlines()
                
                for line_num, line in enumerate(lines, 1):
                    if query_lower in line.lower():
                        relative_path = str(file_path.relative_to(project_path))
                        results.append({
                            "name": query_lower,
                            "kind": "text_match",
                            "location": {
                                "file_path": relative_path,
                                "line": line_num,
                                "character": line.lower().find(query_lower) + 1
                            },
                            "preview": line.strip()[:100]
                        })
            except (UnicodeDecodeError, PermissionError):
                continue
    
    return {"symbols": results[:50]}


@router.get("/call-graph/{project_id}")
async def get_call_graph(project_id: str, function_name: Optional[str] = None):
    project_path = PROJECTS_DIR / project_id
    if not project_path.exists():
        raise HTTPException(status_code=404, detail="Project not found")
    
    return {
        "nodes": [],
        "edges": [],
        "message": "Call graph requires LSP integration with clangd"
    }
