from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
import os

from app.api import docs
from app.api import quiz
from app.api import simulation
from app.api import annotations
from app.api import serial_proxy

app = FastAPI(title="Motor Learning API")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(docs.router, prefix="/api/docs", tags=["docs"])
app.include_router(quiz.router, prefix="/api/quiz", tags=["quiz"])
app.include_router(simulation.router, prefix="/api")
app.include_router(annotations.router, prefix="/api/annotations", tags=["annotations"])
app.include_router(serial_proxy.router, prefix="/api/serial", tags=["serial"])

BASE_DIR = os.path.dirname(os.path.dirname(__file__))
SIMS_DIR = os.path.join(os.path.dirname(BASE_DIR), "sims")
static_dir = os.path.join(BASE_DIR, "static")
if os.path.exists(static_dir):
    assets_dir = os.path.join(static_dir, "assets")
    fonts_dir = os.path.join(static_dir, "fonts")
    if os.path.exists(assets_dir):
        app.mount("/assets", StaticFiles(directory=assets_dir), name="assets")
    if os.path.exists(fonts_dir):
        app.mount("/fonts", StaticFiles(directory=fonts_dir), name="fonts")


@app.get("/")
async def root():
    index_path = os.path.join(BASE_DIR, "static", "index.html")
    if os.path.exists(index_path):
        return FileResponse(index_path)
    return {"message": "Motor Learning API"}


@app.get("/sims/{filename:path}")
async def serve_sim(filename: str):
    file_path = os.path.join(SIMS_DIR, filename)
    if os.path.isfile(file_path):
        return FileResponse(file_path)
    raise HTTPException(status_code=404, detail="Not found")


@app.get("/{filename:path}")
async def serve_static(filename: str):
    file_path = os.path.join(static_dir, filename)
    if os.path.isfile(file_path):
        return FileResponse(file_path)
    raise HTTPException(status_code=404, detail="Not found")
