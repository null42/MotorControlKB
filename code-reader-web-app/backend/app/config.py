from pydantic_settings import BaseSettings
from typing import List
import os

class Settings(BaseSettings):
    PROJECTS_DIR: str = os.path.join(os.path.dirname(__file__), "..", "..", "data", "projects")
    DATABASE_URL: str = os.path.join(os.path.dirname(__file__), "..", "..", "data", "db", "annotations.db")
    CORS_ORIGINS: List[str] = ["http://localhost:5173", "http://127.0.0.1:5173"]
    MAX_UPLOAD_SIZE: int = 100 * 1024 * 1024
    
    class Config:
        env_file = ".env"

settings = Settings()
