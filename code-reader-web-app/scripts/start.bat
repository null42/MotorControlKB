@echo off
echo ========================================
echo Code Reader Web App - Starting Services
echo ========================================
echo.

set SCRIPT_DIR=%~dp0
set PROJECT_ROOT=%SCRIPT_DIR%..

cd /d "%PROJECT_ROOT%\backend"

if not exist "venv" (
    echo [ERROR] Virtual environment not found, please run install.bat first
    pause
    exit /b 1
)

echo Starting backend server (port 8000)...
start "Code Reader Backend" cmd /k "call venv\Scripts\activate.bat && uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload"

cd /d "%PROJECT_ROOT%\frontend"

echo Starting frontend server (port 5173)...
start "Code Reader Frontend" cmd /k "npm run dev"

echo.
echo ========================================
echo [DONE] Services started!
echo ========================================
echo.
echo Backend API:  http://localhost:8000
echo API Docs:     http://localhost:8000/docs
echo Frontend:     http://localhost:5173
echo.
echo To stop services, run: scripts\stop.bat
echo.
pause