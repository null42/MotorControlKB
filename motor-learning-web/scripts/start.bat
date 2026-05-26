@echo off
chcp 65001 >nul
title Motor Learning Platform

set PROJECT_ROOT=%~dp0..

echo ========================================
echo   Motor Learning Platform Starting...
echo ========================================
echo   Project Path: %PROJECT_ROOT%
echo ========================================

:: Check Python
where python >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not found. Please install Python 3.8+
    pause
    exit /b 1
)

:: Check Node.js
where npm >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Node.js not found. Please install Node.js 16+
    pause
    exit /b 1
)

:: Check backend virtual environment
if not exist "%PROJECT_ROOT%\backend\venv\Scripts\activate.bat" (
    echo [1/2] Creating backend virtual environment...
    cd /d "%PROJECT_ROOT%\backend"
    python -m venv venv
    call venv\Scripts\activate.bat
    pip install -r requirements.txt -q
    cd /d "%PROJECT_ROOT%"
)

:: Check frontend dependencies
if not exist "%PROJECT_ROOT%\frontend\node_modules" (
    echo [2/2] Installing frontend dependencies...
    cd /d "%PROJECT_ROOT%\frontend"
    call npm install --silent
    cd /d "%PROJECT_ROOT%"
)

echo.
echo ========================================
echo   Starting Services...
echo ========================================

:: Start backend service (new window)
echo Starting backend server (port 8000)...
start "Backend-Port8000" cmd /k "cd /d %PROJECT_ROOT%\backend && call venv\Scripts\activate.bat && uvicorn app.main:app --host 0.0.0.0 --port 8000"

:: Wait for backend to start
echo Waiting for backend...
timeout /t 4 /nobreak >nul

:: Start frontend service (new window)
echo Starting frontend server (port 3000)...
start "Frontend-Port3000" cmd /k "cd /d %PROJECT_ROOT%\frontend && npm run dev"

:: Wait for frontend to start
echo Waiting for frontend...
timeout /t 5 /nobreak >nul

echo.
echo ========================================
echo   Services Started!
echo ========================================
echo.
echo   Frontend: http://localhost:3000
echo   Backend:  http://localhost:8000
echo   API Docs: http://localhost:8000/docs
echo.
echo   Tip: Close Backend/Frontend window to stop service
echo   Tip: This window can be safely closed
echo ========================================

:: Open browser
echo Opening browser...
start "" http://localhost:3000

echo.
echo Browser opened. If not, visit: http://localhost:3000
echo.
pause
