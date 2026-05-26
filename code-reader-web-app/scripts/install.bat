@echo off
echo ========================================
echo Code Reader Web App - Installation
echo ========================================
echo.

set SCRIPT_DIR=%~dp0
set PROJECT_ROOT=%SCRIPT_DIR%..

echo Checking Python...
python --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Python not installed, please download from https://www.python.org/downloads/
    pause
    exit /b 1
)
echo [OK] Python installed

echo Checking Node.js...
node --version >nul 2>&1
if errorlevel 1 (
    echo [ERROR] Node.js not installed, please download from https://nodejs.org/
    pause
    exit /b 1
)
echo [OK] Node.js installed

echo.
echo Setting up Python virtual environment...
cd /d "%PROJECT_ROOT%\backend"

if not exist "venv" (
    python -m venv venv
    echo [OK] Virtual environment created
) else (
    echo [OK] Virtual environment exists
)

echo.
echo Installing Python dependencies...
call venv\Scripts\activate.bat
python -m pip install --upgrade pip
pip install -r requirements.txt
echo [OK] Python dependencies installed

echo.
echo Installing frontend dependencies...
cd /d "%PROJECT_ROOT%\frontend"
if exist "package.json" (
    npm install
    echo [OK] Frontend dependencies installed
) else (
    echo [WARNING] package.json not found
)

echo.
echo ========================================
echo [DONE] Installation successful!
echo ========================================
echo.
echo To start the app, run: scripts\start.bat
echo.
pause