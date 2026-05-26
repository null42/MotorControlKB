@echo off
chcp 65001 >nul 2>&1
setlocal enabledelayedexpansion

title MotorControl Knowledge Base

set "SCRIPT_DIR=%~dp0"
set "FRONTEND_DIR=%SCRIPT_DIR%motor-learning-web\frontend"
set "BACKEND_DIR=%SCRIPT_DIR%motor-learning-web\backend"
set "VENV_DIR=%BACKEND_DIR%\venv"
set "HOST=127.0.0.1"
set "PORT=8001"
set "URL=http://%HOST%:%PORT%"

echo.
echo  ================================================
echo    MotorControl Knowledge Base - Starting...
echo  ================================================
echo.

echo  [0/4] Checking if frontend build is needed...
echo.

set "INDEX_HTML=%BACKEND_DIR%\static\index.html"
set "NEED_BUILD=0"

if not exist "%INDEX_HTML%" (
    echo    [X] No index.html found - will build.
    set "NEED_BUILD=1"
) else (
    for %%i in ("%INDEX_HTML%") do set "IDX_TS=%%~ti"
    for /r "%FRONTEND_DIR%\src" %%f in (*.vue *.ts *.tsx *.js *.jsx *.css *.scss) do (
        for %%s in ("%%f") do set "SRC_TS=%%~ts"
        if "!SRC_TS!" gtr "!IDX_TS!" (
            echo    [X] Modified: %%~nxf
            set "NEED_BUILD=1"
        )
    )
)

if "!NEED_BUILD!"=="1" (
    echo.
    echo    --- Building frontend ---
    echo.
    cd /d "%FRONTEND_DIR%" || (
        echo  [ERROR] Cannot cd to %FRONTEND_DIR%
        pause & exit /b 1
    )
    call npm run build || (
        echo  [ERROR] Build failed - fix the error above and retry.
        pause & exit /b 1
    )
    echo.
    echo  [OK] Build complete.
    echo.
) else (
    echo    [OK] Frontend up-to-date, skipping build.
    echo.
)

echo  [1/4] Checking virtual environment...
if not exist "%VENV_DIR%\Scripts\python.exe" (
    echo.
    echo  [ERROR] Python virtual environment not found!
    echo    Path: %VENV_DIR%
    echo    Run: cd /d "%BACKEND_DIR%" ^& python -m venv venv ^& venv\Scripts\pip install -r requirements.txt
    echo.
    pause
    exit /b 1
)
echo    Virtual environment: OK

echo  [2/4] Checking port %PORT%...
netstat -ano | findstr ":%PORT% " | findstr "LISTENING" >nul 2>&1
if not errorlevel 1 (
    echo    Port in use - releasing...
    for /f "tokens=5" %%a in ('netstat -ano ^| findstr ":%PORT% " ^| findstr "LISTENING"') do taskkill /PID %%a /F >nul 2>&1
    timeout /t 2 /nobreak >nul
    netstat -ano | findstr ":%PORT% " | findstr "LISTENING" >nul 2>&1
    if not errorlevel 1 (
        echo  [ERROR] Port %PORT% still occupied! Manually: netstat -ano ^| findstr :%PORT%
        pause & exit /b 1
    )
    echo    Port released: OK
)
echo    Port %PORT%: Available

echo  [3/4] Starting backend server...
cd /d "%BACKEND_DIR%" || (pause & exit /b 1)
start "MC-KnowledgeBase-Server" cmd /k ""%VENV_DIR%\Scripts\python.exe" -m uvicorn app.main:app --host %HOST% --port %PORT%"

echo  [4/4] Waiting for server...
set READY=0
for /L %%i in (1,1,20) do (
    if !READY! equ 0 (
        timeout /t 1 /nobreak >nul
        "%VENV_DIR%\Scripts\python.exe" -c "import urllib.request; urllib.request.urlopen('http://%HOST%:%PORT%/')" >nul 2>&1
        if not errorlevel 1 (set READY=1 & echo    Server ready! ^(%%i sec^))
    )
)

if %READY% equ 0 (
    echo  [WARN] Server not ready within 20s, opening browser anyway...
    start "" "%URL%"
    pause & exit /b 1
)

echo.
echo  ================================================
echo    Started successfully!   %URL%
echo  ================================================
echo    Close the server window to stop.

start "" "%URL%"
echo  Browser opened.
echo.
echo  This window can be safely closed.
echo.
pause