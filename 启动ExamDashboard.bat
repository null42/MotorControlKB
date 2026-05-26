@echo off
chcp 65001 >nul 2>&1
setlocal enabledelayedexpansion

title MotorControl ExamDashboard

set "SCRIPT_DIR=%~dp0"
set "PROJECT_DIR=%SCRIPT_DIR%ExamDashboard"
set "HOST=127.0.0.1"
set "PORT=3000"
set "URL=http://%HOST%:%PORT%"

echo.
echo  ================================================
echo    MotorControl ExamDashboard - Starting...
echo  ================================================
echo.

echo  [1/3] Checking project dependencies...
if not exist "%PROJECT_DIR%\node_modules\" (
    echo.
    echo  [INFO] node_modules not found, running npm install...
    echo    Directory: %PROJECT_DIR%
    echo.
    cd /d "%PROJECT_DIR%"
    call npm install
    if errorlevel 1 (
        echo.
        echo  [ERROR] npm install failed!
        echo    Please check your Node.js installation.
        echo.
        pause
        exit /b 1
    )
    echo    npm install: OK
) else (
    echo    node_modules: OK
)

echo  [2/3] Checking port %PORT%...
netstat -ano | findstr ":%PORT% " | findstr "LISTENING" >nul 2>&1
if not errorlevel 1 (
    echo.
    echo  [WARN] Port %PORT% is in use!
    echo    Trying to release...
    echo.
    for /f "tokens=5" %%a in ('netstat -ano ^| findstr ":%PORT% " ^| findstr "LISTENING"') do (
        taskkill /PID %%a /F >nul 2>&1
    )
    timeout /t 2 /nobreak >nul
    netstat -ano | findstr ":%PORT% " | findstr "LISTENING" >nul 2>&1
    if not errorlevel 1 (
        echo  [ERROR] Port %PORT% still in use!
        echo    Run manually: netstat -ano ^| findstr :%PORT%
        echo    Then: taskkill /PID PID_NUMBER /F
        echo.
        pause
        exit /b 1
    )
    echo    Port released: OK
)
echo    Port %PORT%: Available

echo  [3/3] Starting Next.js dev server...
echo    Working dir: %PROJECT_DIR%
echo    URL: %URL%
echo.

cd /d "%PROJECT_DIR%"
start "MC-ExamDashboard-Server" cmd /k "npx next dev -H %HOST% -p %PORT% && pause"

echo.
echo  ================================================
echo    ExamDashboard is starting...
echo  ================================================
echo.
echo    URL: %URL%
echo.
echo    Waiting for server to be ready...
echo.

set READY=0
for /L %%i in (1,1,30) do (
    if !READY! equ 0 (
        timeout /t 1 /nobreak >nul
        curl -s -o nul "%URL%" >nul 2>&1
        if not errorlevel 1 (
            set READY=1
            echo    Server ready! (%%i sec)
        )
    )
)

if %READY% equ 0 (
    echo.
    echo  [WARN] Server not ready within 30s
    echo    First startup may take longer due to compilation.
    echo    Check the server window for progress.
    echo.
    echo  Attempting to open browser anyway...
    start "" "%URL%"
    echo.
    pause
    exit /b 0
)

echo.
echo  ================================================
echo    Started successfully!
echo  ================================================
echo.
echo    URL: %URL%
echo    Close the server window to stop
echo.

start "" "%URL%"

echo  Browser opened. If not, visit: %URL%
echo.
echo  This window can be safely closed.
echo.
pause
