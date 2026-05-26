@echo off
echo ========================================
echo Code Reader Web App - Stopping Services
echo ========================================
echo.

echo Stopping backend service...
taskkill /f /fi "WINDOWTITLE eq Code Reader Backend*" >nul 2>&1

echo Stopping frontend service...
taskkill /f /fi "WINDOWTITLE eq Code Reader Frontend*" >nul 2>&1

echo Stopping other related processes...
taskkill /f /im python.exe /fi "WINDOWTITLE eq *uvicorn*" >nul 2>&1
taskkill /f /im node.exe /fi "WINDOWTITLE eq *vite*" >nul 2>&1

echo.
echo ========================================
echo [DONE] All services stopped
echo ========================================
echo.
pause