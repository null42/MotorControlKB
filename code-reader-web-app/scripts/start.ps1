#!/usr/bin/env pwsh

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Code Reader Web App - Starting Services" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

$BackendDir = Join-Path $ProjectRoot "backend"
$FrontendDir = Join-Path $ProjectRoot "frontend"

if (-not (Test-Path "$BackendDir\venv")) {
    Write-Host "? Virtual environment not found. Please run install.ps1 first." -ForegroundColor Red
    exit 1
}

Write-Host "Starting backend server on http://localhost:8000" -ForegroundColor Yellow

$BackendJob = Start-Job -ScriptBlock {
    param($BackendDir)
    Set-Location $BackendDir
    & ".\venv\Scripts\Activate.ps1"
    uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload
} -ArgumentList $BackendDir

$BackendJob.Id | Out-File -FilePath "$ProjectRoot\.backend.pid" -Encoding utf8

Write-Host "Starting frontend dev server on http://localhost:5173" -ForegroundColor Yellow

$FrontendJob = Start-Job -ScriptBlock {
    param($FrontendDir)
    Set-Location $FrontendDir
    npm run dev
} -ArgumentList $FrontendDir

$FrontendJob.Id | Out-File -FilePath "$ProjectRoot\.frontend.pid" -Encoding utf8

Start-Sleep -Seconds 3

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "? Services started!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "Backend API:  http://localhost:8000" -ForegroundColor White
Write-Host "API Docs:     http://localhost:8000/docs" -ForegroundColor White
Write-Host "Frontend:     http://localhost:5173" -ForegroundColor White
Write-Host ""
Write-Host "To stop services, run: .\scripts\stop.ps1" -ForegroundColor Yellow
Write-Host ""

Write-Host "Press Ctrl+C to stop all services..." -ForegroundColor Yellow

try {
    while ($true) {
        Start-Sleep -Seconds 1
    }
} finally {
    Write-Host "Stopping services..." -ForegroundColor Yellow
    Stop-Job -Id $BackendJob.Id -ErrorAction SilentlyContinue
    Stop-Job -Id $FrontendJob.Id -ErrorAction SilentlyContinue
    Remove-Job -Id $BackendJob.Id -ErrorAction SilentlyContinue
    Remove-Job -Id $FrontendJob.Id -ErrorAction SilentlyContinue
    Write-Host "Services stopped." -ForegroundColor Green
}
