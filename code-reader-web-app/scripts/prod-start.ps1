#!/usr/bin/env pwsh

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Code Reader Web App - Production Start" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

$BackendDir = Join-Path $ProjectRoot "backend"
$FrontendDir = Join-Path $ProjectRoot "frontend"

if (-not (Test-Path "$BackendDir\venv")) {
    Write-Host "? Virtual environment not found. Please run install.ps1 first." -ForegroundColor Red
    exit 1
}

Write-Host "Building frontend..." -ForegroundColor Yellow
Set-Location $FrontendDir
npm run build

Write-Host "Starting production server on http://localhost:8000" -ForegroundColor Yellow
Set-Location $BackendDir
& ".\venv\Scripts\Activate.ps1"
uvicorn app.main:app --host 0.0.0.0 --port 8000
