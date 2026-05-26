#!/usr/bin/env pwsh

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Code Reader Web App - Stopping Services" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

$BackendPidFile = "$ProjectRoot\.backend.pid"
$FrontendPidFile = "$ProjectRoot\.frontend.pid"

if (Test-Path $BackendPidFile) {
    $BackendJobId = Get-Content $BackendPidFile
    $Job = Get-Job -Id $BackendJobId -ErrorAction SilentlyContinue
    if ($Job) {
        Stop-Job -Id $BackendJobId -ErrorAction SilentlyContinue
        Remove-Job -Id $BackendJobId -ErrorAction SilentlyContinue
        Write-Host "? Backend stopped (Job ID: $BackendJobId)" -ForegroundColor Green
    } else {
        Write-Host "??  Backend job not found" -ForegroundColor Yellow
    }
    Remove-Item $BackendPidFile -ErrorAction SilentlyContinue
} else {
    Write-Host "??  No backend PID file found" -ForegroundColor Yellow
}

if (Test-Path $FrontendPidFile) {
    $FrontendJobId = Get-Content $FrontendPidFile
    $Job = Get-Job -Id $FrontendJobId -ErrorAction SilentlyContinue
    if ($Job) {
        Stop-Job -Id $FrontendJobId -ErrorAction SilentlyContinue
        Remove-Job -Id $FrontendJobId -ErrorAction SilentlyContinue
        Write-Host "? Frontend stopped (Job ID: $FrontendJobId)" -ForegroundColor Green
    } else {
        Write-Host "??  Frontend job not found" -ForegroundColor Yellow
    }
    Remove-Item $FrontendPidFile -ErrorAction SilentlyContinue
} else {
    Write-Host "??  No frontend PID file found" -ForegroundColor Yellow
}

Get-Process -Name "uvicorn" -ErrorAction SilentlyContinue | Stop-Process -ErrorAction SilentlyContinue
Get-Process -Name "node" -ErrorAction SilentlyContinue | Where-Object { $_.CommandLine -like "*vite*" } | Stop-Process -ErrorAction SilentlyContinue

Write-Host ""
Write-Host "? All services stopped" -ForegroundColor Green
