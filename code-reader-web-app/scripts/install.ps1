#!/usr/bin/env pwsh

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Code Reader Web App - Installation Script" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir

function Check-Command {
    param($Command)
    if (Get-Command $Command -ErrorAction SilentlyContinue) {
        Write-Host "? $Command is installed: $(Get-Command $Command).Source" -ForegroundColor Green
        return $true
    } else {
        Write-Host "? $Command is not installed" -ForegroundColor Red
        return $false
    }
}

function Check-Python {
    if (Get-Command python -ErrorAction SilentlyContinue) {
        $Version = python --version 2>&1
        Write-Host "? Python version: $Version" -ForegroundColor Green
        return $true
    } elseif (Get-Command python3 -ErrorAction SilentlyContinue) {
        $Version = python3 --version 2>&1
        Write-Host "? Python version: $Version" -ForegroundColor Green
        return $true
    } else {
        Write-Host "? Python is not installed" -ForegroundColor Red
        return $false
    }
}

function Check-NodeJS {
    if (Get-Command node -ErrorAction SilentlyContinue) {
        $Version = node --version
        Write-Host "? Node.js version: $Version" -ForegroundColor Green
        return $true
    } else {
        Write-Host "? Node.js is not installed" -ForegroundColor Red
        return $false
    }
}

Write-Host ""
Write-Host "Checking prerequisites..." -ForegroundColor Yellow
Write-Host "-------------------------"

$PythonOK = Check-Python
$NodeOK = Check-NodeJS

if (-not $PythonOK -or -not $NodeOK) {
    Write-Host ""
    Write-Host "? Prerequisites not met." -ForegroundColor Red
    if (-not $PythonOK) {
        Write-Host "   Please install Python 3.9+ from https://www.python.org/downloads/" -ForegroundColor Yellow
    }
    if (-not $NodeOK) {
        Write-Host "   Please install Node.js 18+ from https://nodejs.org/" -ForegroundColor Yellow
    }
    exit 1
}

Write-Host ""
Write-Host "Setting up Python virtual environment..." -ForegroundColor Yellow
Write-Host "----------------------------------------"

$BackendDir = Join-Path $ProjectRoot "backend"
Set-Location $BackendDir

if (-not (Test-Path "venv")) {
    python -m venv venv
    Write-Host "? Virtual environment created" -ForegroundColor Green
} else {
    Write-Host "? Virtual environment already exists" -ForegroundColor Green
}

Write-Host ""
Write-Host "Installing Python dependencies..." -ForegroundColor Yellow
Write-Host "----------------------------------"

& ".\venv\Scripts\Activate.ps1"
python -m pip install --upgrade pip
pip install -r requirements.txt

Write-Host "? Python dependencies installed" -ForegroundColor Green

Write-Host ""
Write-Host "Setting up frontend..." -ForegroundColor Yellow
Write-Host "----------------------"

$FrontendDir = Join-Path $ProjectRoot "frontend"
Set-Location $FrontendDir

if (-not (Test-Path "package.json")) {
    Write-Host "Initializing Vue 3 project..." -ForegroundColor Yellow
    npm create vite@latest . -- --template vue-ts --force
}

if (Test-Path "package.json") {
    npm install
    Write-Host "? Frontend dependencies installed" -ForegroundColor Green
}

Write-Host ""
Write-Host "Checking LSP servers..." -ForegroundColor Yellow
Write-Host "-----------------------"

if (Get-Command clangd -ErrorAction SilentlyContinue) {
    Write-Host "? clangd is available" -ForegroundColor Green
} else {
    Write-Host "??  clangd is not installed (optional, for C/C++ support)" -ForegroundColor Yellow
    Write-Host "   Install LLVM from https://releases.llvm.org/" -ForegroundColor Yellow
}

if (Get-Command pylsp -ErrorAction SilentlyContinue) {
    Write-Host "? pylsp is available" -ForegroundColor Green
} else {
    Write-Host "??  pylsp is not installed (optional, for Python support)" -ForegroundColor Yellow
    Write-Host "   Install with: pip install python-lsp-server" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "? Installation complete!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""
Write-Host "To start the application, run:" -ForegroundColor White
Write-Host "  .\scripts\start.ps1" -ForegroundColor Cyan
Write-Host ""
