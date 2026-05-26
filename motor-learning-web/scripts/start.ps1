# 电控学习平台启动脚本 (Windows)

$ErrorActionPreference = "Stop"

$ProjectRoot = Split-Path -Parent $PSScriptRoot
$BackendDir = Join-Path $ProjectRoot "backend"
$FrontendDir = Join-Path $ProjectRoot "frontend"

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "  电控学习平台启动脚本" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# 检查 Python
Write-Host "`n[1/4] 检查 Python 环境..." -ForegroundColor Yellow
try {
    $pythonVersion = python --version 2>&1
    Write-Host "  找到 $pythonVersion" -ForegroundColor Green
} catch {
    Write-Host "  错误: 未找到 Python，请先安装 Python 3.9+" -ForegroundColor Red
    exit 1
}

# 检查 Node.js
Write-Host "`n[2/4] 检查 Node.js 环境..." -ForegroundColor Yellow
try {
    $nodeVersion = node --version 2>&1
    Write-Host "  找到 Node.js $nodeVersion" -ForegroundColor Green
} catch {
    Write-Host "  错误: 未找到 Node.js，请先安装 Node.js 18+" -ForegroundColor Red
    exit 1
}

# 安装后端依赖
Write-Host "`n[3/4] 安装后端依赖..." -ForegroundColor Yellow
Set-Location $BackendDir
if (-not (Test-Path "venv")) {
    Write-Host "  创建虚拟环境..." -ForegroundColor Gray
    python -m venv venv
}
Write-Host "  激活虚拟环境并安装依赖..." -ForegroundColor Gray
.\venv\Scripts\Activate.ps1
pip install -r requirements.txt -q

# 安装前端依赖
Write-Host "`n[4/4] 安装前端依赖..." -ForegroundColor Yellow
Set-Location $FrontendDir
if (-not (Test-Path "node_modules")) {
    Write-Host "  运行 npm install..." -ForegroundColor Gray
    npm install --silent
}

# 启动服务
Write-Host "`n========================================" -ForegroundColor Cyan
Write-Host "  启动服务..." -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan

# 启动后端
Write-Host "`n启动后端服务 (端口 8000)..." -ForegroundColor Green
Set-Location $BackendDir
Start-Process -FilePath "powershell" -ArgumentList "-NoExit", "-Command", ".\venv\Scripts\Activate.ps1; uvicorn app.main:app --reload --host 0.0.0.0 --port 8000" -WindowStyle Normal

# 等待后端启动
Start-Sleep -Seconds 3

# 启动前端
Write-Host "启动前端服务 (端口 3000)..." -ForegroundColor Green
Set-Location $FrontendDir
Start-Process -FilePath "powershell" -ArgumentList "-NoExit", "-Command", "npm run dev" -WindowStyle Normal

Write-Host "`n========================================" -ForegroundColor Green
Write-Host "  服务已启动!" -ForegroundColor Green
Write-Host "========================================" -ForegroundColor Green
Write-Host "`n  前端地址: http://localhost:3000" -ForegroundColor Cyan
Write-Host "  后端地址: http://localhost:8000" -ForegroundColor Cyan
Write-Host "  API文档:  http://localhost:8000/docs" -ForegroundColor Cyan
Write-Host "`n  按 Ctrl+C 停止服务" -ForegroundColor Yellow
