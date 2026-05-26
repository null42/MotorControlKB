# 电控学习平台停止脚本 (Windows)

Write-Host "停止服务..." -ForegroundColor Yellow

# 停止后端
Get-Process -Name python -ErrorAction SilentlyContinue | Stop-Process -Force
Get-Process -Name uvicorn -ErrorAction SilentlyContinue | Stop-Process -Force

# 停止前端
Get-Process -Name node -ErrorAction SilentlyContinue | Stop-Process -Force

Write-Host "服务已停止" -ForegroundColor Green
