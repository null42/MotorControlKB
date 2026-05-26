#!/bin/bash

# 电控学习平台启动脚本 (Linux/macOS)

set -e

ProjectRoot="$(cd "$(dirname "$0")/.." && pwd)"
BackendDir="$ProjectRoot/backend"
FrontendDir="$ProjectRoot/frontend"

echo "========================================"
echo "  电控学习平台启动脚本"
echo "========================================"

# 检查 Python
echo -e "\n[1/4] 检查 Python 环境..."
if command -v python3 &> /dev/null; then
    echo "  找到 $(python3 --version)"
else
    echo "  错误: 未找到 Python，请先安装 Python 3.9+"
    exit 1
fi

# 检查 Node.js
echo -e "\n[2/4] 检查 Node.js 环境..."
if command -v node &> /dev/null; then
    echo "  找到 Node.js $(node --version)"
else
    echo "  错误: 未找到 Node.js，请先安装 Node.js 18+"
    exit 1
fi

# 安装后端依赖
echo -e "\n[3/4] 安装后端依赖..."
cd "$BackendDir"
if [ ! -d "venv" ]; then
    echo "  创建虚拟环境..."
    python3 -m venv venv
fi
echo "  激活虚拟环境并安装依赖..."
source venv/bin/activate
pip install -r requirements.txt -q

# 安装前端依赖
echo -e "\n[4/4] 安装前端依赖..."
cd "$FrontendDir"
if [ ! -d "node_modules" ]; then
    echo "  运行 npm install..."
    npm install --silent
fi

# 启动服务
echo -e "\n========================================"
echo "  启动服务..."
echo "========================================"

# 启动后端
echo -e "\n启动后端服务 (端口 8000)..."
cd "$BackendDir"
source venv/bin/activate
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000 &
BACKEND_PID=$!

# 等待后端启动
sleep 3

# 启动前端
echo "启动前端服务 (端口 3000)..."
cd "$FrontendDir"
npm run dev &
FRONTEND_PID=$!

echo -e "\n========================================"
echo "  服务已启动!"
echo "========================================"
echo -e "\n  前端地址: http://localhost:3000"
echo "  后端地址: http://localhost:8000"
echo "  API文档:  http://localhost:8000/docs"
echo -e "\n  按 Ctrl+C 停止服务"

# 等待子进程
trap "kill $BACKEND_PID $FRONTEND_PID 2>/dev/null" EXIT
wait
