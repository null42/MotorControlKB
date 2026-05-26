#!/bin/bash

echo "========================================"
echo "Code Reader Web App - Starting Services"
echo "========================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT/backend"

if [ ! -d "venv" ]; then
    echo "? Virtual environment not found. Please run install.sh first."
    exit 1
fi

source venv/bin/activate

echo "Starting backend server on http://localhost:8000"
uvicorn app.main:app --host 0.0.0.0 --port 8000 --reload &
BACKEND_PID=$!
echo $BACKEND_PID > "$PROJECT_ROOT/.backend.pid"

cd "$PROJECT_ROOT/frontend"

echo "Starting frontend dev server on http://localhost:5173"
npm run dev &
FRONTEND_PID=$!
echo $FRONTEND_PID > "$PROJECT_ROOT/.frontend.pid"

echo ""
echo "========================================"
echo "? Services started!"
echo "========================================"
echo ""
echo "Backend API:  http://localhost:8000"
echo "API Docs:     http://localhost:8000/docs"
echo "Frontend:     http://localhost:5173"
echo ""
echo "To stop services, run: ./scripts/stop.sh"
echo ""

wait
