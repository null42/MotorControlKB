#!/bin/bash

echo "========================================"
echo "Code Reader Web App - Stopping Services"
echo "========================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

if [ -f "$PROJECT_ROOT/.backend.pid" ]; then
    BACKEND_PID=$(cat "$PROJECT_ROOT/.backend.pid")
    if kill -0 $BACKEND_PID 2>/dev/null; then
        kill $BACKEND_PID
        echo "? Backend stopped (PID: $BACKEND_PID)"
    else
        echo "??  Backend process not running"
    fi
    rm "$PROJECT_ROOT/.backend.pid"
else
    echo "??  No backend PID file found"
fi

if [ -f "$PROJECT_ROOT/.frontend.pid" ]; then
    FRONTEND_PID=$(cat "$PROJECT_ROOT/.frontend.pid")
    if kill -0 $FRONTEND_PID 2>/dev/null; then
        kill $FRONTEND_PID
        echo "? Frontend stopped (PID: $FRONTEND_PID)"
    else
        echo "??  Frontend process not running"
    fi
    rm "$PROJECT_ROOT/.frontend.pid"
else
    echo "??  No frontend PID file found"
fi

pkill -f "uvicorn app.main:app" 2>/dev/null
pkill -f "vite" 2>/dev/null

echo ""
echo "? All services stopped"
