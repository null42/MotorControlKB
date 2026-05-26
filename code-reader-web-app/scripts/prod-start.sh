#!/bin/bash

echo "========================================"
echo "Code Reader Web App - Production Start"
echo "========================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT/backend"

if [ ! -d "venv" ]; then
    echo "? Virtual environment not found. Please run install.sh first."
    exit 1
fi

source venv/bin/activate

echo "Building frontend..."
cd "$PROJECT_ROOT/frontend"
npm run build

echo "Starting production server on http://localhost:8000"
cd "$PROJECT_ROOT/backend"
uvicorn app.main:app --host 0.0.0.0 --port 8000
