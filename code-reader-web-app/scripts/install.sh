#!/bin/bash

echo "========================================"
echo "Code Reader Web App - Installation Script"
echo "========================================"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

check_command() {
    if ! command -v "$1" &> /dev/null; then
        echo "? $1 is not installed"
        return 1
    else
        echo "? $1 is installed: $(command -v "$1")"
        return 0
    fi
}

check_python() {
    if command -v python3 &> /dev/null; then
        PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
        echo "? Python version: $PYTHON_VERSION"
        return 0
    elif command -v python &> /dev/null; then
        PYTHON_VERSION=$(python --version 2>&1 | awk '{print $2}')
        echo "? Python version: $PYTHON_VERSION"
        return 0
    else
        echo "? Python is not installed"
        return 1
    fi
}

check_nodejs() {
    if command -v node &> /dev/null; then
        NODE_VERSION=$(node --version)
        echo "? Node.js version: $NODE_VERSION"
        return 0
    else
        echo "? Node.js is not installed"
        return 1
    fi
}

echo ""
echo "Checking prerequisites..."
echo "-------------------------"

PYTHON_OK=true
NODE_OK=true

if ! check_python; then
    PYTHON_OK=false
    echo "   Please install Python 3.9+ from https://www.python.org/downloads/"
fi

if ! check_nodejs; then
    NODE_OK=false
    echo "   Please install Node.js 18+ from https://nodejs.org/"
fi

if [ "$PYTHON_OK" = false ] || [ "$NODE_OK" = false ]; then
    echo ""
    echo "? Prerequisites not met. Please install missing dependencies."
    exit 1
fi

echo ""
echo "Setting up Python virtual environment..."
echo "----------------------------------------"

cd "$PROJECT_ROOT/backend"

if [ ! -d "venv" ]; then
    python3 -m venv venv || python -m venv venv
    echo "? Virtual environment created"
else
    echo "? Virtual environment already exists"
fi

source venv/bin/activate

echo ""
echo "Installing Python dependencies..."
echo "----------------------------------"

pip install --upgrade pip
pip install -r requirements.txt

echo "? Python dependencies installed"

echo ""
echo "Setting up frontend..."
echo "----------------------"

cd "$PROJECT_ROOT/frontend"

if [ ! -f "package.json" ]; then
    echo "Initializing Vue 3 project..."
    npm create vite@latest . -- --template vue-ts --force
fi

if [ -f "package.json" ]; then
    npm install
    echo "? Frontend dependencies installed"
fi

echo ""
echo "Checking LSP servers..."
echo "-----------------------"

if command -v clangd &> /dev/null; then
    echo "? clangd is available: $(clangd --version | head -1)"
else
    echo "??  clangd is not installed (optional, for C/C++ support)"
    echo "   Install with: apt install clangd (Ubuntu) or brew install llvm (macOS)"
fi

if command -v pylsp &> /dev/null; then
    echo "? pylsp is available"
else
    echo "??  pylsp is not installed (optional, for Python support)"
    echo "   Install with: pip install python-lsp-server"
fi

echo ""
echo "========================================"
echo "? Installation complete!"
echo "========================================"
echo ""
echo "To start the application, run:"
echo "  ./scripts/start.sh"
echo ""
