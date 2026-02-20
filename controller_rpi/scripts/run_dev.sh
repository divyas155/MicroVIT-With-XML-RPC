#!/bin/bash
# Development mode: Run Controller AI service in foreground

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONTROLLER_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$CONTROLLER_DIR/src"

# Activate Python virtual environment if exists
if [ -d "$CONTROLLER_DIR/venv" ]; then
    source "$CONTROLLER_DIR/venv/bin/activate"
fi

# Load environment variables
if [ -f "$CONTROLLER_DIR/config/.env" ]; then
    export $(cat "$CONTROLLER_DIR/config/.env" | grep -v '^#' | xargs)
fi

# Set defaults
export OLLAMA_NO_GPU=${OLLAMA_NO_GPU:-1}

# Check if Ollama is running
if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "⚠️  Ollama not running. Starting Ollama in background..."
    OLLAMA_NO_GPU=$OLLAMA_NO_GPU ollama serve &
    sleep 3
fi

# Run controller service
cd "$SRC_DIR"
python3 controller_ai_service.py
