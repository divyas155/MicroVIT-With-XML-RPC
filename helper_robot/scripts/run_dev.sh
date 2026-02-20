#!/bin/bash
# Development mode: Run Helper Robot AI service in foreground

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HELPER_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$HELPER_DIR/src"

# Activate Python virtual environment if exists
if [ -d "$HELPER_DIR/venv" ]; then
    source "$HELPER_DIR/venv/bin/activate"
fi

# Load environment variables
if [ -f "$HELPER_DIR/config/.env" ]; then
    export $(cat "$HELPER_DIR/config/.env" | grep -v '^#' | xargs)
fi

# Set defaults
export OLLAMA_NO_GPU=${OLLAMA_NO_GPU:-1}

# Check if Ollama is running
if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "⚠️  Ollama not running. Starting Ollama in background..."
    OLLAMA_NO_GPU=$OLLAMA_NO_GPU ollama serve &
    sleep 3
fi

# Run helper robot service
cd "$SRC_DIR"
python3 helper_robot_ai_service.py
