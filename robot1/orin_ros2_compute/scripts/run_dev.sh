#!/bin/bash
# Development mode: Run Orin ROS2 compute AI service in foreground

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ORIN_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$ORIN_DIR/src"

# Activate Python virtual environment if exists
if [ -d "$ORIN_DIR/venv" ]; then
    source "$ORIN_DIR/venv/bin/activate"
fi

# Load environment variables (robustly handle comments and spaces)
if [ -f "$ORIN_DIR/config/.env" ]; then
    set -a
    . "$ORIN_DIR/config/.env"
    set +a
fi

# Set defaults
export NANO_IP=${NANO_IP:-"10.13.68.184"}
export NANO_PORT=${NANO_PORT:-8000}
export OLLAMA_NO_GPU=${OLLAMA_NO_GPU:-1}

# Check if Ollama is running
if ! curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "⚠️  Ollama not running. Starting Ollama in background..."
    OLLAMA_NO_GPU=$OLLAMA_NO_GPU ollama serve &
    sleep 3
fi

# Run AI service
cd "$SRC_DIR"
python3 robot1_ai_service_realtime.py --simulation
