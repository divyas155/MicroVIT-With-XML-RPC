#!/bin/bash
# Restart AI Service with correct configuration
# Run this on Orin after running check_ollama.sh

echo "=========================================="
echo "Restarting AI Service with Full Configuration"
echo "=========================================="
echo ""

# Check if .env exists
ENV_FILE="$HOME/MicroVIT/robot1/orin_ros2_compute/.env"
if [ ! -f "$ENV_FILE" ]; then
    echo "❌ .env file not found at $ENV_FILE"
    echo "   Run ~/check_ollama.sh first to create it"
    exit 1
fi

echo "✅ Found .env file"
echo ""

# Load environment variables
echo "Loading environment variables from .env..."
export $(cat "$ENV_FILE" | grep -v '^#' | xargs)

echo "Configuration:"
echo "  OLLAMA_MODEL: ${OLLAMA_MODEL:-not set}"
echo "  USE_MICROVIT: ${USE_MICROVIT:-not set}"
echo "  NANO_IP: ${NANO_IP:-not set}"
echo "  NANO_PORT: ${NANO_PORT:-not set}"
echo ""

# Check if venv exists
VENV_DIR="$HOME/MicroVIT/robot1/orin_ros2_compute/venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "⚠️  Virtual environment not found. Creating..."
    cd "$HOME/MicroVIT/robot1/orin_ros2_compute"
    python3 -m venv venv
    source venv/bin/activate
    pip install -r requirements.txt
else
    echo "✅ Virtual environment found"
    source "$VENV_DIR/bin/activate"
fi

echo ""
echo "Starting AI service..."
echo ""

cd "$HOME/MicroVIT/robot1/orin_ros2_compute/src"
python3 robot1_ai_service_realtime.py --simulation
