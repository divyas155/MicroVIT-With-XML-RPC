#!/bin/bash
# Check and fix Ollama model availability
# Run this on Orin

echo "=========================================="
echo "Ollama Model Diagnostic and Fix"
echo "=========================================="
echo ""

# 1. Check if Ollama is running
echo "1. Checking Ollama service..."
if curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
    echo "   ✅ Ollama is running"
else
    echo "   ❌ Ollama is not running"
    echo "   Starting Ollama..."
    OLLAMA_NO_GPU=1 ollama serve > /tmp/ollama.log 2>&1 &
    sleep 3
    if curl -s http://localhost:11434/api/tags > /dev/null 2>&1; then
        echo "   ✅ Ollama started"
    else
        echo "   ❌ Failed to start Ollama"
        exit 1
    fi
fi
echo ""

# 2. List available models
echo "2. Checking available models..."
MODELS=$(ollama list 2>/dev/null | tail -n +2 | awk '{print $1}' | grep -v "^$" || echo "")
if [ -z "$MODELS" ]; then
    echo "   ⚠️  No models found"
    MODELS_AVAILABLE="no"
else
    echo "   Available models:"
    echo "$MODELS" | while read model; do
        echo "     - $model"
    done
    MODELS_AVAILABLE="yes"
fi
echo ""

# 3. Check for expected models
echo "3. Checking for expected models..."
EXPECTED_MODELS=("qwen2.5:0.5b" "qwen2:0.5b" "phi3:mini" "llama3.2:1b" "tinyllama")
FOUND_MODEL=""

for model in "${EXPECTED_MODELS[@]}"; do
    if echo "$MODELS" | grep -q "^${model}$"; then
        echo "   ✅ Found: $model"
        FOUND_MODEL="$model"
        break
    fi
done

if [ -z "$FOUND_MODEL" ]; then
    echo "   ❌ None of the expected models found"
    echo ""
    echo "4. Pulling recommended model..."
    echo "   Attempting to pull qwen2.5:0.5b..."
    if ollama pull qwen2.5:0.5b 2>&1 | tee /tmp/ollama_pull.log; then
        FOUND_MODEL="qwen2.5:0.5b"
        echo "   ✅ Successfully pulled qwen2.5:0.5b"
    else
        echo "   ⚠️  Failed to pull qwen2.5:0.5b, trying alternatives..."
        
        # Try qwen2:0.5b
        echo "   Trying qwen2:0.5b..."
        if ollama pull qwen2:0.5b 2>&1 | tee /tmp/ollama_pull.log; then
            FOUND_MODEL="qwen2:0.5b"
            echo "   ✅ Successfully pulled qwen2:0.5b"
        else
            # Try phi3:mini
            echo "   Trying phi3:mini..."
            if ollama pull phi3:mini 2>&1 | tee /tmp/ollama_pull.log; then
                FOUND_MODEL="phi3:mini"
                echo "   ✅ Successfully pulled phi3:mini"
            else
                echo "   ❌ Failed to pull any model"
                echo "   Check logs: cat /tmp/ollama_pull.log"
                exit 1
            fi
        fi
    fi
else
    echo "   ✅ Using existing model: $FOUND_MODEL"
fi
echo ""

# 5. Test the model
echo "5. Testing model: $FOUND_MODEL"
TEST_RESPONSE=$(curl -s http://localhost:11434/api/generate -d "{
  \"model\": \"$FOUND_MODEL\",
  \"prompt\": \"Hello\",
  \"stream\": false
}" 2>&1)

if echo "$TEST_RESPONSE" | grep -q '"response"'; then
    echo "   ✅ Model is working!"
    echo "   Test response: $(echo "$TEST_RESPONSE" | grep -o '"response":"[^"]*' | cut -d'"' -f4 | head -c 50)..."
else
    echo "   ❌ Model test failed"
    echo "   Response: $TEST_RESPONSE"
    exit 1
fi
echo ""

# 6. Generate .env file with correct model
echo "6. Creating .env file with correct configuration..."
ENV_FILE="$HOME/MicroVIT/robot1/orin_ros2_compute/.env"
mkdir -p "$(dirname "$ENV_FILE")"

cat > "$ENV_FILE" << EOF
# Robot Configuration
ROBOT_ID=jetson1

# Nano Connection
NANO_IP=10.13.68.184
NANO_PORT=8000

# Ollama Configuration
OLLAMA_HOST=http://localhost:11434
OLLAMA_MODEL=$FOUND_MODEL
OLLAMA_NO_GPU=1

# MicroViT Configuration - ENABLE THIS!
USE_MICROVIT=true
MICROVIT_MODEL_NAME=apple/mobilevit-small
MICROVIT_VARIANT=S1
MICROVIT_USE_CPU=false

# Ollama Text Generation
OLLAMA_TEXT_MODEL=$FOUND_MODEL
OLLAMA_MAX_TOKENS=500
OLLAMA_TIMEOUT=120

# MQTT Configuration (Controller broker)
MQTT_BROKER_HOST=10.13.68.48
MQTT_BROKER_PORT=1883
EOF

echo "   ✅ Created .env file at: $ENV_FILE"
echo "   Model set to: $FOUND_MODEL"
echo ""

# 7. Summary
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""
echo "✅ Ollama is running"
echo "✅ Model available: $FOUND_MODEL"
echo "✅ Model tested and working"
echo "✅ .env file created with correct configuration"
echo ""
echo "Next steps:"
echo "1. Restart your AI service:"
echo "   cd ~/MicroVIT/robot1/orin_ros2_compute"
echo "   source venv/bin/activate"
echo "   export \$(cat .env | grep -v '^#' | xargs)"
echo "   cd src"
echo "   python3 robot1_ai_service_realtime.py --simulation"
echo ""
echo "2. You should now see:"
echo "   ✅ MicroViT model loaded successfully!"
echo "   ✅ Generated AI message with MicroViT+Ollama (FULL):"
echo "   [Full natural language messages]"
echo ""
