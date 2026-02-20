# Fix Ollama 404 Error - Model Not Found

## Issue
Ollama is returning 404 on `/api/generate` which means the model `qwen2.5:0.5b` is not available.

## Solution: Pull the Model

**On Orin, check available models:**
```bash
ollama list
```

**If `qwen2.5:0.5b` is not listed, pull it:**
```bash
ollama pull qwen2.5:0.5b
```

**Or use a different model that's already available:**
```bash
# Check what models you have
ollama list

# Common small models:
ollama pull phi3:mini
ollama pull llama3.2:1b
ollama pull qwen2:0.5b  # Note: might be qwen2 not qwen2.5
```

## Update Environment Variable

After pulling the model, update your environment:

```bash
# If using qwen2.5:0.5b
export OLLAMA_MODEL=qwen2.5:0.5b

# Or if using phi3:mini
export OLLAMA_MODEL=phi3:mini

# Or if using qwen2:0.5b
export OLLAMA_MODEL=qwen2:0.5b
```

## Verify Model Works

**Test the model directly:**
```bash
ollama run qwen2.5:0.5b "Hello, test message"
```

**Or test via API:**
```bash
curl http://localhost:11434/api/generate -d '{
  "model": "qwen2.5:0.5b",
  "prompt": "Hello",
  "stream": false
}'
```

## Restart AI Service

After pulling the model, restart your AI service:

```bash
cd ~/MicroVIT/robot1/orin_ros2_compute
source venv/bin/activate
export USE_MICROVIT=true
export OLLAMA_MODEL=qwen2.5:0.5b  # Or whatever model you pulled
export NANO_IP=10.13.68.184
export NANO_PORT=8000
export OLLAMA_NO_GPU=1
cd src
python3 robot1_ai_service_realtime.py --simulation
```

## Common Model Names

If `qwen2.5:0.5b` doesn't work, try:
- `qwen2:0.5b` (without the .5)
- `phi3:mini` (very small, fast)
- `llama3.2:1b` (small llama model)
- `tinyllama` (smallest)
