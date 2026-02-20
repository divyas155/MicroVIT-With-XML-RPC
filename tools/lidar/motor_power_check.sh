#!/bin/bash
# Check if LiDAR motor is spinning and power status
# Run this on Nano

echo "=========================================="
echo "LiDAR Motor & Power Diagnostic"
echo "=========================================="
echo ""

LIDAR_DEVICE="/dev/ttyUSB0"

echo "CRITICAL CHECKS:"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "1. PHYSICAL CHECK - Is the LiDAR motor spinning?"
echo "   → Look at/feel the LiDAR disc - it should be ROTATING"
echo "   → If NOT spinning, this is the problem!"
echo ""
read -p "   Is the motor spinning? (y/n): " -n 1 -r
MOTOR_SPINNING=$REPLY
echo ""
echo ""

echo "2. LED CHECK - Is the LiDAR LED on?"
echo "   → Most LiDARs have a status LED (green/red)"
echo "   → Should be ON when powered"
echo ""
read -p "   Is the LED on? (y/n): " -n 1 -r
LED_ON=$REPLY
echo ""
echo ""

echo "3. RAW DATA TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Testing if LiDAR sends ANY data..."
echo "(This will take 5 seconds)"
echo ""

# Configure serial port
stty -F "$LIDAR_DEVICE" 115200 cs8 -cstopb -parenb raw -echo 2>/dev/null

# Try to read data
RAW_DATA=$(timeout 5 cat "$LIDAR_DEVICE" 2>&1 | head -c 100)

if [ -n "$RAW_DATA" ] && [ ${#RAW_DATA} -gt 0 ]; then
    echo "✅ Received ${#RAW_DATA} bytes of data"
    echo "   First 50 bytes (hex):"
    echo "$RAW_DATA" | head -c 50 | hexdump -C | head -3
    HAS_DATA="yes"
else
    echo "❌ NO DATA received from LiDAR"
    echo "   This confirms the LiDAR is not communicating"
    HAS_DATA="no"
fi
echo ""

echo "=========================================="
echo "DIAGNOSIS"
echo "=========================================="
echo ""

if [[ ! $MOTOR_SPINNING =~ ^[Yy]$ ]]; then
    echo "⚠️⚠️⚠️ PROBLEM IDENTIFIED: Motor NOT spinning"
    echo ""
    echo "This is a HARDWARE/POWER issue. Solutions:"
    echo ""
    echo "SOLUTION 1: External Power Supply (MOST COMMON FIX)"
    echo "  → Many RPLidars need external 5V power (not just USB)"
    echo "  → Connect LiDAR to 5V power supply"
    echo "  → USB only provides data connection"
    echo ""
    echo "SOLUTION 2: Try Different USB Port"
    echo "  → Some USB ports provide more power"
    echo "  → Try USB 3.0 port if available"
    echo ""
    echo "SOLUTION 3: Check Power Requirements"
    echo "  → Check LiDAR manual for power specs"
    echo "  → RPLidar A1: Usually USB power OK"
    echo "  → RPLidar A2/A3: Often need external 5V"
    echo ""
    echo "SOLUTION 4: Check USB Cable"
    echo "  → Try a different USB cable"
    echo "  → Some cables don't carry enough power"
    echo ""
    echo "SOLUTION 5: Power Cycle"
    echo "  → Unplug LiDAR completely"
    echo "  → Wait 10 seconds"
    echo "  → Plug back in"
    echo ""
elif [[ $HAS_DATA == "no" ]]; then
    echo "⚠️⚠️⚠️ PROBLEM: Motor spinning but NO DATA"
    echo ""
    echo "Possible causes:"
    echo "  1. Wrong LiDAR model (not RPLidar)"
    echo "  2. Faulty LiDAR unit"
    echo "  3. Wrong baud rate (but we tested all common ones)"
    echo "  4. LiDAR needs initialization command"
    echo ""
    echo "Check:"
    echo "  → What LiDAR model do you have? (A1, A2, A3, S1, or other?)"
    echo "  → Check LiDAR manual for specific requirements"
else
    echo "✅ Motor spinning AND data received!"
    echo "   Issue might be baud rate or driver configuration"
fi

echo ""
echo "=========================================="
