#!/bin/bash
echo "🔍 Camera Hardware Mapping Test Script"
echo "----------------------------------------"

# Function to test a channel
test_channel() {
    CHANNEL=$1
    HEX_CODE=$2
    NAME=$3
    
    echo ""
    echo "👉 Testing Channel $CHANNEL (Sending $HEX_CODE)..."
    echo "   Target: $NAME"
    
    # 1. Switch Mux
    i2ctransfer -f -y 4 w3@0x0c 0xff 0x55 $HEX_CODE
    if [ $? -eq 0 ]; then
        echo "   ✅ I2C Switch Command Sent"
    else
        echo "   ❌ I2C Command Failed!"
        return
    fi
    
    # 2. Verify Camera Availability
    echo "   Checking camera connection..."
    rpicam-hello --list-cameras > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "   ✅ Camera Detected on Channel $CHANNEL"
        
        # 3. Optional: Capture check (if needed)
        # rpicam-still -t 500 -o test_ch${CHANNEL}.jpg
    else
        echo "   ❌ NO Camera detected on Channel $CHANNEL"
    fi
}

# Stop any running camera nodes to free resource
echo "🛑 Stopping existing camera processes..."
pkill -f csi_camera_node
pkill -f rpicam-vid
pkill -f rpicam-hello
sleep 2

# TEST 1: Code 0x01 (Original Cam 0)
test_channel "1" "0x01" "Original Software CAM 0"

sleep 1

# TEST 2: Code 0x02 (Original Cam 1)
test_channel "2" "0x02" "Original Software CAM 1"

echo ""
echo "----------------------------------------"
echo "❓ Please observe physically or check logs to identify which camera active."
echo "   - If 0x01 activates OUTPUT Tray -> Code map was WRONG."
echo "   - If 0x01 activates INPUT Tray  -> Code map was CORRECT."
echo "----------------------------------------"
