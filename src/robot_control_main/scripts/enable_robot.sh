#!/bin/bash
ROBOT_IP="192.168.27.8"
PORT=29999

# Function to send command and read response
send_cmd() {
    printf "$1\n" | nc -w 2 $ROBOT_IP $PORT
}

echo "Connecting to robot at $ROBOT_IP:$PORT..."

# Send commands
echo "Sending ClearError..."
send_cmd "ClearError()"

sleep 0.2

echo "Sending EnableRobot..."
send_cmd "EnableRobot()"

sleep 0.5

echo "Setting User(0)..."
send_cmd "User(0)"

echo "Setting Tool(1)..."
send_cmd "Tool(1)"

echo "Setting SpeedJ(50)..."
send_cmd "SpeedJ(50)"

echo "Setting AccJ(20)..."
send_cmd "AccJ(20)"

echo "Setting CP(1)..."
send_cmd "CP(1)"

echo "✅ Robot enable commands completed"
exit 0