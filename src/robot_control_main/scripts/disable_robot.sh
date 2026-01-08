#!/bin/bash
# Disable Dobot Robot

ROBOT_IP="192.168.27.8"
PORT=29999

echo "DisableRobot()" | nc $ROBOT_IP $PORT
echo "Robot disabled"
