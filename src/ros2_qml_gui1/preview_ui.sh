#!/bin/bash

# Script to preview Robot Control Panel UI without ROS2

echo "🎨 Robot Control Panel - UI Preview"
echo "===================================="
echo ""
echo "This will launch a preview of the robot control panel"
echo "without requiring ROS2 or the full application."
echo ""

# Check if qmlscene is installed
if ! command -v qmlscene &> /dev/null; then
    echo "❌ qmlscene not found!"
    echo ""
    echo "Please install it with:"
    echo "  sudo apt-get install qml-module-qtquick-controls2"
    echo "  sudo apt-get install qml-module-qtquick-layouts"
    echo "  sudo apt-get install qtdeclarative5-dev-tools"
    exit 1
fi

# Run the preview
echo "✅ Launching preview..."
echo ""
echo "Click buttons to see console output"
echo "Press Ctrl+C to exit"
echo ""

cd "$(dirname "$0")"
qmlscene qml/RobotControlPreview.qml
