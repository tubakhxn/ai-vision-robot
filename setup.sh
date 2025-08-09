#!/bin/bash

# Setup script for Object Detection + Obstacle Avoidance Robot
# Run this script on your Raspberry Pi to configure the system

echo "=================================================="
echo "Object Detection Robot Setup Script"
echo "=================================================="

# Check if running on Raspberry Pi
if ! grep -q "Raspberry Pi" /proc/cpuinfo; then
    echo "Warning: This script is designed for Raspberry Pi"
    echo "Some features may not work on other systems"
fi

# Update system packages
echo "1. Updating system packages..."
sudo apt update && sudo apt upgrade -y

# Install system dependencies
echo "2. Installing system dependencies..."
sudo apt install -y python3-pip python3-venv git cmake build-essential

# Enable camera and SPI interfaces
echo "3. Configuring Raspberry Pi interfaces..."
echo "Enabling camera interface..."
sudo raspi-config nonint do_camera 0

echo "Enabling SPI interface..."
sudo raspi-config nonint do_spi 0

echo "Enabling I2C interface..."
sudo raspi-config nonint do_i2c 0

# Install GPIO library
echo "4. Installing RPi.GPIO library..."
sudo apt install -y python3-rpi.gpio

# Create virtual environment
echo "5. Creating Python virtual environment..."
python3 -m venv robot_env
source robot_env/bin/activate

# Upgrade pip
pip install --upgrade pip

# Install OpenCV dependencies
echo "6. Installing OpenCV dependencies..."
sudo apt install -y libopencv-dev python3-opencv
sudo apt install -y libatlas-base-dev libhdf5-dev libhdf5-serial-dev
sudo apt install -y libqtgui4 libqt4-test

# Install Python packages
echo "7. Installing Python packages..."
pip install opencv-python==4.8.1.78
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install ultralytics
pip install numpy pillow

# Test camera
echo "8. Testing camera connection..."
if vcgencmd get_camera | grep -q "detected=1"; then
    echo "✓ Camera detected successfully"
else
    echo "✗ Camera not detected - check connection"
fi

# Create desktop shortcut
echo "9. Creating desktop shortcut..."
cat > ~/Desktop/robot_controller.desktop << EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Robot Controller
Comment=Object Detection Robot Controller
Exec=lxterminal -e "cd $(pwd) && source robot_env/bin/activate && python main.py"
Icon=robot
Terminal=true
Categories=Development;
EOF

chmod +x ~/Desktop/robot_controller.desktop

# Create systemd service (optional)
echo "10. Creating systemd service..."
sudo tee /etc/systemd/system/robot.service > /dev/null << EOF
[Unit]
Description=Object Detection Robot
After=network.target

[Service]
Type=simple
User=pi
WorkingDirectory=$(pwd)
ExecStart=$(pwd)/robot_env/bin/python $(pwd)/main.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

echo "=================================================="
echo "Setup Complete!"
echo "=================================================="
echo ""
echo "Next steps:"
echo "1. Reboot your Raspberry Pi: sudo reboot"
echo "2. Connect your hardware according to the wiring diagram"
echo "3. Test the system: source robot_env/bin/activate && python main.py"
echo ""
echo "Optional: Enable auto-start on boot:"
echo "sudo systemctl enable robot.service"
echo "sudo systemctl start robot.service"
echo ""
echo "Hardware connections required:"
echo "- L298N Motor Driver"
echo "- 2x HC-SR04 Ultrasonic Sensors"  
echo "- Pi Camera Module"
echo "- DC Motors and wheels"
echo ""
echo "See README.md for detailed wiring instructions"
