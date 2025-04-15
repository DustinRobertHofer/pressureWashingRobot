#!/bin/bash
# Installation script for Pressure Washing Robot on Raspberry Pi
# Run with sudo: sudo bash install_dependencies.sh

echo "Installing dependencies for Pressure Washing Robot..."

# Update package lists
echo "Updating package lists..."
apt-get update

# Install Python 3 and pip if not already installed
echo "Installing Python 3 and pip..."
apt-get install -y python3 python3-pip

# Install GPIO library
echo "Installing GPIO libraries..."
apt-get install -y python3-rpi.gpio

# Install I2C tools and libraries
echo "Installing I2C tools..."
apt-get install -y i2c-tools python3-smbus

# Install serial tools for laser range finder
echo "Installing serial tools..."
apt-get install -y python3-serial

# Enable I2C interface
echo "Enabling I2C interface..."
if ! grep -q "^dtparam=i2c_arm=on" /boot/config.txt; then
    echo "dtparam=i2c_arm=on" >> /boot/config.txt
fi

# Enable serial interface (for laser range finder)
echo "Enabling serial interface..."
if ! grep -q "^enable_uart=1" /boot/config.txt; then
    echo "enable_uart=1" >> /boot/config.txt
fi
# Disable serial console to free up UART for laser range finder
sed -i 's/console=serial0,115200 //g' /boot/cmdline.txt

# Enable hardware PWM for better motor control
echo "Enabling PWM..."
if ! grep -q "^dtoverlay=pwm-2chan" /boot/config.txt; then
    echo "dtoverlay=pwm-2chan" >> /boot/config.txt
fi

# Install additional Python packages
echo "Installing Python libraries..."
pip3 install numpy pyserial

# Create a systemd service for autostart (optional)
echo "Creating systemd service for auto-start (disabled by default)..."
cat > /etc/systemd/system/pressure_washing_robot.service << EOL
[Unit]
Description=Pressure Washing Robot Service
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/pi/pressure_washing_robot/controllers/robotControl/hardware/pi_robot_controller.py
WorkingDirectory=/home/pi/pressure_washing_robot
StandardOutput=inherit
StandardError=inherit
Restart=always
User=pi

[Install]
WantedBy=multi-user.target
EOL

# Set permissions for the service file
chmod 644 /etc/systemd/system/pressure_washing_robot.service

echo ""
echo "Installation complete!"
echo ""
echo "To start the service automatically at boot:"
echo "  sudo systemctl enable pressure_washing_robot.service"
echo ""
echo "To start the service now:"
echo "  sudo systemctl start pressure_washing_robot.service"
echo ""
echo "To check status:"
echo "  sudo systemctl status pressure_washing_robot.service"
echo ""
echo "To run the robot manually:"
echo "  python3 /path/to/pi_robot_controller.py"
echo ""
echo "Reboot the Raspberry Pi for I2C, UART and PWM changes to take effect:"
echo "  sudo reboot" 