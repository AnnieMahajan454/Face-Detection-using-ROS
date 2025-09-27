#!/bin/bash
# Setup script to make Python scripts executable

echo "Setting up Face Detection ROS Package..."

# Make Python scripts executable
chmod +x scripts/*.py

echo "Scripts made executable:"
ls -la scripts/

echo "Setup complete! You can now build the package with 'catkin_make'"
