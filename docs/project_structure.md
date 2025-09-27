# Face Detection ROS Package - Project Structure

This document describes the structure of the Face Detection ROS package.

## Directory Structure

```
face_detection_pkg/
├── .gitignore                          # Git ignore file
├── CMakeLists.txt                      # CMake build configuration
├── package.xml                         # ROS package metadata
├── README.md                           # Main project documentation
├── WINDOWS_SETUP.md                   # Windows-specific setup guide
├── requirements.txt                    # Python dependencies
├── setup_permissions.sh               # Unix permission setup script
├── test_application.py                 # Comprehensive test script
├── docs/                              # Documentation directory
│   └── project_structure.md          # This file
├── scripts/                           # Python executable scripts
│   ├── camera_node.py                 # Camera capture node
│   ├── face_detector_node.py          # Face detection node
│   └── face_visualizer_node.py        # Visualization node
├── launch/                            # ROS launch files
│   ├── face_detection.launch          # Complete system launch
│   ├── camera_only.launch             # Camera node only
│   └── detection_only.launch          # Detection without visualization
└── config/                            # Configuration files
    └── face_detection_params.yaml     # System parameters
```

## Key Components

### Core Nodes
- **Camera Node**: Captures video from camera and publishes ROS Image messages
- **Face Detection Node**: Processes images and detects faces using OpenCV
- **Visualization Node**: Displays results with interactive controls

### Configuration
- **Launch Files**: Different deployment scenarios (full, camera-only, headless)
- **Parameter Files**: Configurable detection and camera settings

### Documentation
- **README**: Complete usage and installation guide
- **Windows Setup**: Platform-specific instructions
- **Project Structure**: This architectural overview

### Testing
- **Test Application**: Comprehensive validation script for Windows
- **Requirements**: Python package dependencies

This modular structure allows for easy development, testing, and deployment across different platforms and use cases.
