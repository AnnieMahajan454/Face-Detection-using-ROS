# Face Detection using ROS

A comprehensive ROS package for real-time face detection using OpenCV and Haar cascades. This package captures video from a camera, detects faces in real-time, and provides visualization and data output capabilities.

## Features

- **Real-time face detection** using OpenCV Haar cascades
- **ROS-based architecture** with modular nodes
- **Live visualization** with annotated video feed
- **Configurable parameters** via YAML files and ROS parameters
- **Multiple launch configurations** for different use cases
- **Face position data publishing** for integration with other systems
- **Image saving capabilities** for detected faces and original frames

## Package Structure

```
face_detection_pkg/
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata and dependencies
├── README.md                   # This file
├── scripts/                    # Python nodes
│   ├── face_detector_node.py   # Main face detection node
│   ├── camera_node.py          # Camera capture node
│   └── face_visualizer_node.py # Visualization node
├── launch/                     # Launch files
│   ├── face_detection.launch   # Complete system launch
│   ├── camera_only.launch      # Camera node only
│   └── detection_only.launch   # Detection without visualization
└── config/                     # Configuration files
    └── face_detection_params.yaml # Parameters configuration
```

## Prerequisites

### System Requirements
- Ubuntu 18.04/20.04/22.04 (or compatible Linux distribution)
- ROS Melodic/Noetic (or ROS2 with minor modifications)
- Python 3.6+
- Camera (USB webcam or built-in camera)

### Dependencies

#### ROS Packages
```bash
sudo apt-get install ros-$ROS_NOETIC-cv-bridge
sudo apt-get install ros-$ROS_NOETIC-image-transport
sudo apt-get install ros-$ROS_NOETIC-sensor-msgs
sudo apt-get install ros-$ROS_NOETIC-std-msgs
sudo apt-get install ros-$ROS_NOETIC-geometry-msgs
```

#### Python Packages
```bash
pip3 install opencv-python
pip3 install numpy
# Or using apt:
sudo apt-get install python3-opencv
sudo apt-get install python3-numpy
```

## Installation

1. **Create a ROS workspace** (if you don't have one):
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

2. **Clone or copy this package** to your ROS workspace:
```bash
cd ~/catkin_ws/src
# If using git:
git clone https://github.com/AnnieMahajan454/Face-Detection-using-ROS.git face_detection_pkg
# Or copy the entire face_detection_pkg folder to ~/catkin_ws/src/
```

3. **Make scripts executable**:
```bash
cd ~/catkin_ws/src/face_detection_pkg/scripts
chmod +x *.py
```

4. **Build the package**:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Usage

### Quick Start

1. **Launch the complete face detection system**:
```bash
roslaunch face_detection_pkg face_detection.launch
```

This will start:
- Camera node (captures video)
- Face detection node (processes images and detects faces)
- Visualization node (displays results)

### Available Launch Files

#### 1. Complete System
```bash
roslaunch face_detection_pkg face_detection.launch
```
Launches all nodes with visualization.

#### 2. Camera Only
```bash
roslaunch face_detection_pkg camera_only.launch
```
Only starts the camera node for testing camera functionality.

#### 3. Detection Only
```bash
roslaunch face_detection_pkg detection_only.launch
```
Starts camera and detection nodes without visualization (headless mode).

### Manual Node Execution

You can also run nodes individually:

1. **Start ROS master**:
```bash
roscore
```

2. **Run camera node**:
```bash
rosrun face_detection_pkg camera_node.py
```

3. **Run face detection node** (in another terminal):
```bash
rosrun face_detection_pkg face_detector_node.py
```

4. **Run visualization node** (in another terminal):
```bash
rosrun face_detection_pkg face_visualizer_node.py
```

## Configuration

### Parameters

The system can be configured through ROS parameters and the YAML configuration file.

#### Main Parameters (can be set in launch files):

**Camera Parameters:**
- `camera_index`: Camera device index (default: 0)
- `frame_rate`: Capture frame rate (default: 30.0)
- `image_width`: Image width (default: 640)
- `image_height`: Image height (default: 480)

**Face Detection Parameters:**
- `scale_factor`: Detection scale factor (default: 1.1)
- `min_neighbors`: Minimum neighbors for detection (default: 5)
- `min_size`: Minimum face size [width, height] (default: [30, 30])
- `max_size`: Maximum face size [width, height] (default: [300, 300])

**Visualization Parameters:**
- `display_original`: Show original camera feed (default: true)
- `display_annotated`: Show annotated feed (default: true)
- `window_name_original`: Original window name
- `window_name_annotated`: Annotated window name

### Configuration File

Edit `config/face_detection_params.yaml` to modify default parameters.

## ROS Topics

### Published Topics

- `/camera/image_raw` (sensor_msgs/Image): Raw camera images
- `/face_detection/annotated_image` (sensor_msgs/Image): Images with face detection annotations
- `/face_detection/face_positions` (std_msgs/Int32MultiArray): Face position data [x1, y1, w1, h1, x2, y2, w2, h2, ...]

### Subscribed Topics

- Face detection node subscribes to `/camera/image_raw`
- Visualization node subscribes to `/camera/image_raw`, `/face_detection/annotated_image`, and `/face_detection/face_positions`

## Interactive Controls

When running the visualization node, the following keyboard controls are available in the OpenCV windows:

- **'q'**: Quit the application
- **'s'**: Save current images and face position data
- **'r'**: Reset/refresh display windows
- **Any other key**: Display help information

## Troubleshooting

### Common Issues

1. **Camera not detected**:
   - Check if camera is connected and not used by another application
   - Try different camera indices (0, 1, 2, ...)
   - On Linux, check permissions: `ls -l /dev/video*`

2. **No face detection**:
   - Ensure proper lighting conditions
   - Adjust detection parameters (`scale_factor`, `min_neighbors`)
   - Try different Haar cascade classifiers

3. **Poor performance**:
   - Reduce image resolution
   - Increase `min_size` parameter
   - Lower frame rate

4. **ROS package not found**:
   - Ensure the workspace is built: `catkin_make`
   - Source the workspace: `source devel/setup.bash`
   - Check package is in the correct location

### Debugging

1. **Check ROS topics**:
```bash
rostopic list
rostopic echo /face_detection/face_positions
```

2. **Monitor image topics**:
```bash
rosrun rqt_image_view rqt_image_view
```

3. **Check node status**:
```bash
rosnode list
rosnode info /face_detector_node
```

## Customization

### Adding New Detection Features

1. **Modify the face detection node** (`scripts/face_detector_node.py`)
2. **Add new publishers/subscribers** as needed
3. **Update launch files** with new parameters
4. **Rebuild the package**: `catkin_make`

### Using Different Classifiers

OpenCV provides several Haar cascade classifiers:
- `haarcascade_frontalface_default.xml` (default)
- `haarcascade_frontalface_alt.xml`
- `haarcascade_frontalface_alt2.xml`
- `haarcascade_profileface.xml`

Modify the `cascade_path` parameter in the configuration or launch files.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## Version History

- **v1.0.0**: Initial release with basic face detection functionality
  - Real-time face detection using Haar cascades
  - ROS integration with multiple nodes
  - Visualization and configuration capabilities
