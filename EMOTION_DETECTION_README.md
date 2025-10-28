# Emotion Detection & Mood Chatbot System

This extension adds emotion detection and mood-based recommendation features to the face detection ROS package.

## Features

- **Real-time Emotion Detection**: Detects 7 emotions (happy, sad, angry, fear, surprise, disgust, neutral)
- **Mood-based Chatbot**: Provides personalized recommendations based on detected emotions
- **Interactive GUI**: Side-by-side display of live video feed and chatbot recommendations
- **ROS1 Integration**: Fully integrated with ROS topics and nodes
- **Gazebo Compatible**: Can work with Gazebo simulation cameras

## System Architecture

The system consists of 5 ROS nodes:

1. **camera_node.py**: Captures camera images
2. **face_detector_node.py**: Detects faces using OpenCV
3. **emotion_detector_node.py**: Analyzes emotions using DeepFace
4. **mood_chatbot_node.py**: Generates mood-based recommendations
5. **emotion_gui_node.py**: Displays video feed and recommendations

## Installation

### 1. Install Python Dependencies

```bash
cd ~/face_detection_ros_ws/src/face_detection_pkg
pip install -r requirements.txt
```

**Note**: Installing TensorFlow and DeepFace may take several minutes.

### 2. Make Scripts Executable

```bash
chmod +x scripts/emotion_detector_node.py
chmod +x scripts/mood_chatbot_node.py
chmod +x scripts/emotion_gui_node.py
```

### 3. Build the Package

```bash
cd ~/face_detection_ros_ws
catkin_make
source devel/setup.bash
```

## Usage

### Complete System with GUI

Launch the full system with camera, face detection, emotion detection, chatbot, and GUI:

```bash
roslaunch face_detection_pkg emotion_system.launch
```

### Without GUI (for Gazebo Integration)

If you want to use Gazebo or another camera source:

```bash
# Start your Gazebo simulation with a camera first
roslaunch your_gazebo_package your_world.launch

# Then launch emotion detection (remapping camera topic if needed)
roslaunch face_detection_pkg emotion_detection_no_gui.launch image_topic:=/your/camera/topic
```

### View Recommendations via Terminal

```bash
# Subscribe to recommendations
rostopic echo /chatbot/recommendations

# Subscribe to detected emotions
rostopic echo /emotion_detection/emotions
```

## ROS Topics

### Published Topics

- `/emotion_detection/emotions` (std_msgs/String): JSON string with detected emotions
- `/emotion_detection/annotated_image` (sensor_msgs/Image): Video with emotion labels
- `/chatbot/recommendations` (std_msgs/String): JSON string with mood recommendations

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image): Input camera feed
- `/face_detection/face_positions` (std_msgs/Int32MultiArray): Detected face positions

## Configuration

### Launch File Parameters

#### emotion_system.launch

- `camera_index` (default: 0): Camera device index
- `frame_rate` (default: 30.0): Camera frame rate
- `image_width` (default: 640): Image width
- `image_height` (default: 480): Image height
- `scale_factor` (default: 1.1): Face detection scale factor
- `min_neighbors` (default: 5): Face detection minimum neighbors
- `detection_interval` (default: 1.0): Emotion detection interval in seconds

Example with custom parameters:

```bash
roslaunch face_detection_pkg emotion_system.launch camera_index:=1 detection_interval:=2.0
```

## Chatbot Recommendations

The chatbot provides context-aware recommendations based on detected emotions:

### Happy
- Listen to upbeat music
- Share happiness with others
- Dance or be active

### Sad
- Take a walk outside
- Listen to calming music
- Reach out to friends
- Practice self-care

### Angry
- Take deep breaths
- Go for a walk
- Try physical exercise
- Practice meditation

### Fear
- Grounding techniques
- Soothing music
- Connect with support network

### Neutral
- Good time to be productive
- Try new activities
- Learn something new

## Gazebo Integration

To use with Gazebo:

1. Create a Gazebo world with a camera sensor
2. Launch your Gazebo simulation
3. Find the camera topic (usually `/camera/rgb/image_raw` or similar)
4. Launch emotion detection system:

```bash
roslaunch face_detection_pkg emotion_detection_no_gui.launch image_topic:=/your_camera_topic
```

5. Optionally, launch the GUI separately:

```bash
rosrun face_detection_pkg emotion_gui_node.py
```

## Troubleshooting

### DeepFace Not Installing

If DeepFace installation fails:

```bash
pip install --upgrade pip
pip install deepface --no-cache-dir
```

### TensorFlow Compatibility Issues

For GPU support issues:

```bash
pip install tensorflow-cpu>=2.10.0
```

### GUI Not Displaying on WSL/Remote Systems

For WSL or remote systems, you may need X11 forwarding:

```bash
export DISPLAY=:0
```

Or use the no-GUI launch file and view topics separately.

### Camera Not Found

Check available cameras:

```bash
ls /dev/video*
```

Then specify the correct index:

```bash
roslaunch face_detection_pkg emotion_system.launch camera_index:=1
```

## Performance Notes

- Emotion detection runs at configurable intervals (default 1 second) to balance accuracy and performance
- First emotion detection may take longer as DeepFace loads models
- For better performance on low-end hardware, increase `detection_interval` to 2-3 seconds

## Future Enhancements

- Integration with robot behaviors based on detected emotions
- Voice-based recommendation delivery
- Emotion history logging and analytics
- Multi-face emotion tracking
- Custom recommendation templates

## License

MIT License - Same as the parent face_detection_pkg
