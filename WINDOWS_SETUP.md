# Windows Setup Guide for Face Detection ROS Package

This guide provides specific instructions for setting up and testing the Face Detection ROS Package on Windows.

## Prerequisites for Windows

### 1. Python Installation
- **Python 3.7 or higher** (Python 3.11 recommended)
- Download from: https://python.org/downloads/
- ⚠️ **Important**: During installation, check "Add Python to PATH"

### 2. Install Required Python Packages
Open **PowerShell** or **Command Prompt** as Administrator and run:

```powershell
# Install OpenCV and NumPy
pip install opencv-python numpy

# Verify installation
python -c "import cv2; print('OpenCV version:', cv2.__version__)"
python -c "import numpy; print('NumPy version:', numpy.__version__)"
```

### 3. Camera Requirements
- **USB webcam** or **built-in camera**
- Ensure camera is not being used by other applications (Skype, Teams, etc.)

## Quick Testing (Without ROS)

### Step 1: Run the Test Script

Navigate to the package directory and run the comprehensive test:

```powershell
cd "C:\Users\annie\face_detection_ros_ws\src\face_detection_pkg"
python test_application.py
```

This will test:
- ✅ Python dependencies (OpenCV, NumPy)
- ✅ OpenCV functionality and Haar cascades
- ✅ Camera availability
- ✅ Script syntax validation
- ✅ Configuration files
- 🎮 Optional: Live face detection demo

### Step 2: Manual Testing

If you want to test individual components:

```powershell
# Test OpenCV installation
python -c "import cv2; print('OpenCV OK')"

# Test camera access
python -c "import cv2; cap = cv2.VideoCapture(0); print('Camera available:', cap.isOpened()); cap.release()"

# Check Haar cascade file
python -c "import cv2; print('Cascade path:', cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')"
```

## Standalone Face Detection Demo

You can run a standalone face detection demo without ROS:

```powershell
cd "C:\Users\annie\face_detection_ros_ws\src\face_detection_pkg"
python -c "
import cv2
cap = cv2.VideoCapture(0)
face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
while True:
    ret, frame = cap.read()
    if not ret: break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.1, 5)
    for (x,y,w,h) in faces:
        cv2.rectangle(frame, (x,y), (x+w,y+h), (0,255,0), 2)
    cv2.putText(frame, f'Faces: {len(faces)}', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)
    cv2.imshow('Face Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'): break
cap.release()
cv2.destroyAllWindows()
"
```

Press 'q' to quit the demo.

## Installing ROS on Windows (Optional)

If you want to run the full ROS system, you have these options:

### Option 1: WSL2 with Ubuntu (Recommended)

1. **Install WSL2**:
   ```powershell
   wsl --install -d Ubuntu-22.04
   ```

2. **In Ubuntu WSL**, install ROS:
   ```bash
   # Update system
   sudo apt update && sudo apt upgrade -y
   
   # Install ROS Noetic
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
   sudo apt update
   sudo apt install ros-noetic-desktop-full
   
   # Install dependencies
   sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
   sudo apt install python3-opencv python3-numpy
   
   # Setup environment
   echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```

3. **Copy your package to WSL**:
   ```bash
   mkdir -p ~/catkin_ws/src
   # Copy the face_detection_pkg folder to ~/catkin_ws/src/
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

### Option 2: ROS2 on Windows (Native)

1. **Download ROS2**: https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html
2. **Install Visual Studio 2019** or later
3. **Follow ROS2 Windows installation guide**

## Troubleshooting Windows Issues

### Common Problems

1. **"cv2 module not found"**:
   ```powershell
   pip uninstall opencv-python
   pip install opencv-python
   ```

2. **Camera not detected**:
   - Check Device Manager → Cameras
   - Try different camera indices (0, 1, 2)
   - Close other applications using camera

3. **Permission denied errors**:
   - Run PowerShell as Administrator
   - Check antivirus software blocking camera access

4. **Python not found**:
   - Reinstall Python with "Add to PATH" option
   - Or add Python to PATH manually

### Performance Tips

1. **Reduce image resolution** for better performance:
   - Edit `config/face_detection_params.yaml`
   - Set `image_width: 320` and `image_height: 240`

2. **Adjust detection parameters**:
   - Increase `min_size: [50, 50]` for faster detection
   - Reduce `scale_factor: 1.2` for more accurate detection

3. **Lower frame rate**:
   - Set `frame_rate: 15.0` in configuration

## Testing Results

After running `python test_application.py`, you should see:

```
============================================================
Face Detection ROS Package - Test Script
============================================================
Platform: Windows 10
Python Version: 3.11.x

🔍 Testing Python Dependencies...
✅ cv2 - OK
✅ numpy - OK

🔍 Testing OpenCV Face Detection...
✅ Haar cascade classifier loaded successfully
✅ Face detection test completed (found 0 faces in blank image)

🔍 Testing Camera Availability...
✅ Camera (index 0) is available
✅ Camera can capture frames (size: (480, 640, 3))

🔍 Testing Script Imports...
✅ camera_node.py - Syntax OK
✅ face_detector_node.py - Syntax OK
✅ face_visualizer_node.py - Syntax OK

🔍 Testing Configuration Files...
✅ package.xml - OK (1255 bytes)
✅ CMakeLists.txt - OK (2661 bytes)
✅ config/face_detection_params.yaml - OK (1839 bytes)
✅ launch/face_detection.launch - OK (1275 bytes)
✅ launch/camera_only.launch - OK (392 bytes)
✅ launch/detection_only.launch - OK (882 bytes)

============================================================
TEST SUMMARY
============================================================
DEPENDENCIES     ✅ PASS
OPENCV          ✅ PASS
CAMERA          ✅ PASS
SCRIPTS         ✅ PASS
CONFIG          ✅ PASS

🎉 ALL TESTS PASSED! The application is ready to use.
```

## Next Steps

1. **If all tests pass**: Your application is ready!
2. **To use with ROS**: Set up WSL2 + Ubuntu + ROS
3. **For standalone use**: Run the test script's demo mode
4. **For development**: Modify the Python scripts as needed

## Contact & Support

- **Issues**: Check the main README.md for troubleshooting
- **GitHub**: https://github.com/AnnieMahajan454/Face-Detection-using-ROS
- **Windows-specific problems**: Create an issue with "Windows" label
