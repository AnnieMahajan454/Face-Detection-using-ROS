#!/usr/bin/env python3
"""
Test Script for Face Detection ROS Package
This script tests the face detection application components on Windows
without requiring ROS to be installed.
"""

import os
import sys
import importlib.util
import platform
import traceback

print("=" * 60)
print("Face Detection ROS Package - Test Script")
print("=" * 60)
print(f"Platform: {platform.system()} {platform.release()}")
print(f"Python Version: {sys.version}")
print("-" * 60)

def test_python_dependencies():
    """Test if required Python packages are available."""
    print("\n🔍 Testing Python Dependencies...")
    
    required_packages = {
        'cv2': 'opencv-python',
        'numpy': 'numpy'
    }
    
    missing_packages = []
    
    for package, pip_name in required_packages.items():
        try:
            __import__(package)
            print(f"✅ {package} - OK")
        except ImportError:
            print(f"❌ {package} - MISSING (install with: pip install {pip_name})")
            missing_packages.append(pip_name)
    
    return missing_packages

def test_opencv_functionality():
    """Test OpenCV face detection functionality."""
    print("\n🔍 Testing OpenCV Face Detection...")
    
    try:
        import cv2
        import numpy as np
        
        # Test Haar cascade loading
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        face_cascade = cv2.CascadeClassifier(cascade_path)
        
        if face_cascade.empty():
            print("❌ Haar cascade classifier failed to load")
            return False
        else:
            print("✅ Haar cascade classifier loaded successfully")
        
        # Test with a dummy image
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)
        gray = cv2.cvtColor(test_image, cv2.COLOR_BGR2GRAY)
        
        # Test face detection (should return empty array for blank image)
        faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5)
        print(f"✅ Face detection test completed (found {len(faces)} faces in blank image)")
        
        return True
        
    except Exception as e:
        print(f"❌ OpenCV functionality test failed: {e}")
        return False

def test_camera_availability():
    """Test if camera is available."""
    print("\n🔍 Testing Camera Availability...")
    
    try:
        import cv2
        
        # Try to open camera
        cap = cv2.VideoCapture(0)
        
        if cap.isOpened():
            print("✅ Camera (index 0) is available")
            
            # Try to read a frame
            ret, frame = cap.read()
            if ret and frame is not None:
                print(f"✅ Camera can capture frames (size: {frame.shape})")
            else:
                print("⚠️ Camera opened but cannot capture frames")
            
            cap.release()
            return True
        else:
            print("❌ Camera (index 0) is not available")
            print("   - Check if camera is connected")
            print("   - Close other applications using the camera")
            print("   - Try different camera indices (1, 2, etc.)")
            return False
            
    except Exception as e:
        print(f"❌ Camera test failed: {e}")
        return False

def test_script_imports():
    """Test if ROS scripts can be imported (syntax check)."""
    print("\n🔍 Testing Script Imports...")
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    scripts_path = os.path.join(script_dir, 'scripts')
    
    scripts = [
        'camera_node.py',
        'face_detector_node.py',
        'face_visualizer_node.py'
    ]
    
    all_good = True
    
    for script in scripts:
        script_path = os.path.join(scripts_path, script)
        
        if not os.path.exists(script_path):
            print(f"❌ {script} - File not found")
            all_good = False
            continue
        
        try:
            # Try to compile the script
            with open(script_path, 'r') as f:
                source_code = f.read()
            
            compile(source_code, script_path, 'exec')
            print(f"✅ {script} - Syntax OK")
            
        except SyntaxError as e:
            print(f"❌ {script} - Syntax Error: {e}")
            all_good = False
        except Exception as e:
            print(f"❌ {script} - Error: {e}")
            all_good = False
    
    return all_good

def test_configuration_files():
    """Test if configuration files exist and are valid."""
    print("\n🔍 Testing Configuration Files...")
    
    base_dir = os.path.dirname(os.path.abspath(__file__))
    
    config_files = {
        'package.xml': 'Package configuration',
        'CMakeLists.txt': 'Build configuration',
        'config/face_detection_params.yaml': 'Parameters configuration',
        'launch/face_detection.launch': 'Main launch file',
        'launch/camera_only.launch': 'Camera launch file',
        'launch/detection_only.launch': 'Detection launch file'
    }
    
    all_good = True
    
    for file_path, description in config_files.items():
        full_path = os.path.join(base_dir, file_path)
        
        if os.path.exists(full_path):
            file_size = os.path.getsize(full_path)
            print(f"✅ {file_path} - OK ({file_size} bytes)")
        else:
            print(f"❌ {file_path} - Missing")
            all_good = False
    
    return all_good

def run_standalone_face_detection_demo():
    """Run a simple standalone face detection demo."""
    print("\n🔍 Running Standalone Face Detection Demo...")
    
    try:
        import cv2
        import numpy as np
        
        # Load face cascade
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        face_cascade = cv2.CascadeClassifier(cascade_path)
        
        # Open camera
        cap = cv2.VideoCapture(0)
        
        if not cap.isOpened():
            print("❌ Cannot open camera for demo")
            return False
        
        print("✅ Demo started! Press 'q' to quit the demo window.")
        print("   A window should open showing your camera feed with face detection.")
        
        frame_count = 0
        detected_faces_count = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Convert to grayscale for face detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Detect faces
            faces = face_cascade.detectMultiScale(gray, 1.1, 5)
            
            # Draw rectangles around faces
            for (x, y, w, h) in faces:
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
                cv2.putText(frame, 'Face', (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Add frame info
            cv2.putText(frame, f'Faces: {len(faces)}', (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.putText(frame, f'Frame: {frame_count}', (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
            cv2.putText(frame, 'Press Q to quit', (10, frame.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Show frame
            cv2.imshow('Face Detection Demo', frame)
            
            if len(faces) > 0:
                detected_faces_count += 1
            
            frame_count += 1
            
            # Quit on 'q' key
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        print(f"✅ Demo completed!")
        print(f"   Processed {frame_count} frames")
        print(f"   Detected faces in {detected_faces_count} frames")
        
        return True
        
    except Exception as e:
        print(f"❌ Demo failed: {e}")
        traceback.print_exc()
        return False

def main():
    """Main test function."""
    print("Starting comprehensive test of Face Detection ROS Package...\n")
    
    # Test results
    results = {}
    
    # Run tests
    results['dependencies'] = len(test_python_dependencies()) == 0
    results['opencv'] = test_opencv_functionality()
    results['camera'] = test_camera_availability()
    results['scripts'] = test_script_imports()
    results['config'] = test_configuration_files()
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    for test_name, passed in results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{test_name.upper():15} {status}")
    
    all_passed = all(results.values())
    
    if all_passed:
        print(f"\n🎉 ALL TESTS PASSED! The application is ready to use.")
        
        # Ask if user wants to run demo
        print("\n" + "-" * 60)
        try:
            response = input("Would you like to run a face detection demo? (y/n): ").lower().strip()
            if response in ['y', 'yes']:
                run_standalone_face_detection_demo()
        except KeyboardInterrupt:
            print("\nDemo cancelled.")
        
    else:
        print(f"\n⚠️  Some tests failed. Please fix the issues before using the application.")
        failed_tests = [name for name, passed in results.items() if not passed]
        print(f"Failed tests: {', '.join(failed_tests)}")
    
    print("\n" + "=" * 60)
    print("Test completed!")

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
        traceback.print_exc()
