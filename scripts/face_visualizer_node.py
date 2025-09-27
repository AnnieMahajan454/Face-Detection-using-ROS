#!/usr/bin/env python3
"""
Face Visualizer Node
This node subscribes to annotated images and face positions,
and displays them using OpenCV windows.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError


class FaceVisualizerNode:
    def __init__(self):
        """Initialize the face visualizer node."""
        rospy.init_node('face_visualizer_node', anonymous=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.display_original = rospy.get_param('~display_original', True)
        self.display_annotated = rospy.get_param('~display_annotated', True)
        self.window_name_original = rospy.get_param('~window_name_original', 'Original Camera')
        self.window_name_annotated = rospy.get_param('~window_name_annotated', 'Face Detection')
        
        # Current images
        self.original_image = None
        self.annotated_image = None
        self.face_positions = []
        
        # Subscribers
        if self.display_original:
            self.original_sub = rospy.Subscriber('/camera/image_raw', Image, 
                                               self.original_image_callback)
        
        if self.display_annotated:
            self.annotated_sub = rospy.Subscriber('/face_detection/annotated_image', Image, 
                                                self.annotated_image_callback)
        
        self.face_positions_sub = rospy.Subscriber('/face_detection/face_positions', 
                                                 Int32MultiArray, 
                                                 self.face_positions_callback)
        
        # Setup OpenCV windows
        if self.display_original:
            cv2.namedWindow(self.window_name_original, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name_original, 640, 480)
        
        if self.display_annotated:
            cv2.namedWindow(self.window_name_annotated, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name_annotated, 640, 480)
        
        # Timer for displaying images
        self.timer = rospy.Timer(rospy.Duration(1.0/30.0), self.display_timer_callback)
        
        rospy.loginfo("Face Visualizer Node initialized successfully!")
        rospy.loginfo("Press 'q' in any OpenCV window to quit")
    
    def original_image_callback(self, data):
        """
        Callback for original camera images.
        
        Args:
            data: ROS Image message
        """
        try:
            self.original_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error (original): {e}")
    
    def annotated_image_callback(self, data):
        """
        Callback for annotated images with face detection results.
        
        Args:
            data: ROS Image message
        """
        try:
            self.annotated_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error (annotated): {e}")
    
    def face_positions_callback(self, data):
        """
        Callback for face position data.
        
        Args:
            data: Int32MultiArray message containing face positions
        """
        if len(data.data) > 0:
            # Reshape flat array back to (n, 4) format
            face_data = np.array(data.data).reshape(-1, 4)
            self.face_positions = face_data.tolist()
        else:
            self.face_positions = []
        
        # Log face information
        num_faces = len(self.face_positions)
        if num_faces > 0:
            rospy.loginfo(f"Received positions for {num_faces} face(s)")
            for i, (x, y, w, h) in enumerate(self.face_positions):
                rospy.loginfo(f"  Face {i+1}: ({x}, {y}) size {w}x{h}")
    
    def display_timer_callback(self, event):
        """
        Timer callback to display images.
        
        Args:
            event: ROS timer event
        """
        # Display original image
        if self.display_original and self.original_image is not None:
            cv2.imshow(self.window_name_original, self.original_image)
        
        # Display annotated image
        if self.display_annotated and self.annotated_image is not None:
            cv2.imshow(self.window_name_annotated, self.annotated_image)
        
        # Handle key presses
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.loginfo("Quit key pressed, shutting down...")
            rospy.signal_shutdown("User requested shutdown")
        elif key == ord('s'):
            # Save current images
            self.save_images()
        elif key == ord('r'):
            # Reset/clear windows
            self.reset_display()
        elif key != 255:  # Any other key
            rospy.loginfo(f"Key pressed: {chr(key)} (ASCII: {key})")
            rospy.loginfo("Available commands:")
            rospy.loginfo("  'q' - Quit")
            rospy.loginfo("  's' - Save current images")
            rospy.loginfo("  'r' - Reset display")
    
    def save_images(self):
        """Save current images to disk."""
        timestamp = rospy.get_time()
        
        if self.original_image is not None:
            filename = f"original_image_{timestamp:.2f}.jpg"
            cv2.imwrite(filename, self.original_image)
            rospy.loginfo(f"Saved original image as {filename}")
        
        if self.annotated_image is not None:
            filename = f"annotated_image_{timestamp:.2f}.jpg"
            cv2.imwrite(filename, self.annotated_image)
            rospy.loginfo(f"Saved annotated image as {filename}")
        
        # Also save face position data
        if self.face_positions:
            filename = f"face_positions_{timestamp:.2f}.txt"
            with open(filename, 'w') as f:
                f.write(f"Face positions at timestamp {timestamp}\n")
                f.write(f"Number of faces: {len(self.face_positions)}\n")
                f.write("Format: x, y, width, height\n")
                for i, (x, y, w, h) in enumerate(self.face_positions):
                    f.write(f"Face {i+1}: {x}, {y}, {w}, {h}\n")
            rospy.loginfo(f"Saved face positions as {filename}")
    
    def reset_display(self):
        """Reset/clear the display windows."""
        if self.display_original:
            cv2.destroyWindow(self.window_name_original)
            cv2.namedWindow(self.window_name_original, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name_original, 640, 480)
        
        if self.display_annotated:
            cv2.destroyWindow(self.window_name_annotated)
            cv2.namedWindow(self.window_name_annotated, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name_annotated, 640, 480)
        
        rospy.loginfo("Display windows reset")
    
    def shutdown(self):
        """Clean up resources."""
        cv2.destroyAllWindows()
        rospy.loginfo("Face Visualizer Node shutting down")


def main():
    """Main function."""
    try:
        visualizer = FaceVisualizerNode()
        rospy.on_shutdown(visualizer.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Face Visualizer Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error in Face Visualizer Node: {e}")


if __name__ == '__main__':
    main()
