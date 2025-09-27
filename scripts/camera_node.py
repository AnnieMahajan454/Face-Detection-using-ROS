#!/usr/bin/env python3
"""
Camera Node
This node captures images from a camera and publishes them as ROS Image messages.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError


class CameraNode:
    def __init__(self):
        """Initialize the camera node."""
        rospy.init_node('camera_node', anonymous=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Camera parameters
        self.camera_index = rospy.get_param('~camera_index', 0)
        self.frame_rate = rospy.get_param('~frame_rate', 30.0)
        self.image_width = rospy.get_param('~image_width', 640)
        self.image_height = rospy.get_param('~image_height', 480)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(self.camera_index)
        if not self.cap.isOpened():
            rospy.logerr(f"Failed to open camera {self.camera_index}")
            return
        
        # Set camera properties
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)
        
        # Publishers
        self.image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=1)
        
        # Timer for publishing images
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.frame_rate), self.timer_callback)
        
        rospy.loginfo(f"Camera Node initialized with camera {self.camera_index}")
        rospy.loginfo(f"Resolution: {self.image_width}x{self.image_height}, FPS: {self.frame_rate}")
    
    def timer_callback(self, event):
        """
        Timer callback to capture and publish images.
        
        Args:
            event: ROS timer event
        """
        if not self.cap.isOpened():
            rospy.logwarn("Camera not opened")
            return
        
        # Capture frame
        ret, frame = self.cap.read()
        if not ret:
            rospy.logwarn("Failed to capture frame from camera")
            return
        
        # Create ROS Image message
        try:
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            img_msg.header = Header()
            img_msg.header.stamp = rospy.Time.now()
            img_msg.header.frame_id = "camera_frame"
            
            # Publish image
            self.image_pub.publish(img_msg)
            
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def shutdown(self):
        """Clean up camera resources."""
        if self.cap.isOpened():
            self.cap.release()
        rospy.loginfo("Camera Node shutting down")


def main():
    """Main function."""
    try:
        camera_node = CameraNode()
        rospy.on_shutdown(camera_node.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Camera Node interrupted")
    except Exception as e:
        rospy.logerr(f"Error in Camera Node: {e}")


if __name__ == '__main__':
    main()
