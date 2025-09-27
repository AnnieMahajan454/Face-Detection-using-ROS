#!/usr/bin/env python3
"""
Face Detection Node
This node subscribes to camera images, performs face detection using OpenCV,
and publishes the detected faces and annotated images.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import Header, Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError


class FaceDetectorNode:
    def __init__(self):
        """Initialize the face detector node."""
        rospy.init_node('face_detector_node', anonymous=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Load face cascade classifier
        cascade_path = rospy.get_param('~cascade_path', 
                                     cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        
        if self.face_cascade.empty():
            rospy.logerr("Failed to load face cascade classifier!")
            return
        
        # Detection parameters
        self.scale_factor = rospy.get_param('~scale_factor', 1.1)
        self.min_neighbors = rospy.get_param('~min_neighbors', 5)
        self.min_size = rospy.get_param('~min_size', [30, 30])
        self.max_size = rospy.get_param('~max_size', [300, 300])
        
        # Publishers
        self.annotated_image_pub = rospy.Publisher('/face_detection/annotated_image', 
                                                 Image, queue_size=1)
        self.face_positions_pub = rospy.Publisher('/face_detection/face_positions', 
                                                Int32MultiArray, queue_size=1)
        
        # Subscriber
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, 
                                        self.image_callback)
        
        rospy.loginfo("Face Detection Node initialized successfully!")
    
    def detect_faces(self, gray_image):
        """
        Detect faces in the grayscale image.
        
        Args:
            gray_image: Grayscale image as numpy array
            
        Returns:
            faces: Array of face rectangles (x, y, w, h)
        """
        faces = self.face_cascade.detectMultiScale(
            gray_image,
            scaleFactor=self.scale_factor,
            minNeighbors=self.min_neighbors,
            minSize=tuple(self.min_size),
            maxSize=tuple(self.max_size)
        )
        return faces
    
    def draw_faces(self, image, faces):
        """
        Draw rectangles around detected faces.
        
        Args:
            image: Color image as numpy array
            faces: Array of face rectangles
            
        Returns:
            annotated_image: Image with face rectangles drawn
        """
        annotated_image = image.copy()
        
        for (x, y, w, h) in faces:
            # Draw rectangle around face
            cv2.rectangle(annotated_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            
            # Add label
            cv2.putText(annotated_image, 'Face', (x, y - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Add face count
        face_count = len(faces)
        cv2.putText(annotated_image, f'Faces: {face_count}', (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
        
        return annotated_image
    
    def publish_face_positions(self, faces, header):
        """
        Publish face positions as Int32MultiArray.
        
        Args:
            faces: Array of face rectangles
            header: ROS message header
        """
        if len(faces) > 0:
            # Flatten faces array: [x1, y1, w1, h1, x2, y2, w2, h2, ...]
            face_data = faces.flatten().tolist()
        else:
            face_data = []
        
        msg = Int32MultiArray()
        msg.data = face_data
        
        self.face_positions_pub.publish(msg)
    
    def image_callback(self, data):
        """
        Callback function for image messages.
        
        Args:
            data: ROS Image message
        """
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
            return
        
        # Convert to grayscale for face detection
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = self.detect_faces(gray_image)
        
        # Draw faces on original image
        annotated_image = self.draw_faces(cv_image, faces)
        
        # Publish annotated image
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            annotated_msg.header = data.header
            self.annotated_image_pub.publish(annotated_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        
        # Publish face positions
        self.publish_face_positions(faces, data.header)
        
        # Log detection results
        if len(faces) > 0:
            rospy.loginfo(f"Detected {len(faces)} face(s)")


def main():
    """Main function."""
    try:
        face_detector = FaceDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Face Detection Node shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Face Detection Node: {e}")


if __name__ == '__main__':
    main()
