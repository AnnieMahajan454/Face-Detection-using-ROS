#!/usr/bin/env python3
"""
Emotion Detection Node
This node subscribes to camera images and face positions, performs emotion detection,
and publishes detected emotions.
"""

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32MultiArray
from cv_bridge import CvBridge, CvBridgeError
import json

try:
    from deepface import DeepFace
    DEEPFACE_AVAILABLE = True
except ImportError:
    DEEPFACE_AVAILABLE = False
    rospy.logwarn("DeepFace not available, falling back to basic emotion detection")


class EmotionDetectorNode:
    def __init__(self):
        """Initialize the emotion detector node."""
        rospy.init_node('emotion_detector_node', anonymous=True)
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Store latest image and faces
        self.latest_image = None
        self.latest_faces = []
        
        # Emotion detection parameters
        self.detection_interval = rospy.get_param('~detection_interval', 1.0)  # seconds
        self.last_detection_time = rospy.Time.now()
        
        # Publishers
        self.emotion_pub = rospy.Publisher('/emotion_detection/emotions', 
                                         String, queue_size=1)
        self.emotion_image_pub = rospy.Publisher('/emotion_detection/annotated_image',
                                               Image, queue_size=1)
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, 
                                        self.image_callback)
        self.faces_sub = rospy.Subscriber('/face_detection/face_positions',
                                        Int32MultiArray, self.faces_callback)
        
        if not DEEPFACE_AVAILABLE:
            rospy.logwarn("Install deepface for emotion detection: pip install deepface")
        
        rospy.loginfo("Emotion Detection Node initialized successfully!")
    
    def image_callback(self, data):
        """Store the latest image."""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def faces_callback(self, data):
        """Store the latest face positions."""
        if len(data.data) > 0:
            # Reshape flat array to list of [x, y, w, h]
            self.latest_faces = []
            for i in range(0, len(data.data), 4):
                if i + 3 < len(data.data):
                    face = [data.data[i], data.data[i+1], 
                           data.data[i+2], data.data[i+3]]
                    self.latest_faces.append(face)
            
            # Process emotions if interval has passed
            current_time = rospy.Time.now()
            if (current_time - self.last_detection_time).to_sec() >= self.detection_interval:
                self.detect_and_publish_emotions()
                self.last_detection_time = current_time
        else:
            self.latest_faces = []
    
    def detect_emotions(self, face_img):
        """
        Detect emotion in a face image.
        
        Args:
            face_img: Face image as numpy array
            
        Returns:
            emotion: Detected emotion string
            confidence: Confidence score
        """
        if not DEEPFACE_AVAILABLE:
            return "neutral", 0.5
        
        try:
            # Use DeepFace for emotion detection
            result = DeepFace.analyze(face_img, actions=['emotion'], 
                                    enforce_detection=False, silent=True)
            
            # Handle both single face and multiple faces result
            if isinstance(result, list):
                result = result[0]
            
            # Get dominant emotion
            emotion = result['dominant_emotion']
            confidence = result['emotion'][emotion] / 100.0
            
            return emotion, confidence
            
        except Exception as e:
            rospy.logwarn(f"Emotion detection failed: {e}")
            return "unknown", 0.0
    
    def detect_and_publish_emotions(self):
        """Detect emotions for all faces and publish results."""
        if self.latest_image is None or len(self.latest_faces) == 0:
            return
        
        emotions_data = []
        annotated_image = self.latest_image.copy()
        
        for idx, face in enumerate(self.latest_faces):
            x, y, w, h = face
            
            # Extract face region with some padding
            padding = 10
            y1 = max(0, y - padding)
            y2 = min(self.latest_image.shape[0], y + h + padding)
            x1 = max(0, x - padding)
            x2 = min(self.latest_image.shape[1], x + w + padding)
            
            face_img = self.latest_image[y1:y2, x1:x2]
            
            if face_img.size == 0:
                continue
            
            # Detect emotion
            emotion, confidence = self.detect_emotions(face_img)
            
            emotions_data.append({
                'face_id': idx,
                'emotion': emotion,
                'confidence': confidence,
                'position': {'x': x, 'y': y, 'w': w, 'h': h}
            })
            
            # Draw emotion on image
            color = self.get_emotion_color(emotion)
            cv2.rectangle(annotated_image, (x, y), (x + w, y + h), color, 2)
            
            # Add emotion label
            label = f"{emotion} ({confidence:.2f})"
            cv2.putText(annotated_image, label, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Publish emotions as JSON string
        if emotions_data:
            emotion_msg = String()
            emotion_msg.data = json.dumps(emotions_data)
            self.emotion_pub.publish(emotion_msg)
            
            rospy.loginfo(f"Detected emotions: {[e['emotion'] for e in emotions_data]}")
        
        # Publish annotated image
        try:
            img_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            img_msg.header.stamp = rospy.Time.now()
            self.emotion_image_pub.publish(img_msg)
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def get_emotion_color(self, emotion):
        """Get color for emotion visualization."""
        colors = {
            'happy': (0, 255, 0),      # Green
            'sad': (255, 0, 0),        # Blue
            'angry': (0, 0, 255),      # Red
            'fear': (128, 0, 128),     # Purple
            'surprise': (0, 255, 255), # Yellow
            'disgust': (0, 128, 128),  # Teal
            'neutral': (128, 128, 128) # Gray
        }
        return colors.get(emotion.lower(), (255, 255, 255))


def main():
    """Main function."""
    try:
        emotion_detector = EmotionDetectorNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Emotion Detection Node shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Emotion Detection Node: {e}")


if __name__ == '__main__':
    main()
