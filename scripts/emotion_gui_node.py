#!/usr/bin/env python3
"""
Emotion GUI Node
This node displays the camera feed with emotion detection annotations
and chatbot recommendations in a side-by-side GUI.
"""

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import json
import tkinter as tk
from tkinter import ttk, scrolledtext
from PIL import Image as PILImage, ImageTk
import threading


class EmotionGUI:
    def __init__(self, master):
        """Initialize the GUI."""
        self.master = master
        self.master.title("Emotion Detection & Mood Chatbot")
        self.master.geometry("1200x700")
        self.master.configure(bg='#2b2b2b')
        
        # Initialize ROS node
        rospy.init_node('emotion_gui_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Data storage
        self.current_image = None
        self.current_emotion = "waiting..."
        self.recommendations_list = []
        
        # Create GUI layout
        self.create_widgets()
        
        # ROS Subscribers
        self.image_sub = rospy.Subscriber('/emotion_detection/annotated_image',
                                        Image, self.image_callback)
        self.rec_sub = rospy.Subscriber('/chatbot/recommendations',
                                      String, self.recommendation_callback)
        
        # Start ROS in separate thread
        self.ros_thread = threading.Thread(target=self.ros_spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Update GUI periodically
        self.update_gui()
        
        rospy.loginfo("Emotion GUI initialized successfully!")
    
    def create_widgets(self):
        """Create the GUI widgets."""
        # Main container
        main_frame = tk.Frame(self.master, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left side - Video feed
        left_frame = tk.Frame(main_frame, bg='#1e1e1e', relief=tk.RAISED, borderwidth=2)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 5))
        
        # Video title
        video_title = tk.Label(left_frame, text="📹 Live Emotion Detection",
                             font=('Arial', 14, 'bold'), bg='#1e1e1e', fg='#ffffff')
        video_title.pack(pady=10)
        
        # Video display
        self.video_label = tk.Label(left_frame, bg='#000000')
        self.video_label.pack(padx=10, pady=10)
        
        # Emotion status
        self.emotion_label = tk.Label(left_frame, text="Emotion: waiting...",
                                     font=('Arial', 12), bg='#1e1e1e', fg='#00ff00')
        self.emotion_label.pack(pady=5)
        
        # Right side - Chatbot
        right_frame = tk.Frame(main_frame, bg='#1e1e1e', relief=tk.RAISED, borderwidth=2)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=(5, 0))
        
        # Chatbot title
        chatbot_title = tk.Label(right_frame, text="💬 Mood Recommendations",
                               font=('Arial', 14, 'bold'), bg='#1e1e1e', fg='#ffffff')
        chatbot_title.pack(pady=10)
        
        # Recommendations display
        self.rec_text = scrolledtext.ScrolledText(right_frame, wrap=tk.WORD,
                                                 font=('Arial', 11),
                                                 bg='#2b2b2b', fg='#ffffff',
                                                 insertbackground='#ffffff',
                                                 height=30, width=45)
        self.rec_text.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        self.rec_text.insert(tk.END, "Waiting for emotion detection...\n\n")
        self.rec_text.configure(state='disabled')
        
        # Control buttons
        button_frame = tk.Frame(right_frame, bg='#1e1e1e')
        button_frame.pack(pady=10)
        
        clear_btn = tk.Button(button_frame, text="Clear History",
                            command=self.clear_recommendations,
                            bg='#404040', fg='#ffffff',
                            font=('Arial', 10), padx=10, pady=5)
        clear_btn.pack(side=tk.LEFT, padx=5)
        
        # Configure grid weights
        main_frame.columnconfigure(0, weight=2)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
    
    def image_callback(self, data):
        """Callback for emotion annotated images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self.current_image = cv_image
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
    
    def recommendation_callback(self, data):
        """Callback for chatbot recommendations."""
        try:
            rec_data = json.loads(data.data)
            emotion = rec_data['emotion']
            confidence = rec_data['confidence']
            recommendation = rec_data['recommendation']
            
            self.current_emotion = f"{emotion} ({confidence:.2%})"
            
            # Add to recommendations list
            timestamp = rospy.Time.now()
            time_str = f"[{timestamp.secs % 86400 // 3600:02d}:{timestamp.secs % 3600 // 60:02d}:{timestamp.secs % 60:02d}]"
            
            self.recommendations_list.append({
                'time': time_str,
                'emotion': emotion,
                'recommendation': recommendation
            })
            
            # Update text widget
            self.master.after(0, self.update_recommendations_display)
            
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse recommendation data: {e}")
    
    def update_recommendations_display(self):
        """Update the recommendations text display."""
        self.rec_text.configure(state='normal')
        self.rec_text.delete(1.0, tk.END)
        
        if not self.recommendations_list:
            self.rec_text.insert(tk.END, "No recommendations yet...\n\n")
        else:
            # Show recent recommendations (last 20)
            recent = self.recommendations_list[-20:]
            for rec in recent:
                self.rec_text.insert(tk.END, f"{rec['time']}\n", 'time')
                self.rec_text.insert(tk.END, f"Emotion: {rec['emotion'].upper()}\n", 'emotion')
                self.rec_text.insert(tk.END, f"{rec['recommendation']}\n\n", 'rec')
        
        self.rec_text.configure(state='disabled')
        self.rec_text.see(tk.END)
    
    def clear_recommendations(self):
        """Clear the recommendations history."""
        self.recommendations_list = []
        self.rec_text.configure(state='normal')
        self.rec_text.delete(1.0, tk.END)
        self.rec_text.insert(tk.END, "History cleared.\n\n")
        self.rec_text.configure(state='disabled')
    
    def update_gui(self):
        """Update GUI elements periodically."""
        # Update video feed
        if self.current_image is not None:
            # Resize image to fit display
            display_width = 640
            display_height = 480
            resized = cv2.resize(self.current_image, (display_width, display_height))
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image
            pil_image = PILImage.fromarray(rgb_image)
            
            # Convert to PhotoImage
            photo = ImageTk.PhotoImage(image=pil_image)
            
            # Update label
            self.video_label.configure(image=photo)
            self.video_label.image = photo
        
        # Update emotion label
        self.emotion_label.configure(text=f"Emotion: {self.current_emotion}")
        
        # Schedule next update
        self.master.after(30, self.update_gui)
    
    def ros_spin(self):
        """Run ROS spin in separate thread."""
        rospy.spin()
    
    def on_closing(self):
        """Handle window closing."""
        rospy.loginfo("Shutting down Emotion GUI")
        self.master.destroy()


def main():
    """Main function."""
    root = tk.Tk()
    app = EmotionGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        rospy.loginfo("Emotion GUI shutting down")


if __name__ == '__main__':
    main()
