#!/usr/bin/env python3
"""
Standalone Emotion Detection Test
This script tests the emotion detection and chatbot without ROS.
Works on Windows, Linux, and macOS.
"""

import cv2
import numpy as np
import json
import random
from collections import deque
import tkinter as tk
from tkinter import scrolledtext
from PIL import Image as PILImage, ImageTk
import threading
import time

try:
    from deepface import DeepFace
    DEEPFACE_AVAILABLE = True
except ImportError:
    DEEPFACE_AVAILABLE = False
    print("⚠️ DeepFace not installed. Install with: pip install deepface")


class StandaloneEmotionDetector:
    def __init__(self):
        """Initialize the standalone emotion detector."""
        self.cap = None
        self.running = False
        self.current_emotion = "waiting..."
        self.recommendations_list = []
        self.emotion_history = deque(maxlen=5)
        
        # Load face cascade
        self.face_cascade = cv2.CascadeClassifier(
            cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        )
        
        # Recommendation database
        self.recommendations = {
            'happy': [
                "🎉 You're feeling great! Keep up the positive energy!",
                "🎵 Listen to some upbeat music to maintain your happy mood",
                "🌟 Share your happiness with someone today",
                "💃 Dance to your favorite song!",
                "📸 Capture this moment with a photo or selfie"
            ],
            'sad': [
                "💙 It's okay to feel sad sometimes. Take a moment for yourself",
                "🎶 Try listening to calming music or your comfort playlist",
                "🚶 A short walk outside might help clear your mind",
                "☕ Treat yourself to your favorite comfort food or drink",
                "📞 Consider reaching out to a friend or loved one",
                "📝 Writing down your thoughts might help process your feelings"
            ],
            'angry': [
                "😤 Take a few deep breaths to calm down",
                "🚶 Go for a walk to release some tension",
                "🎵 Listen to calming music or nature sounds",
                "🥊 Try some physical exercise to channel your energy",
                "💭 Count to 10 slowly before reacting",
                "🧘 Practice meditation or mindfulness exercises"
            ],
            'fear': [
                "🌟 Take deep breaths - you're safe right now",
                "🧘 Try a grounding technique: name 5 things you can see",
                "🎵 Listen to soothing music to calm your nerves",
                "💪 Remember past challenges you've overcome",
                "📞 Reach out to someone you trust if you need support"
            ],
            'surprise': [
                "😮 Unexpected moments make life interesting!",
                "🎉 Embrace the spontaneity of the moment",
                "📝 Journal about this surprising experience",
                "🤔 Take a moment to process what just happened"
            ],
            'disgust': [
                "😐 Step away from what's bothering you for a moment",
                "🚶 Take a short break and get some fresh air",
                "🎵 Listen to your favorite music to shift your mood",
                "💭 Focus on something pleasant or beautiful"
            ],
            'neutral': [
                "😊 You seem calm and balanced",
                "🎯 A good time to be productive or try something new",
                "🎵 Discover new music or podcasts",
                "🚶 A leisurely walk might be enjoyable",
                "📚 Read something interesting or learn a new skill"
            ]
        }
    
    def detect_faces(self, gray_image):
        """Detect faces in grayscale image."""
        faces = self.face_cascade.detectMultiScale(
            gray_image,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )
        return faces
    
    def detect_emotion(self, face_img):
        """Detect emotion in a face image."""
        if not DEEPFACE_AVAILABLE:
            return "neutral", 0.5
        
        try:
            result = DeepFace.analyze(face_img, actions=['emotion'], 
                                    enforce_detection=False, silent=True)
            
            if isinstance(result, list):
                result = result[0]
            
            emotion = result['dominant_emotion']
            confidence = result['emotion'][emotion] / 100.0
            
            return emotion, confidence
            
        except Exception as e:
            return "unknown", 0.0
    
    def get_emotion_color(self, emotion):
        """Get color for emotion visualization."""
        colors = {
            'happy': (0, 255, 0),
            'sad': (255, 0, 0),
            'angry': (0, 0, 255),
            'fear': (128, 0, 128),
            'surprise': (0, 255, 255),
            'disgust': (0, 128, 128),
            'neutral': (128, 128, 128)
        }
        return colors.get(emotion.lower(), (255, 255, 255))
    
    def generate_recommendation(self, emotion, confidence):
        """Generate recommendation based on emotion."""
        emotion_key = emotion.lower()
        if emotion_key not in self.recommendations:
            emotion_key = 'neutral'
        
        recs = self.recommendations[emotion_key]
        
        # Check persistent emotion
        if len(self.emotion_history) >= 3:
            recent = list(self.emotion_history)[-3:]
            if all(e == emotion for e in recent):
                if emotion_key == 'sad':
                    return "💙 You've been feeling down for a while. " + random.choice(recs)
                elif emotion_key == 'angry':
                    return "😤 I notice you're still upset. " + random.choice(recs)
        
        if confidence < 0.5:
            return f"🤔 Not quite sure, but maybe: {random.choice(recs)}"
        
        return random.choice(recs)


class EmotionGUIStandalone:
    def __init__(self, master):
        """Initialize the GUI."""
        self.master = master
        self.master.title("Emotion Detection & Mood Chatbot (Standalone)")
        self.master.geometry("1200x700")
        self.master.configure(bg='#2b2b2b')
        
        self.detector = StandaloneEmotionDetector()
        self.running = False
        self.last_detection_time = time.time()
        self.detection_interval = 1.5  # seconds
        
        self.create_widgets()
        
        # Start camera
        self.start_camera()
        
        print("✅ Emotion Detection GUI initialized!")
    
    def create_widgets(self):
        """Create GUI widgets."""
        main_frame = tk.Frame(self.master, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Left side - Video
        left_frame = tk.Frame(main_frame, bg='#1e1e1e', relief=tk.RAISED, borderwidth=2)
        left_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 5))
        
        video_title = tk.Label(left_frame, text="📹 Live Emotion Detection",
                             font=('Arial', 14, 'bold'), bg='#1e1e1e', fg='#ffffff')
        video_title.pack(pady=10)
        
        self.video_label = tk.Label(left_frame, bg='#000000')
        self.video_label.pack(padx=10, pady=10)
        
        self.emotion_label = tk.Label(left_frame, text="Emotion: waiting...",
                                     font=('Arial', 12), bg='#1e1e1e', fg='#00ff00')
        self.emotion_label.pack(pady=5)
        
        # Right side - Chatbot
        right_frame = tk.Frame(main_frame, bg='#1e1e1e', relief=tk.RAISED, borderwidth=2)
        right_frame.grid(row=0, column=1, sticky='nsew', padx=(5, 0))
        
        chatbot_title = tk.Label(right_frame, text="💬 Mood Recommendations",
                               font=('Arial', 14, 'bold'), bg='#1e1e1e', fg='#ffffff')
        chatbot_title.pack(pady=10)
        
        self.rec_text = scrolledtext.ScrolledText(right_frame, wrap=tk.WORD,
                                                 font=('Arial', 11),
                                                 bg='#2b2b2b', fg='#ffffff',
                                                 height=30, width=45)
        self.rec_text.pack(padx=10, pady=10, fill=tk.BOTH, expand=True)
        self.rec_text.insert(tk.END, "Starting camera...\n\n")
        
        # Buttons
        button_frame = tk.Frame(right_frame, bg='#1e1e1e')
        button_frame.pack(pady=10)
        
        clear_btn = tk.Button(button_frame, text="Clear History",
                            command=self.clear_recommendations,
                            bg='#404040', fg='#ffffff',
                            font=('Arial', 10), padx=10, pady=5)
        clear_btn.pack(side=tk.LEFT, padx=5)
        
        main_frame.columnconfigure(0, weight=2)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(0, weight=1)
    
    def start_camera(self):
        """Start camera capture."""
        self.detector.cap = cv2.VideoCapture(0)
        if not self.detector.cap.isOpened():
            print("❌ Failed to open camera")
            self.rec_text.insert(tk.END, "❌ Camera not found!\n")
            return
        
        self.running = True
        self.update_frame()
    
    def update_frame(self):
        """Update video frame."""
        if not self.running:
            return
        
        ret, frame = self.detector.cap.read()
        if not ret:
            self.master.after(30, self.update_frame)
            return
        
        # Detect faces
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.detector.detect_faces(gray)
        
        # Process emotion detection
        current_time = time.time()
        if len(faces) > 0 and (current_time - self.last_detection_time) >= self.detection_interval:
            x, y, w, h = faces[0]
            
            # Extract face
            padding = 10
            y1 = max(0, y - padding)
            y2 = min(frame.shape[0], y + h + padding)
            x1 = max(0, x - padding)
            x2 = min(frame.shape[1], x + w + padding)
            face_img = frame[y1:y2, x1:x2]
            
            if face_img.size > 0:
                # Detect emotion
                emotion, confidence = self.detector.detect_emotion(face_img)
                
                if emotion != "unknown":
                    # Update history
                    self.detector.emotion_history.append(emotion)
                    self.detector.current_emotion = f"{emotion} ({confidence:.2%})"
                    
                    # Generate recommendation
                    recommendation = self.detector.generate_recommendation(emotion, confidence)
                    
                    # Add to list
                    time_str = time.strftime("[%H:%M:%S]")
                    self.detector.recommendations_list.append({
                        'time': time_str,
                        'emotion': emotion,
                        'recommendation': recommendation
                    })
                    
                    self.update_recommendations_display()
                    self.last_detection_time = current_time
        
        # Draw faces with emotions
        for (x, y, w, h) in faces:
            color = self.detector.get_emotion_color(
                self.detector.current_emotion.split()[0] if self.detector.current_emotion != "waiting..." else "neutral"
            )
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, self.detector.current_emotion, (x, y - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Display frame
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = PILImage.fromarray(rgb_frame)
        img = img.resize((640, 480))
        photo = ImageTk.PhotoImage(image=img)
        
        self.video_label.configure(image=photo)
        self.video_label.image = photo
        
        # Update emotion label
        self.emotion_label.configure(text=f"Emotion: {self.detector.current_emotion}")
        
        # Schedule next update
        self.master.after(30, self.update_frame)
    
    def update_recommendations_display(self):
        """Update recommendations text."""
        self.rec_text.delete(1.0, tk.END)
        
        recent = self.detector.recommendations_list[-20:]
        for rec in recent:
            self.rec_text.insert(tk.END, f"{rec['time']}\n")
            self.rec_text.insert(tk.END, f"Emotion: {rec['emotion'].upper()}\n")
            self.rec_text.insert(tk.END, f"{rec['recommendation']}\n\n")
        
        self.rec_text.see(tk.END)
    
    def clear_recommendations(self):
        """Clear recommendations."""
        self.detector.recommendations_list = []
        self.rec_text.delete(1.0, tk.END)
        self.rec_text.insert(tk.END, "History cleared.\n\n")
    
    def on_closing(self):
        """Handle window closing."""
        self.running = False
        if self.detector.cap:
            self.detector.cap.release()
        self.master.destroy()


def main():
    """Main function."""
    print("🚀 Starting Emotion Detection System...")
    print("=" * 50)
    
    if not DEEPFACE_AVAILABLE:
        print("\n⚠️  WARNING: DeepFace not installed!")
        print("Install it with: pip install deepface tensorflow")
        print("\nContinuing with basic features...\n")
    
    root = tk.Tk()
    app = EmotionGUIStandalone(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        print("\n👋 Shutting down...")


if __name__ == '__main__':
    main()
