#!/usr/bin/env python3
"""
Mood Chatbot Node
This node subscribes to emotion data and generates mood-based recommendations
like listening to music, going for a walk, or relaxation exercises.
"""

import rospy
from std_msgs.msg import String
import json
import random
from collections import deque


class MoodChatbotNode:
    def __init__(self):
        """Initialize the mood chatbot node."""
        rospy.init_node('mood_chatbot_node', anonymous=True)
        
        # Store recent emotions for context
        self.emotion_history = deque(maxlen=5)
        self.current_emotion = None
        
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
            ],
            'unknown': [
                "🤔 Take a moment to check in with yourself",
                "🧘 Practice mindfulness to understand your feelings better",
                "🎵 Some background music might help set the mood"
            ]
        }
        
        # Publishers
        self.recommendation_pub = rospy.Publisher('/chatbot/recommendations',
                                                String, queue_size=1)
        
        # Subscribers
        self.emotion_sub = rospy.Subscriber('/emotion_detection/emotions',
                                          String, self.emotion_callback)
        
        rospy.loginfo("Mood Chatbot Node initialized successfully!")
    
    def emotion_callback(self, data):
        """Process emotion data and generate recommendations."""
        try:
            emotions_data = json.loads(data.data)
            
            if not emotions_data:
                return
            
            # Get the primary emotion (first detected face)
            primary_emotion = emotions_data[0]['emotion']
            confidence = emotions_data[0]['confidence']
            
            # Update emotion history
            self.emotion_history.append(primary_emotion)
            self.current_emotion = primary_emotion
            
            # Generate and publish recommendation
            recommendation = self.generate_recommendation(primary_emotion, confidence)
            
            # Create recommendation message
            rec_msg = String()
            rec_data = {
                'emotion': primary_emotion,
                'confidence': confidence,
                'recommendation': recommendation,
                'timestamp': rospy.Time.now().to_sec()
            }
            rec_msg.data = json.dumps(rec_data)
            
            self.recommendation_pub.publish(rec_msg)
            rospy.loginfo(f"Recommendation for {primary_emotion}: {recommendation}")
            
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse emotion data: {e}")
        except Exception as e:
            rospy.logerr(f"Error processing emotions: {e}")
    
    def generate_recommendation(self, emotion, confidence):
        """
        Generate a recommendation based on emotion and confidence.
        
        Args:
            emotion: Detected emotion string
            confidence: Confidence score
            
        Returns:
            recommendation: Recommendation string
        """
        # Get recommendations for this emotion
        emotion_key = emotion.lower()
        if emotion_key not in self.recommendations:
            emotion_key = 'unknown'
        
        recs = self.recommendations[emotion_key]
        
        # Select recommendation based on history
        if len(self.emotion_history) >= 3:
            # Check if emotion is persistent
            recent = list(self.emotion_history)[-3:]
            if all(e == emotion for e in recent):
                # Add context about persistent emotion
                if emotion_key == 'sad':
                    return "💙 You've been feeling down for a while. " + random.choice(recs)
                elif emotion_key == 'angry':
                    return "😤 I notice you're still upset. " + random.choice(recs)
        
        # Return random recommendation with confidence consideration
        if confidence < 0.5:
            return f"🤔 Not quite sure, but maybe: {random.choice(recs)}"
        
        return random.choice(recs)
    
    def get_emotion_trend(self):
        """Analyze emotion trend from history."""
        if len(self.emotion_history) < 2:
            return "stable"
        
        recent = list(self.emotion_history)
        if recent[-1] != recent[-2]:
            return "changing"
        
        return "stable"


def main():
    """Main function."""
    try:
        chatbot = MoodChatbotNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mood Chatbot Node shutting down")
    except Exception as e:
        rospy.logerr(f"Error in Mood Chatbot Node: {e}")


if __name__ == '__main__':
    main()
