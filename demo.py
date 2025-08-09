"""
🤖 ROBOT DEMO - Computer Vision Test
This demo shows you how the robot's "eyes" work using your computer's camera!

Since you're new to robotics, this will help you understand:
1. How the robot "sees" the world
2. How AI object detection works
3. How the robot makes decisions

No hardware needed - just your computer's camera!
"""

import cv2
import time
import numpy as np
from ultralytics import YOLO

class RobotDemo:
    def __init__(self):
        print("🤖 STARTING ROBOT VISION DEMO")
        print("=" * 50)
        print("Loading AI brain (YOLO model)...")
        
        # Load YOLO model (will download automatically first time)
        self.model = YOLO('yolov5s.pt')
        print("✅ AI brain loaded successfully!")
        
        # Try to open camera
        print("Opening camera...")
        self.camera = cv2.VideoCapture(0)
        
        if not self.camera.isOpened():
            print("❌ No camera found. The demo will use a test image instead.")
            self.use_camera = False
        else:
            print("✅ Camera opened successfully!")
            self.use_camera = True
        
        print("\n🎯 WHAT THE ROBOT WILL DO:")
        print("- Green box = Safe objects (robot ignores)")
        print("- Red box = Obstacles (robot would avoid)")
        print("- Yellow text = Robot's decision")
        print("\nPress 'q' to quit, 'space' to pause")
        print("=" * 50)
    
    def analyze_frame(self, frame):
        """Analyze frame like the robot would"""
        # Run AI detection
        results = self.model(frame, conf=0.5, verbose=False)
        
        # Process detections
        detections = []
        obstacle_detected = False
        
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Get detection info
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = self.model.names[class_id]
                    
                    # Calculate center and size
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    width = int(x2 - x1)
                    height = int(y2 - y1)
                    
                    # Determine if this is an obstacle
                    # Robot considers people, chairs, etc. as obstacles
                    obstacle_classes = ['person', 'chair', 'couch', 'tv', 'laptop', 
                                      'mouse', 'keyboard', 'cell phone', 'bottle', 'cup']
                    is_obstacle = class_name in obstacle_classes
                    
                    # Check if object is in front center (robot's path)
                    frame_center = frame.shape[1] // 2
                    in_front = abs(center_x - frame_center) < frame.shape[1] * 0.3
                    
                    detection = {
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'center': [center_x, center_y],
                        'class': class_name,
                        'confidence': float(confidence),
                        'is_obstacle': is_obstacle,
                        'in_front': in_front,
                        'size': width * height
                    }
                    
                    detections.append(detection)
                    
                    if is_obstacle and in_front:
                        obstacle_detected = True
        
        return detections, obstacle_detected
    
    def draw_detections(self, frame, detections, robot_decision):
        """Draw detection boxes and robot decision"""
        height, width = frame.shape[:2]
        
        # Draw robot's "front zone" (where it looks for obstacles)
        zone_left = int(width * 0.35)
        zone_right = int(width * 0.65)
        cv2.rectangle(frame, (zone_left, 0), (zone_right, height), (255, 255, 0), 2)
        cv2.putText(frame, "ROBOT FRONT ZONE", (zone_left + 10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        
        # Draw detections
        for detection in detections:
            x1, y1, x2, y2 = detection['bbox']
            class_name = detection['class']
            confidence = detection['confidence']
            is_obstacle = detection['is_obstacle']
            in_front = detection['in_front']
            
            # Choose color based on obstacle status
            if is_obstacle and in_front:
                color = (0, 0, 255)  # Red for obstacles in front
                status = "OBSTACLE!"
            elif is_obstacle:
                color = (0, 165, 255)  # Orange for obstacles to side
                status = "obstacle (side)"
            else:
                color = (0, 255, 0)  # Green for safe objects
                status = "safe"
            
            # Draw bounding box
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{class_name} ({confidence:.2f}) - {status}"
            cv2.putText(frame, label, (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # Draw robot decision
        decision_color = (0, 255, 255)  # Yellow
        if robot_decision == "OBSTACLE DETECTED - TURN RIGHT":
            decision_color = (0, 0, 255)  # Red
        elif robot_decision == "PATH CLEAR - MOVE FORWARD":
            decision_color = (0, 255, 0)  # Green
            
        cv2.putText(frame, f"ROBOT DECISION: {robot_decision}", 
                   (10, height - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, decision_color, 2)
        
        # Draw instructions
        cv2.putText(frame, "Press 'q' to quit, 'space' to pause", 
                   (10, height - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return frame
    
    def run_demo(self):
        """Main demo loop"""
        paused = False
        
        try:
            while True:
                if self.use_camera:
                    ret, frame = self.camera.read()
                    if not ret:
                        print("Failed to capture frame")
                        break
                else:
                    # Create a test frame if no camera
                    frame = np.zeros((480, 640, 3), dtype=np.uint8)
                    cv2.putText(frame, "NO CAMERA - This is a demo frame", 
                               (50, 240), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                
                if not paused:
                    # Analyze frame like the robot would
                    detections, obstacle_detected = self.analyze_frame(frame)
                    
                    # Make robot decision
                    if obstacle_detected:
                        robot_decision = "OBSTACLE DETECTED - TURN RIGHT"
                    else:
                        robot_decision = "PATH CLEAR - MOVE FORWARD"
                    
                    # Draw results
                    frame = self.draw_detections(frame, detections, robot_decision)
                
                # Show frame
                cv2.imshow('🤖 Robot Vision Demo - How Your Robot Sees!', frame)
                
                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord(' '):  # Space bar
                    paused = not paused
                    print("Demo paused" if paused else "Demo resumed")
                
        except KeyboardInterrupt:
            print("\nDemo stopped by user")
        except Exception as e:
            print(f"Demo error: {e}")
        finally:
            if self.use_camera:
                self.camera.release()
            cv2.destroyAllWindows()
            print("🤖 Demo ended. Thanks for watching!")

def main():
    """Run the robot demo"""
    try:
        demo = RobotDemo()
        demo.run_demo()
    except Exception as e:
        print(f"Demo failed to start: {e}")
        print("Make sure you have a camera connected or try running the basic tests first")

if __name__ == "__main__":
    main()
