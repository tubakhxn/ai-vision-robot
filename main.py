"""
Main Control Loop for Object Detection + Obstacle Avoidance Robot

This is the main entry point that coordinates:
1. YOLO object detection from Pi Camera
2. Ultrasonic sensor distance monitoring  
3. Motor control for autonomous navigation

Behavior Logic:
- Priority 1: Ultrasonic sensor obstacle < 20cm → Emergency stop and turn left
- Priority 2: YOLO object detection in front within 30cm → Stop and turn right
- Priority 3: Path clear → Move forward

Hardware Requirements:
- Raspberry Pi 4
- Pi Camera Module
- 2x HC-SR04 Ultrasonic Sensors
- L298N Motor Driver
- DC Motors and chassis

Author: Robotics Project
Date: August 2025
"""

import time
import signal
import sys
from threading import Event
from detection import ObjectDetector
from movement import RobotMovement

class ObstacleAvoidanceRobot:
    def __init__(self):
        """
        Initialize the complete robot system
        """
        print("=" * 50)
        print("OBSTACLE AVOIDANCE ROBOT STARTING...")
        print("=" * 50)
        
        # Create exit event for clean shutdown
        self.exit_event = Event()
        
        # Initialize detection system
        print("\n1. Initializing object detection system...")
        try:
            self.detector = ObjectDetector(
                model_name='yolov5s.pt',  # Fast model for real-time detection
                confidence_threshold=0.5
            )
            print("✓ Object detection system ready")
        except Exception as e:
            print(f"✗ Failed to initialize object detection: {e}")
            raise
        
        # Initialize movement system
        print("\n2. Initializing movement control system...")
        try:
            self.movement = RobotMovement()
            print("✓ Movement control system ready")
        except Exception as e:
            print(f"✗ Failed to initialize movement system: {e}")
            raise
        
        # Configuration parameters
        self.ULTRASONIC_THRESHOLD = 20.0  # cm - Emergency stop distance
        self.VISION_THRESHOLD = 30.0      # cm - YOLO detection distance
        self.LOOP_DELAY = 0.2            # seconds - Main loop frequency
        
        # State tracking
        self.last_action = "startup"
        self.consecutive_clear_count = 0
        
        print("\n3. Starting monitoring threads...")
        
        # Start background threads
        self.detector.start_detection_thread()
        self.movement.start_distance_monitoring()
        
        print("✓ All systems initialized successfully!")
        print("\nRobot is ready for autonomous operation.")
        print("Press Ctrl+C to stop the robot safely.\n")
    
    def signal_handler(self, signum, frame):
        """
        Handle Ctrl+C gracefully
        """
        print("\n\nShutdown signal received...")
        self.exit_event.set()
    
    def analyze_situation(self):
        """
        Analyze current sensor data and determine appropriate action
        
        Returns:
            str: Action to take ('emergency_stop', 'avoid_object', 'move_forward', 'stop')
        """
        # Get ultrasonic sensor data
        front_distance, side_distance = self.movement.get_distances()
        is_close, sensor_name, closest_distance = self.movement.is_obstacle_close(
            threshold=self.ULTRASONIC_THRESHOLD
        )
        
        # Get YOLO detection data
        vision_obstacle = self.detector.check_front_obstacle(
            distance_threshold=self.VISION_THRESHOLD
        )
        
        # Decision logic with priority system
        
        # Priority 1: Emergency stop for very close obstacles (ultrasonic)
        if is_close:
            return 'emergency_stop', {
                'sensor': sensor_name,
                'distance': closest_distance,
                'front_distance': front_distance,
                'side_distance': side_distance
            }
        
        # Priority 2: Avoid objects detected by vision (YOLO)
        elif vision_obstacle:
            return 'avoid_object', {
                'detection_type': 'vision',
                'front_distance': front_distance,
                'side_distance': side_distance
            }
        
        # Priority 3: Path is clear - move forward
        else:
            return 'move_forward', {
                'front_distance': front_distance,
                'side_distance': side_distance
            }
    
    def execute_action(self, action, context):
        """
        Execute the determined action
        
        Args:
            action (str): Action to perform
            context (dict): Additional context information
        """
        current_time = time.strftime("%H:%M:%S")
        
        if action == 'emergency_stop':
            if self.last_action != 'emergency_stop':
                print(f"\n[{current_time}] 🚨 EMERGENCY STOP!")
                print(f"Obstacle detected by {context['sensor']} sensor at {context['distance']:.1f}cm")
            
            self.movement.emergency_stop_and_avoid()
            self.consecutive_clear_count = 0
            
        elif action == 'avoid_object':
            if self.last_action != 'avoid_object':
                print(f"\n[{current_time}] 👁️  OBJECT DETECTED - Avoiding")
                print(f"Vision system detected obstacle in front")
            
            self.movement.stop()
            time.sleep(0.3)
            print("Turning right to avoid detected object...")
            self.movement.turn_right(duration=1.2)
            self.consecutive_clear_count = 0
            
        elif action == 'move_forward':
            self.consecutive_clear_count += 1
            
            # Only print status periodically to avoid spam
            if (self.last_action != 'move_forward' or 
                self.consecutive_clear_count % 25 == 0):  # Every 5 seconds at 5Hz
                
                print(f"[{current_time}] ✅ Path clear - Moving forward "
                      f"(Front: {context['front_distance']:.1f}cm, "
                      f"Side: {context['side_distance']:.1f}cm)")
            
            self.movement.move_forward()
            
        else:
            # Fallback - stop robot
            self.movement.stop()
        
        self.last_action = action
    
    def run(self):
        """
        Main control loop
        """
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print("🤖 ROBOT AUTONOMOUS MODE ACTIVE")
        print("Monitoring sensors and making navigation decisions...\n")
        
        try:
            while not self.exit_event.is_set():
                # Analyze current situation
                action, context = self.analyze_situation()
                
                # Execute appropriate action
                self.execute_action(action, context)
                
                # Control loop frequency
                time.sleep(self.LOOP_DELAY)
                
        except KeyboardInterrupt:
            print("\nKeyboard interrupt received")
        except Exception as e:
            print(f"\nUnexpected error in main loop: {e}")
        finally:
            self.shutdown()
    
    def shutdown(self):
        """
        Clean shutdown of all systems
        """
        print("\n" + "=" * 50)
        print("SHUTTING DOWN ROBOT SYSTEMS...")
        print("=" * 50)
        
        # Stop motors first for safety
        print("1. Stopping motors...")
        try:
            self.movement.stop()
            print("✓ Motors stopped")
        except Exception as e:
            print(f"✗ Error stopping motors: {e}")
        
        # Clean up movement system
        print("2. Cleaning up movement system...")
        try:
            self.movement.cleanup()
            print("✓ Movement system cleaned up")
        except Exception as e:
            print(f"✗ Error cleaning up movement: {e}")
        
        # Clean up detection system
        print("3. Cleaning up detection system...")
        try:
            self.detector.cleanup()
            print("✓ Detection system cleaned up")
        except Exception as e:
            print(f"✗ Error cleaning up detection: {e}")
        
        print("\n🤖 Robot shutdown complete. Goodbye!")

def main():
    """
    Main entry point
    """
    try:
        # Create and run robot
        robot = ObstacleAvoidanceRobot()
        robot.run()
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nFatal error: {e}")
        print("Please check hardware connections and try again.")
    finally:
        print("\nProgram terminated.")

if __name__ == "__main__":
    main()
