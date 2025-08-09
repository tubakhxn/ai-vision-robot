"""
Object Detection Module for Obstacle Avoidance Robot
Uses YOLOv5 for real-time object detection from Pi Camera

Hardware: Raspberry Pi 4 + Pi Camera Module
Dependencies: OpenCV, ultralytics (YOLOv5), torch
"""

import cv2
import torch
from ultralytics import YOLO
import numpy as np
from threading import Thread, Lock
import time

class ObjectDetector:
    def __init__(self, model_name='yolov5s.pt', confidence_threshold=0.5):
        """
        Initialize YOLO object detector
        
        Args:
            model_name (str): YOLOv5 model variant (yolov5s.pt for speed)
            confidence_threshold (float): Minimum confidence for detections
        """
        print("Initializing YOLO object detector...")
        
        # Load YOLOv5 model (will download on first run)
        self.model = YOLO(model_name)
        self.confidence_threshold = confidence_threshold
        
        # Detection results storage
        self.latest_detections = []
        self.detection_lock = Lock()
        
        # Camera setup for Pi Camera
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera.set(cv2.CAP_PROP_FPS, 30)
        
        # Verify camera connection
        if not self.camera.isOpened():
            raise RuntimeError("Failed to open Pi Camera. Check connection and enable camera interface.")
        
        print("YOLO detector initialized successfully!")
    
    def detect_objects(self, frame):
        """
        Perform object detection on a single frame
        
        Args:
            frame: OpenCV image frame
            
        Returns:
            list: Detected objects with bounding boxes and confidence scores
        """
        # Run YOLO inference
        results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        
        detections = []
        
        # Process detection results
        for result in results:
            boxes = result.boxes
            if boxes is not None:
                for box in boxes:
                    # Extract bounding box coordinates
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    confidence = box.conf[0].cpu().numpy()
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = self.model.names[class_id]
                    
                    # Calculate center point and area
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)
                    width = int(x2 - x1)
                    height = int(y2 - y1)
                    area = width * height
                    
                    detection = {
                        'class_name': class_name,
                        'confidence': float(confidence),
                        'bbox': [int(x1), int(y1), int(x2), int(y2)],
                        'center': [center_x, center_y],
                        'area': area
                    }
                    detections.append(detection)
        
        return detections
    
    def is_object_in_front(self, detections, frame_shape, front_zone_width=0.4):
        """
        Check if any detected object is in the front zone of the camera view
        
        Args:
            detections (list): List of detected objects
            frame_shape (tuple): Frame dimensions (height, width)
            front_zone_width (float): Width of front detection zone (0.0-1.0)
            
        Returns:
            bool: True if object detected in front zone
        """
        if not detections:
            return False
        
        height, width = frame_shape[:2]
        
        # Define front zone boundaries (center portion of frame)
        left_boundary = int(width * (0.5 - front_zone_width/2))
        right_boundary = int(width * (0.5 + front_zone_width/2))
        
        # Check if any object center is in the front zone
        for detection in detections:
            center_x = detection['center'][0]
            if left_boundary <= center_x <= right_boundary:
                return True
        
        return False
    
    def estimate_distance_from_size(self, detection, reference_size=100):
        """
        Rough distance estimation based on object size in pixels
        Larger objects are assumed to be closer
        
        Args:
            detection (dict): Detection with area information
            reference_size (int): Reference pixel area for distance calculation
            
        Returns:
            float: Estimated distance in arbitrary units
        """
        area = detection['area']
        if area == 0:
            return float('inf')
        
        # Simple inverse relationship: larger area = closer object
        estimated_distance = reference_size / (area ** 0.5)
        return estimated_distance
    
    def capture_and_detect(self):
        """
        Continuous camera capture and object detection loop
        Runs in separate thread to avoid blocking main program
        """
        while True:
            ret, frame = self.camera.read()
            if not ret:
                print("Warning: Failed to capture frame from camera")
                continue
            
            # Perform object detection
            detections = self.detect_objects(frame)
            
            # Update latest detections thread-safely
            with self.detection_lock:
                self.latest_detections = detections
            
            # Small delay to prevent excessive CPU usage
            time.sleep(0.1)
    
    def start_detection_thread(self):
        """
        Start background thread for continuous object detection
        """
        detection_thread = Thread(target=self.capture_and_detect, daemon=True)
        detection_thread.start()
        print("Object detection thread started")
    
    def get_latest_detections(self):
        """
        Get the most recent object detections (thread-safe)
        
        Returns:
            list: Latest detected objects
        """
        with self.detection_lock:
            return self.latest_detections.copy()
    
    def check_front_obstacle(self, distance_threshold=30):
        """
        Check if there's an object in front within specified distance
        
        Args:
            distance_threshold (float): Maximum distance to consider as obstacle
            
        Returns:
            bool: True if obstacle detected in front within threshold
        """
        detections = self.get_latest_detections()
        
        if not detections:
            return False
        
        # Get current frame for dimensions
        ret, frame = self.camera.read()
        if not ret:
            return False
        
        # Check if object is in front zone
        if self.is_object_in_front(detections, frame.shape):
            # Find the largest (closest) object in front
            front_objects = []
            height, width = frame.shape[:2]
            left_boundary = int(width * 0.3)
            right_boundary = int(width * 0.7)
            
            for detection in detections:
                center_x = detection['center'][0]
                if left_boundary <= center_x <= right_boundary:
                    front_objects.append(detection)
            
            if front_objects:
                # Find largest object (assumed closest)
                largest_object = max(front_objects, key=lambda x: x['area'])
                estimated_distance = self.estimate_distance_from_size(largest_object)
                
                # Convert to rough cm estimate (this needs calibration for your setup)
                distance_cm = estimated_distance * 10  # Rough conversion factor
                
                print(f"Object detected: {largest_object['class_name']} "
                      f"at estimated {distance_cm:.1f}cm")
                
                return distance_cm < distance_threshold
        
        return False
    
    def cleanup(self):
        """
        Clean up camera resources
        """
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()
        print("Object detector cleaned up")

# Test function for standalone testing
def test_detection():
    """
    Test function to verify object detection functionality
    """
    try:
        detector = ObjectDetector()
        detector.start_detection_thread()
        
        print("Testing object detection for 10 seconds...")
        start_time = time.time()
        
        while time.time() - start_time < 10:
            if detector.check_front_obstacle():
                print("OBSTACLE DETECTED IN FRONT!")
            else:
                print("Path clear")
            time.sleep(1)
        
        detector.cleanup()
        
    except Exception as e:
        print(f"Detection test failed: {e}")

if __name__ == "__main__":
    test_detection()
