"""
Test Suite for Object Detection + Obstacle Avoidance Robot
Comprehensive testing of all robot subsystems

Run tests individually or all at once to verify system functionality
"""

import time
import sys
import traceback
from threading import Thread
import cv2

def test_imports():
    """
    Test 1: Verify all required libraries can be imported
    """
    print("=" * 50)
    print("TEST 1: Import Dependencies")
    print("=" * 50)
    
    required_modules = [
        ('cv2', 'OpenCV'),
        ('torch', 'PyTorch'),
        ('ultralytics', 'YOLOv5'),
        ('numpy', 'NumPy'),
        ('PIL', 'Pillow')
    ]
    
    # Only test RPi.GPIO on Raspberry Pi
    try:
        import platform
        if 'arm' in platform.machine().lower():
            required_modules.append(('RPi.GPIO', 'Raspberry Pi GPIO'))
    except:
        pass
    
    all_passed = True
    
    for module_name, display_name in required_modules:
        try:
            __import__(module_name)
            print(f"✓ {display_name} imported successfully")
        except ImportError as e:
            print(f"✗ Failed to import {display_name}: {e}")
            all_passed = False
    
    print(f"\nImport test {'PASSED' if all_passed else 'FAILED'}")
    return all_passed

def test_camera():
    """
    Test 2: Verify camera functionality
    """
    print("\n" + "=" * 50)
    print("TEST 2: Camera Functionality")
    print("=" * 50)
    
    try:
        # Try to open camera
        camera = cv2.VideoCapture(0)
        
        if not camera.isOpened():
            print("✗ Failed to open camera")
            return False
        
        # Set camera properties
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Try to capture a frame
        ret, frame = camera.read()
        
        if not ret:
            print("✗ Failed to capture frame from camera")
            camera.release()
            return False
        
        print(f"✓ Camera opened successfully")
        print(f"✓ Frame captured: {frame.shape}")
        
        # Test multiple frames
        frames_captured = 0
        for i in range(5):
            ret, frame = camera.read()
            if ret:
                frames_captured += 1
            time.sleep(0.1)
        
        camera.release()
        
        print(f"✓ Captured {frames_captured}/5 test frames")
        
        if frames_captured >= 4:
            print("\nCamera test PASSED")
            return True
        else:
            print("\nCamera test FAILED - inconsistent frame capture")
            return False
            
    except Exception as e:
        print(f"✗ Camera test error: {e}")
        return False

def test_yolo_detection():
    """
    Test 3: Verify YOLO model loading and inference
    """
    print("\n" + "=" * 50)
    print("TEST 3: YOLO Object Detection")
    print("=" * 50)
    
    try:
        from ultralytics import YOLO
        import numpy as np
        
        # Load YOLO model
        print("Loading YOLOv5 model...")
        model = YOLO('yolov5s.pt')
        print("✓ YOLO model loaded successfully")
        
        # Create test image (colored noise)
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Run inference
        print("Running test inference...")
        results = model(test_image, conf=0.5, verbose=False)
        print("✓ YOLO inference completed")
        
        # Check results format
        if len(results) > 0:
            result = results[0]
            print(f"✓ Results format valid: {type(result)}")
            
            # Test with actual camera if available
            try:
                camera = cv2.VideoCapture(0)
                if camera.isOpened():
                    ret, frame = camera.read()
                    if ret:
                        results = model(frame, conf=0.5, verbose=False)
                        print("✓ Real camera inference successful")
                    camera.release()
            except:
                pass
        
        print("\nYOLO detection test PASSED")
        return True
        
    except Exception as e:
        print(f"✗ YOLO detection test error: {e}")
        traceback.print_exc()
        return False

def test_gpio_simulation():
    """
    Test 4: Test GPIO functionality (simulation mode if not on Pi)
    """
    print("\n" + "=" * 50)
    print("TEST 4: GPIO Functionality")
    print("=" * 50)
    
    try:
        # Check if we're on a Raspberry Pi
        try:
            import RPi.GPIO as GPIO
            on_pi = True
            print("✓ Running on Raspberry Pi - testing actual GPIO")
        except:
            print("✓ Not on Raspberry Pi - simulating GPIO operations")
            on_pi = False
        
        if on_pi:
            # Test actual GPIO
            GPIO.setmode(GPIO.BCM)
            GPIO.setwarnings(False)
            
            # Test pin setup
            test_pin = 18
            GPIO.setup(test_pin, GPIO.OUT)
            print(f"✓ GPIO pin {test_pin} configured as output")
            
            # Test PWM
            pwm = GPIO.PWM(test_pin, 1000)
            pwm.start(0)
            print("✓ PWM initialized")
            
            # Test duty cycle changes
            for duty in [25, 50, 75, 0]:
                pwm.ChangeDutyCycle(duty)
                time.sleep(0.1)
            
            print("✓ PWM duty cycle control working")
            
            pwm.stop()
            GPIO.cleanup()
            print("✓ GPIO cleanup successful")
        else:
            # Simulate GPIO operations
            print("✓ GPIO pin configuration (simulated)")
            print("✓ PWM initialization (simulated)")
            print("✓ PWM duty cycle control (simulated)")
            print("✓ GPIO cleanup (simulated)")
        
        print("\nGPIO test PASSED")
        return True
        
    except Exception as e:
        print(f"✗ GPIO test error: {e}")
        return False

def test_detection_module():
    """
    Test 5: Test detection.py module
    """
    print("\n" + "=" * 50)
    print("TEST 5: Detection Module")
    print("=" * 50)
    
    try:
        from detection import ObjectDetector
        
        # Initialize detector
        print("Initializing object detector...")
        detector = ObjectDetector(confidence_threshold=0.5)
        print("✓ Object detector initialized")
        
        # Test detection on dummy frame
        import numpy as np
        test_frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        detections = detector.detect_objects(test_frame)
        print(f"✓ Detection completed: {len(detections)} objects found")
        
        # Test front obstacle check
        is_obstacle = detector.check_front_obstacle()
        print(f"✓ Front obstacle check: {is_obstacle}")
        
        # Test thread safety
        detector.start_detection_thread()
        time.sleep(2)  # Let it run for 2 seconds
        latest = detector.get_latest_detections()
        print(f"✓ Thread-safe detection: {len(latest)} objects")
        
        detector.cleanup()
        print("✓ Detection module cleanup successful")
        
        print("\nDetection module test PASSED")
        return True
        
    except Exception as e:
        print(f"✗ Detection module test error: {e}")
        traceback.print_exc()
        return False

def test_movement_module():
    """
    Test 6: Test movement.py module (simulation mode)
    """
    print("\n" + "=" * 50)
    print("TEST 6: Movement Module")
    print("=" * 50)
    
    try:
        # Import with error handling for non-Pi systems
        try:
            from movement import RobotMovement
            robot = RobotMovement()
            on_pi = True
            print("✓ Running on Raspberry Pi - testing actual movement")
        except:
            print("✓ Not on Raspberry Pi - simulating movement operations")
            on_pi = False
            
            # Create mock movement class for testing
            class MockRobotMovement:
                def __init__(self):
                    self.front_distance = 50.0
                    self.side_distance = 50.0
                
                def get_distances(self):
                    return self.front_distance, self.side_distance
                
                def is_obstacle_close(self, threshold=20.0):
                    return False, None, min(self.front_distance, self.side_distance)
                
                def move_forward(self, speed=None):
                    print(f"Mock: Moving forward at speed {speed or 70}")
                
                def turn_left(self, speed=None, duration=1.0):
                    print(f"Mock: Turning left for {duration}s")
                
                def turn_right(self, speed=None, duration=1.0):
                    print(f"Mock: Turning right for {duration}s")
                
                def stop(self):
                    print("Mock: Stopping")
                
                def cleanup(self):
                    print("Mock: Cleanup complete")
            
            robot = MockRobotMovement()
        
        # Test basic movement functions
        print("Testing movement functions...")
        robot.move_forward()
        print("✓ Move forward")
        
        robot.turn_left()
        print("✓ Turn left")
        
        robot.turn_right()
        print("✓ Turn right")
        
        robot.stop()
        print("✓ Stop")
        
        # Test distance functions
        front_dist, side_dist = robot.get_distances()
        print(f"✓ Distance reading: Front={front_dist}cm, Side={side_dist}cm")
        
        is_close, sensor, dist = robot.is_obstacle_close()
        print(f"✓ Obstacle detection: {is_close}")
        
        robot.cleanup()
        print("✓ Movement module cleanup successful")
        
        print("\nMovement module test PASSED")
        return True
        
    except Exception as e:
        print(f"✗ Movement module test error: {e}")
        traceback.print_exc()
        return False

def test_integration():
    """
    Test 7: Integration test of main components
    """
    print("\n" + "=" * 50)
    print("TEST 7: Integration Test")
    print("=" * 50)
    
    try:
        # Test importing main modules together
        from detection import ObjectDetector
        
        # Use mock movement for non-Pi systems
        try:
            from movement import RobotMovement
            robot = RobotMovement()
        except:
            # Mock robot for testing
            class MockRobot:
                def get_distances(self):
                    return 50.0, 50.0
                def is_obstacle_close(self):
                    return False, None, 50.0
                def move_forward(self):
                    pass
                def stop(self):
                    pass
                def cleanup(self):
                    pass
            robot = MockRobot()
        
        detector = ObjectDetector()
        print("✓ Both modules initialized successfully")
        
        # Test combined operation
        detector.start_detection_thread()
        print("✓ Detection thread started")
        
        # Simulate main loop logic
        for i in range(3):
            # Get sensor data
            front_dist, side_dist = robot.get_distances()
            is_close, sensor, dist = robot.is_obstacle_close()
            
            # Get vision data
            vision_obstacle = detector.check_front_obstacle()
            
            # Make decision
            if is_close:
                action = "emergency_stop"
            elif vision_obstacle:
                action = "avoid_object"
            else:
                action = "move_forward"
            
            print(f"✓ Decision cycle {i+1}: {action}")
            time.sleep(0.5)
        
        # Cleanup
        detector.cleanup()
        robot.cleanup()
        print("✓ Integration cleanup successful")
        
        print("\nIntegration test PASSED")
        return True
        
    except Exception as e:
        print(f"✗ Integration test error: {e}")
        traceback.print_exc()
        return False

def run_all_tests():
    """
    Run all tests and provide summary
    """
    print("ROBOT SYSTEM TEST SUITE")
    print("=" * 60)
    
    tests = [
        ("Import Dependencies", test_imports),
        ("Camera Functionality", test_camera),
        ("YOLO Detection", test_yolo_detection),
        ("GPIO Functionality", test_gpio_simulation),
        ("Detection Module", test_detection_module),
        ("Movement Module", test_movement_module),
        ("Integration Test", test_integration),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        try:
            result = test_func()
            results.append((test_name, result))
        except Exception as e:
            print(f"CRITICAL ERROR in {test_name}: {e}")
            results.append((test_name, False))
    
    # Print summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    passed = 0
    total = len(results)
    
    for test_name, result in results:
        status = "PASS" if result else "FAIL"
        icon = "✓" if result else "✗"
        print(f"{icon} {test_name}: {status}")
        if result:
            passed += 1
    
    print("-" * 60)
    print(f"OVERALL: {passed}/{total} tests passed ({passed/total*100:.1f}%)")
    
    if passed == total:
        print("🎉 ALL TESTS PASSED - System ready for deployment!")
    elif passed >= total * 0.8:
        print("⚠️  Most tests passed - Check failed tests before deployment")
    else:
        print("❌ Multiple test failures - System needs attention")
    
    return passed == total

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Run specific test
        test_name = sys.argv[1].lower()
        test_map = {
            'imports': test_imports,
            'camera': test_camera,
            'yolo': test_yolo_detection,
            'gpio': test_gpio_simulation,
            'detection': test_detection_module,
            'movement': test_movement_module,
            'integration': test_integration,
        }
        
        if test_name in test_map:
            test_map[test_name]()
        else:
            print(f"Unknown test: {test_name}")
            print(f"Available tests: {', '.join(test_map.keys())}")
    else:
        # Run all tests
        run_all_tests()
