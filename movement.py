"""
Movement Control Module for Obstacle Avoidance Robot
Controls DC motors via L298N motor driver and HC-SR04 ultrasonic sensors

Hardware Connections:
- L298N Motor Driver connected to GPIO pins for motor control
- HC-SR04 Ultrasonic sensors for distance measurement
- External 12V power supply for motors

GPIO Pin Configuration:
- Motor control: GPIO 18, 24, 23, 25, 8, 12
- Ultrasonic sensors: GPIO 16, 20, 21, 26
"""

import RPi.GPIO as GPIO
import time
from threading import Thread, Lock

class RobotMovement:
    def __init__(self):
        """
        Initialize motor control and ultrasonic sensors
        """
        print("Initializing robot movement system...")
        
        # GPIO pin definitions for L298N Motor Driver
        self.MOTOR_LEFT_PWM = 18    # ENA - Left motor speed control
        self.MOTOR_LEFT_IN1 = 24    # IN1 - Left motor direction
        self.MOTOR_LEFT_IN2 = 23    # IN2 - Left motor direction
        
        self.MOTOR_RIGHT_PWM = 12   # ENB - Right motor speed control  
        self.MOTOR_RIGHT_IN3 = 25   # IN3 - Right motor direction
        self.MOTOR_RIGHT_IN4 = 8    # IN4 - Right motor direction
        
        # GPIO pin definitions for HC-SR04 Ultrasonic Sensors
        self.FRONT_SENSOR_TRIG = 16  # Front sensor trigger
        self.FRONT_SENSOR_ECHO = 20  # Front sensor echo
        
        self.SIDE_SENSOR_TRIG = 21   # Side sensor trigger  
        self.SIDE_SENSOR_ECHO = 26   # Side sensor echo
        
        # Motor speed settings (0-100)
        self.DEFAULT_SPEED = 70
        self.TURN_SPEED = 60
        
        # Distance measurement variables
        self.front_distance = 100.0  # cm
        self.side_distance = 100.0   # cm
        self.distance_lock = Lock()
        
        # Setup GPIO
        self.setup_gpio()
        
        # Initialize PWM for motor speed control
        self.setup_motors()
        
        print("Robot movement system initialized successfully!")
    
    def setup_gpio(self):
        """
        Configure GPIO pins for motors and sensors
        """
        # Set GPIO mode
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        
        # Setup motor control pins as outputs
        motor_pins = [
            self.MOTOR_LEFT_PWM, self.MOTOR_LEFT_IN1, self.MOTOR_LEFT_IN2,
            self.MOTOR_RIGHT_PWM, self.MOTOR_RIGHT_IN3, self.MOTOR_RIGHT_IN4
        ]
        
        for pin in motor_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # Setup ultrasonic sensor pins
        GPIO.setup(self.FRONT_SENSOR_TRIG, GPIO.OUT)
        GPIO.setup(self.FRONT_SENSOR_ECHO, GPIO.IN)
        GPIO.setup(self.SIDE_SENSOR_TRIG, GPIO.OUT)
        GPIO.setup(self.SIDE_SENSOR_ECHO, GPIO.IN)
        
        # Initialize trigger pins to LOW
        GPIO.output(self.FRONT_SENSOR_TRIG, GPIO.LOW)
        GPIO.output(self.SIDE_SENSOR_TRIG, GPIO.LOW)
    
    def setup_motors(self):
        """
        Initialize PWM for motor speed control
        """
        # Create PWM instances for speed control (1000 Hz frequency)
        self.left_motor_pwm = GPIO.PWM(self.MOTOR_LEFT_PWM, 1000)
        self.right_motor_pwm = GPIO.PWM(self.MOTOR_RIGHT_PWM, 1000)
        
        # Start PWM with 0% duty cycle (motors stopped)
        self.left_motor_pwm.start(0)
        self.right_motor_pwm.start(0)
    
    def measure_distance(self, trig_pin, echo_pin, timeout=0.03):
        """
        Measure distance using HC-SR04 ultrasonic sensor
        
        Args:
            trig_pin (int): GPIO pin for trigger
            echo_pin (int): GPIO pin for echo
            timeout (float): Maximum time to wait for echo
            
        Returns:
            float: Distance in centimeters
        """
        try:
            # Send 10μs pulse to trigger
            GPIO.output(trig_pin, GPIO.HIGH)
            time.sleep(0.00001)  # 10 microseconds
            GPIO.output(trig_pin, GPIO.LOW)
            
            # Wait for echo to start
            pulse_start = time.time()
            timeout_start = pulse_start
            
            while GPIO.input(echo_pin) == GPIO.LOW:
                pulse_start = time.time()
                if pulse_start - timeout_start > timeout:
                    return 999.0  # Timeout - assume far distance
            
            # Wait for echo to end
            pulse_end = time.time()
            timeout_start = pulse_end
            
            while GPIO.input(echo_pin) == GPIO.HIGH:
                pulse_end = time.time()
                if pulse_end - timeout_start > timeout:
                    return 999.0  # Timeout - assume far distance
            
            # Calculate distance
            pulse_duration = pulse_end - pulse_start
            # Speed of sound = 34300 cm/s, divide by 2 for round trip
            distance = (pulse_duration * 34300) / 2
            
            # Clamp distance to reasonable range
            return max(2.0, min(400.0, distance))
            
        except Exception as e:
            print(f"Distance measurement error: {e}")
            return 999.0
    
    def update_distances(self):
        """
        Continuously update distance measurements from both sensors
        Runs in background thread
        """
        while True:
            try:
                # Measure front distance
                front_dist = self.measure_distance(
                    self.FRONT_SENSOR_TRIG, 
                    self.FRONT_SENSOR_ECHO
                )
                
                # Small delay between measurements
                time.sleep(0.05)
                
                # Measure side distance
                side_dist = self.measure_distance(
                    self.SIDE_SENSOR_TRIG, 
                    self.SIDE_SENSOR_ECHO
                )
                
                # Update distances thread-safely
                with self.distance_lock:
                    self.front_distance = front_dist
                    self.side_distance = side_dist
                
                # Debug output every 2 seconds
                if int(time.time()) % 2 == 0:
                    print(f"Distances - Front: {front_dist:.1f}cm, Side: {side_dist:.1f}cm")
                
                time.sleep(0.1)  # 10Hz update rate
                
            except Exception as e:
                print(f"Distance update error: {e}")
                time.sleep(0.5)
    
    def start_distance_monitoring(self):
        """
        Start background thread for continuous distance monitoring
        """
        distance_thread = Thread(target=self.update_distances, daemon=True)
        distance_thread.start()
        print("Distance monitoring thread started")
    
    def get_distances(self):
        """
        Get current distance readings (thread-safe)
        
        Returns:
            tuple: (front_distance, side_distance) in cm
        """
        with self.distance_lock:
            return self.front_distance, self.side_distance
    
    def set_motor_speed(self, left_speed, right_speed):
        """
        Set motor speeds
        
        Args:
            left_speed (int): Left motor speed (-100 to 100)
            right_speed (int): Right motor speed (-100 to 100)
        """
        # Handle left motor
        if left_speed > 0:
            # Forward
            GPIO.output(self.MOTOR_LEFT_IN1, GPIO.HIGH)
            GPIO.output(self.MOTOR_LEFT_IN2, GPIO.LOW)
            self.left_motor_pwm.ChangeDutyCycle(abs(left_speed))
        elif left_speed < 0:
            # Backward
            GPIO.output(self.MOTOR_LEFT_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_LEFT_IN2, GPIO.HIGH)
            self.left_motor_pwm.ChangeDutyCycle(abs(left_speed))
        else:
            # Stop
            GPIO.output(self.MOTOR_LEFT_IN1, GPIO.LOW)
            GPIO.output(self.MOTOR_LEFT_IN2, GPIO.LOW)
            self.left_motor_pwm.ChangeDutyCycle(0)
        
        # Handle right motor
        if right_speed > 0:
            # Forward
            GPIO.output(self.MOTOR_RIGHT_IN3, GPIO.HIGH)
            GPIO.output(self.MOTOR_RIGHT_IN4, GPIO.LOW)
            self.right_motor_pwm.ChangeDutyCycle(abs(right_speed))
        elif right_speed < 0:
            # Backward
            GPIO.output(self.MOTOR_RIGHT_IN3, GPIO.LOW)
            GPIO.output(self.MOTOR_RIGHT_IN4, GPIO.HIGH)
            self.right_motor_pwm.ChangeDutyCycle(abs(right_speed))
        else:
            # Stop
            GPIO.output(self.MOTOR_RIGHT_IN3, GPIO.LOW)
            GPIO.output(self.MOTOR_RIGHT_IN4, GPIO.LOW)
            self.right_motor_pwm.ChangeDutyCycle(0)
    
    def move_forward(self, speed=None):
        """
        Move robot forward
        
        Args:
            speed (int): Motor speed (0-100), uses default if None
        """
        if speed is None:
            speed = self.DEFAULT_SPEED
        
        print(f"Moving forward at speed {speed}")
        self.set_motor_speed(speed, speed)
    
    def move_backward(self, speed=None):
        """
        Move robot backward
        
        Args:
            speed (int): Motor speed (0-100), uses default if None
        """
        if speed is None:
            speed = self.DEFAULT_SPEED
        
        print(f"Moving backward at speed {speed}")
        self.set_motor_speed(-speed, -speed)
    
    def turn_left(self, speed=None, duration=1.0):
        """
        Turn robot left
        
        Args:
            speed (int): Turn speed (0-100), uses turn speed if None
            duration (float): Turn duration in seconds
        """
        if speed is None:
            speed = self.TURN_SPEED
        
        print(f"Turning left for {duration}s at speed {speed}")
        self.set_motor_speed(-speed, speed)  # Left backward, right forward
        time.sleep(duration)
        self.stop()
    
    def turn_right(self, speed=None, duration=1.0):
        """
        Turn robot right
        
        Args:
            speed (int): Turn speed (0-100), uses turn speed if None
            duration (float): Turn duration in seconds
        """
        if speed is None:
            speed = self.TURN_SPEED
        
        print(f"Turning right for {duration}s at speed {speed}")
        self.set_motor_speed(speed, -speed)  # Left forward, right backward
        time.sleep(duration)
        self.stop()
    
    def stop(self):
        """
        Stop all motors immediately
        """
        print("Stopping robot")
        self.set_motor_speed(0, 0)
    
    def is_obstacle_close(self, threshold=20.0):
        """
        Check if any ultrasonic sensor detects obstacle within threshold
        
        Args:
            threshold (float): Distance threshold in cm
            
        Returns:
            tuple: (is_close, sensor_name, distance)
        """
        front_dist, side_dist = self.get_distances()
        
        if front_dist < threshold:
            return True, "front", front_dist
        elif side_dist < threshold:
            return True, "side", side_dist
        else:
            return False, None, min(front_dist, side_dist)
    
    def emergency_stop_and_avoid(self):
        """
        Emergency stop and avoidance maneuver
        Called when ultrasonic sensor detects close obstacle
        """
        print("EMERGENCY STOP - Obstacle detected by ultrasonic sensor!")
        self.stop()
        time.sleep(0.5)  # Brief pause
        
        # Back up slightly
        print("Backing up...")
        self.move_backward(50)
        time.sleep(0.8)
        self.stop()
        
        # Turn left to avoid obstacle
        print("Turning left to avoid obstacle")
        self.turn_left(duration=1.5)
    
    def cleanup(self):
        """
        Clean up GPIO resources
        """
        print("Cleaning up movement system...")
        self.stop()
        
        # Stop PWM
        if hasattr(self, 'left_motor_pwm'):
            self.left_motor_pwm.stop()
        if hasattr(self, 'right_motor_pwm'):
            self.right_motor_pwm.stop()
        
        # Clean up GPIO
        GPIO.cleanup()
        print("Movement system cleaned up")

# Test function for standalone testing
def test_movement():
    """
    Test function to verify movement functionality
    """
    try:
        robot = RobotMovement()
        robot.start_distance_monitoring()
        
        print("Testing robot movement for 15 seconds...")
        
        # Test sequence
        robot.move_forward()
        time.sleep(2)
        
        robot.turn_right()
        time.sleep(1)
        
        robot.move_forward()
        time.sleep(2)
        
        robot.turn_left()
        time.sleep(1)
        
        robot.stop()
        
        # Test distance monitoring
        for i in range(10):
            front_dist, side_dist = robot.get_distances()
            print(f"Front: {front_dist:.1f}cm, Side: {side_dist:.1f}cm")
            
            is_close, sensor, dist = robot.is_obstacle_close()
            if is_close:
                print(f"Obstacle detected by {sensor} sensor at {dist:.1f}cm!")
            
            time.sleep(1)
        
        robot.cleanup()
        
    except Exception as e:
        print(f"Movement test failed: {e}")

if __name__ == "__main__":
    test_movement()
