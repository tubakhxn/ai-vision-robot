"""
Configuration file for Object Detection + Obstacle Avoidance Robot
Centralized settings for easy adjustment and calibration
"""

# === HARDWARE PIN CONFIGURATIONS ===

# L298N Motor Driver GPIO Pins
MOTOR_PINS = {
    'LEFT_PWM': 18,     # ENA - Left motor speed control
    'LEFT_IN1': 24,     # IN1 - Left motor direction 1  
    'LEFT_IN2': 23,     # IN2 - Left motor direction 2
    'RIGHT_PWM': 12,    # ENB - Right motor speed control
    'RIGHT_IN3': 25,    # IN3 - Right motor direction 1
    'RIGHT_IN4': 8,     # IN4 - Right motor direction 2
}

# HC-SR04 Ultrasonic Sensor GPIO Pins
SENSOR_PINS = {
    'FRONT_TRIG': 16,   # Front sensor trigger
    'FRONT_ECHO': 20,   # Front sensor echo
    'SIDE_TRIG': 21,    # Side sensor trigger
    'SIDE_ECHO': 26,    # Side sensor echo
}

# === DETECTION PARAMETERS ===

# YOLO Object Detection Settings
DETECTION_CONFIG = {
    'MODEL_NAME': 'yolov5s.pt',      # YOLOv5 model (s=small/fast, m=medium, l=large)
    'CONFIDENCE_THRESHOLD': 0.5,      # Minimum confidence for object detection
    'VISION_DISTANCE_THRESHOLD': 30,  # cm - Stop if object detected within this distance
    'FRONT_ZONE_WIDTH': 0.4,         # Width of front detection zone (0.0-1.0)
}

# Camera Settings
CAMERA_CONFIG = {
    'WIDTH': 640,                     # Camera frame width
    'HEIGHT': 480,                    # Camera frame height  
    'FPS': 30,                        # Frames per second
}

# === MOVEMENT PARAMETERS ===

# Motor Speed Settings (0-100)
MOTOR_SPEEDS = {
    'DEFAULT_SPEED': 70,              # Normal forward movement speed
    'TURN_SPEED': 60,                 # Speed during turns
    'BACKUP_SPEED': 50,               # Speed when backing up
}

# Movement Timing (seconds)
MOVEMENT_TIMING = {
    'TURN_DURATION': 1.2,             # How long to turn when avoiding objects
    'BACKUP_DURATION': 0.8,           # How long to back up during emergency stop
    'EMERGENCY_TURN_DURATION': 1.5,   # How long to turn during emergency avoidance
}

# === SENSOR PARAMETERS ===

# Distance Thresholds (centimeters)
DISTANCE_THRESHOLDS = {
    'ULTRASONIC_EMERGENCY': 20,       # Emergency stop distance for ultrasonic sensors
    'ULTRASONIC_WARNING': 30,         # Warning distance for ultrasonic sensors
    'VISION_OBSTACLE': 30,            # Stop distance for vision-detected objects
}

# Sensor Update Rates (Hz)
SENSOR_RATES = {
    'DISTANCE_UPDATE_RATE': 10,       # Ultrasonic sensor reading frequency
    'DETECTION_UPDATE_RATE': 10,      # Object detection frequency
}

# === SYSTEM PARAMETERS ===

# Main Loop Settings
SYSTEM_CONFIG = {
    'MAIN_LOOP_DELAY': 0.2,          # seconds - Main control loop frequency
    'STATUS_PRINT_INTERVAL': 25,      # Loop cycles between status prints
    'SENSOR_TIMEOUT': 0.03,          # seconds - Ultrasonic sensor timeout
}

# Threading Settings
THREAD_CONFIG = {
    'DAEMON_THREADS': True,           # Use daemon threads for background tasks
    'THREAD_STARTUP_DELAY': 0.5,     # seconds - Delay after starting threads
}

# === CALIBRATION PARAMETERS ===

# Distance Estimation Calibration
CALIBRATION = {
    'PIXEL_TO_CM_REFERENCE': 100,     # Reference pixel area for distance estimation
    'DISTANCE_CONVERSION_FACTOR': 10,  # Conversion factor for vision distance estimation
    'MIN_DETECTION_AREA': 500,       # Minimum pixel area to consider as valid detection
}

# Safety Margins
SAFETY_MARGINS = {
    'EMERGENCY_STOP_MARGIN': 5,       # cm - Additional margin for emergency stops
    'TURN_CLEARANCE_MARGIN': 10,     # cm - Minimum clearance after turns
}

# === DEBUG AND LOGGING ===

# Debug Settings
DEBUG_CONFIG = {
    'ENABLE_DISTANCE_LOGGING': True,  # Print distance measurements
    'ENABLE_DETECTION_LOGGING': True, # Print object detections
    'ENABLE_ACTION_LOGGING': True,    # Print robot actions
    'LOG_INTERVAL': 2,                # seconds - Interval between debug logs
}

# Performance Monitoring
PERFORMANCE_CONFIG = {
    'MONITOR_FPS': False,             # Monitor detection frame rate
    'MONITOR_CPU_USAGE': False,       # Monitor CPU usage
    'PERFORMANCE_LOG_INTERVAL': 10,   # seconds
}

# === HARDWARE SPECIFIC SETTINGS ===

# PWM Configuration
PWM_CONFIG = {
    'FREQUENCY': 1000,                # Hz - PWM frequency for motor control
    'STARTUP_DUTY_CYCLE': 0,         # Initial PWM duty cycle
}

# GPIO Configuration  
GPIO_CONFIG = {
    'MODE': 'BCM',                    # GPIO numbering mode
    'WARNINGS': False,                # Disable GPIO warnings
    'CLEANUP_ON_EXIT': True,          # Clean up GPIO on program exit
}

# === BEHAVIOR TUNING ===

# Avoidance Behavior
BEHAVIOR_CONFIG = {
    'PREFER_RIGHT_TURNS': True,       # Prefer right turns for object avoidance
    'CONSECUTIVE_CLEAR_THRESHOLD': 5,  # Cycles of clear path before confident forward movement
    'STUCK_DETECTION_THRESHOLD': 10,  # Cycles of same action before considering stuck
}

# Adaptive Behavior
ADAPTIVE_CONFIG = {
    'ENABLE_SPEED_ADAPTATION': False, # Adjust speed based on obstacles
    'ENABLE_DYNAMIC_THRESHOLDS': False, # Adjust thresholds based on environment
    'LEARNING_RATE': 0.1,            # Rate of adaptive changes
}
