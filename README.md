#  Intelligent Autonomous Robot with AI Vision

[![Python](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://python.org)
[![YOLOv5](https://img.shields.io/badge/YOLOv5-Ultralytics-green.svg)](https://github.com/ultralytics/yolov5)
[![Raspberry Pi](https://img.shields.io/badge/Platform-Raspberry%20Pi-red.svg)](https://raspberrypi.org)
[![AI Powered](https://img.shields.io/badge/AI-Powered-brightgreen.svg)](https://github.com/ultralytics/ultralytics)
[![Masters Project](https://img.shields.io/badge/Academic-Masters%20Level-gold.svg)](#)

> **🎓 Masters-Level Project**: Advanced autonomous navigation system combining state-of-the-art computer vision (YOLOv5) with sensor fusion for intelligent obstacle avoidance.

![Robot Vision Demo](demo_screenshot.png)
*Real-time AI object detection in action - Robot identifies obstacles and makes navigation decisions*



##  Core Innovation

This project demonstrates **advanced autonomous navigation** through:

1. **Multi-Modal Perception**: Camera vision + distance sensors
2. **Intelligent Decision Making**: Priority-based obstacle avoidance algorithms  
3. **Real-Time Processing**: Live AI inference on embedded hardware
4. **Robust Architecture**: Thread-safe, fault-tolerant system design

##  Features

- **AI Object Detection** - Real-time YOLOv5 inference
- **Dual Sensor System** - Camera + Ultrasonic sensors
- **Smart Decision Making** - Priority-based obstacle avoidance
- **Modular Design** - Clean, maintainable code
- **Complete Hardware Integration** - L298N motors + HC-SR04 sensors
- **Beginner Friendly** - Extensive documentation and demos






##  Quick Start

```bash
# 1. Clone repository
git clone <your-repo-url>
cd robotics

# 2. Run setup (Raspberry Pi)
chmod +x setup.sh
./setup.sh

# 3. Test installation
python test_robot.py

# 4. Start robot
python main.py
```

##  AI-Powered Navigation Logic

```
CAMERA INPUT →  YOLOv5 AI → DECISION ENGINE →  MOTOR CONTROL
```

**Smart Decision Tree:**
```
if ultrasonic_distance < 20cm:
 EMERGENCY STOP → Turn Left
elif AI_detects_obstacle_in_front():
     STRATEGIC AVOID → Turn Right  
else:
     ADVANCE → Move Forward
```


##  Project Structure

```
robotics/
├── main.py           # Main control loop
├── detection.py      # YOLO object detection
├── movement.py       # Motor & sensor control
├── config.py         # Configuration settings
├── README.md         # This file
├── requirements.txt  # Dependencies
├── setup.sh         # Auto setup script
├── test_robot.py    # Test suite
├── demo.py          # Vision demo
└── beginner_demo.py # Learning demo
```

## 🔌 Wiring Diagram

### L298N Motor Driver
```
L298N → Raspberry Pi GPIO
ENA   → GPIO 18 (PWM Left)
IN1   → GPIO 24 (Left Dir 1)
IN2   → GPIO 23 (Left Dir 2)
IN3   → GPIO 25 (Right Dir 1)
IN4   → GPIO 8  (Right Dir 2)
ENB   → GPIO 12 (PWM Right)
```

### HC-SR04 Sensors
```
Front Sensor: Trig→GPIO16, Echo→GPIO20
Side Sensor:  Trig→GPIO21, Echo→GPIO26
```

## 🧪 Testing

```bash
# Run all tests
python test_robot.py

# Test specific components
python test_robot.py camera    # Camera test
python test_robot.py yolo      # AI detection test
python test_robot.py movement  # Motor test
```

## 🎮 Demos

```bash
# Interactive learning demo
python beginner_demo.py

# Computer vision demo (with camera)
python demo.py
```



## 📄 License

This project is open source and available under the [MIT License](LICENSE).



[![GitHub stars](https://img.shields.io/github/stars/yourusername/robotics?style=social)](https://github.com/yourusername/robotics/stargazers)
[![GitHub forks](https://img.shields.io/github/forks/yourusername/robotics?style=social)](https://github.com/yourusername/robotics/network/members)
