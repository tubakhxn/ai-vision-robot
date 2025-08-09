"""
🤖 SIMPLE ROBOT BRAIN SIMULATOR
Perfect for beginners! Shows you how the robot thinks and makes decisions.
"""

import time
import random
from ultralytics import YOLO

def robot_brain_demo():
    """Simulate how the robot's brain works"""
    print("🤖 ROBOT BRAIN SIMULATOR")
    print("=" * 60)
    print("Loading robot's AI brain...")
    
    # Load the AI model (same one the real robot uses)
    model = YOLO('yolov5s.pt')
    print("✅ Robot brain loaded! (YOLO AI model)")
    
    print("\n🧠 How the robot thinks:")
    print("1. Camera captures what it sees")
    print("2. AI analyzes the image") 
    print("3. Robot decides what to do")
    print("4. Motors move the robot")
    
    print("\n" + "=" * 60)
    print("🎮 SIMULATION STARTING...")
    print("Watch how the robot makes decisions!")
    print("=" * 60)
    
    # Simulate different scenarios
    scenarios = [
        {
            "scene": "Empty hallway ahead",
            "objects": [],
            "ultrasonic_distance": 50,
            "expected_action": "MOVE FORWARD"
        },
        {
            "scene": "Person walking in front",
            "objects": [{"name": "person", "position": "center", "distance": 25}],
            "ultrasonic_distance": 45,
            "expected_action": "STOP and TURN RIGHT"
        },
        {
            "scene": "Chair blocking path",
            "objects": [{"name": "chair", "position": "center", "distance": 15}],
            "ultrasonic_distance": 18,
            "expected_action": "EMERGENCY STOP and TURN LEFT"
        },
        {
            "scene": "Cat walking to the side",
            "objects": [{"name": "cat", "position": "left", "distance": 30}],
            "ultrasonic_distance": 60,
            "expected_action": "MOVE FORWARD"
        },
        {
            "scene": "Table in front, book on side",
            "objects": [
                {"name": "dining table", "position": "center", "distance": 20},
                {"name": "book", "position": "right", "distance": 40}
            ],
            "ultrasonic_distance": 25,
            "expected_action": "STOP and TURN RIGHT"
        }
    ]
    
    for i, scenario in enumerate(scenarios, 1):
        print(f"\n📸 SCENARIO {i}: {scenario['scene']}")
        print("-" * 40)
        
        # Show what the robot "sees"
        print("👁️  ROBOT'S VISION:")
        if scenario['objects']:
            for obj in scenario['objects']:
                print(f"   • {obj['name']} detected at {obj['position']} ({obj['distance']}cm away)")
        else:
            print("   • No objects detected")
        
        # Show sensor data
        print(f"📡 ULTRASONIC SENSOR: {scenario['ultrasonic_distance']}cm")
        
        # Simulate thinking process
        print("🧠 ROBOT THINKING...")
        time.sleep(1)  # Pause for effect
        
        # Apply robot logic
        print("⚙️  DECISION LOGIC:")
        
        # Priority 1: Ultrasonic emergency
        if scenario['ultrasonic_distance'] < 20:
            decision = "🚨 EMERGENCY STOP - Turn Left"
            reason = f"Ultrasonic sensor detected obstacle at {scenario['ultrasonic_distance']}cm (< 20cm threshold)"
        
        # Priority 2: Vision obstacle in center
        elif any(obj['position'] == 'center' and obj['distance'] < 30 for obj in scenario['objects']):
            decision = "🛑 STOP - Turn Right" 
            reason = "AI detected object in front path within 30cm"
        
        # Priority 3: Path clear
        else:
            decision = "✅ MOVE FORWARD"
            reason = "Path is clear - no obstacles detected"
        
        print(f"   Reason: {reason}")
        print(f"🤖 ROBOT ACTION: {decision}")
        
        # Show expected vs actual
        if decision.replace("🚨 ", "").replace("🛑 ", "").replace("✅ ", "") == scenario['expected_action']:
            print("✅ Decision matches expected behavior!")
        else:
            print(f"⚠️  Expected: {scenario['expected_action']}")
        
        input("\nPress Enter to continue to next scenario...")
    
    print("\n" + "=" * 60)
    print("🎓 WHAT YOU LEARNED:")
    print("✅ How AI object detection works (YOLO)")
    print("✅ How robots prioritize different sensors")
    print("✅ How robots make decisions based on sensor data")
    print("✅ Why robots need both vision AND distance sensors")
    
    print("\n🚀 NEXT STEPS:")
    print("1. Get the hardware components (Raspberry Pi, motors, sensors)")
    print("2. Follow the wiring diagram in README.md")
    print("3. Run the real robot code: python main.py")
    print("4. Watch your robot navigate autonomously!")
    
    print("\n🤖 End of simulation. Happy robotics learning!")

if __name__ == "__main__":
    robot_brain_demo()
