from xarm import Controller, Servo  
import time


arm = Controller('USB')
prev_angles = [0.0] * 6  # Previous angles [Base,Shoulder,Elbow,Wrist1,Wrist2,Gripper]
JointToID = {1: "Gripper", 2: "Wrist2", 3: "Wrist1", 4: "Elbow", 5: "Shoulder", 6: "Base"}
order = [6,5,4,3,2,1]

try:
        curr_angles = [20, -3.5, 5.5, -75.6, 106.2, -35.4]
        arm.setPosition(servos = 6, position = -50.5, duration = 1000, wait = False)
        changed = False
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopped.")