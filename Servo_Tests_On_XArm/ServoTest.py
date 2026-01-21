import xarm
import time

#Servo IDs:
#ID 6: Base rotation
#ID 5: Shoulder
#ID 4: Elbow
#ID 3: Wrist 1
#ID 2: Wrist 2
#ID 1: Gripper/wrist roll

arm = xarm.Controller('USB')

position = []
JointToID = {1: "Gripper", 2: "Wrist2", 3: "Wrist1", 4: "Elbow", 5: "Shoulder", 6: "Base"}

try:
    while True:
        for id in [6,5,4,3,2,1]:
            angle = arm.getPosition(id, degrees=True)
            print(f"{JointToID[id]}:{angle:6.1f}")
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopped.")

arm.servoOff()

# position = arm.getPosition(6, True)
# print(position)
#angle = []
# for i in range (1,7):
#     position.append(arm.getPosition(i, False))
# print(position)