from xarm import Controller
import time

arm = Controller('USB')
prev_angles = [0.0] * 6  # Previous angles [Base,Shoulder,Elbow,Wrist1,Wrist2,Gripper]
JointToID = {1: "Gripper", 2: "Wrist2", 3: "Wrist1", 4: "Elbow", 5: "Shoulder", 6: "Base"}
order = [6,5,4,3,2,1]

#Note: Curr_angles now appends values as a list
try:
    while True:
        curr_angles = []
        changed = False

        for id in order:
            angle = arm.getPosition(id, degrees=True)
            curr_angles.append(angle)

        # Check for noise :)
        for i, (prev, curr) in enumerate(zip(prev_angles, curr_angles)):
            if abs(prev - curr) > 0.3:
                changed = True
                break

        if changed:
            for i, id in enumerate(order):
                print(f"{JointToID[id]:<10}:{curr_angles[i]:6.1f} (Difference: {curr_angles[i]-prev_angles[i]:+5.1f})")

            print ("End\n")

        prev_angles = curr_angles[:]  # Update previous
        time.sleep(0.05)

except KeyboardInterrupt:
    print("Stopped.")


