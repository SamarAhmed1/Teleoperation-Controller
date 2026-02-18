from xarm import Controller, Servo  
import time


arm = Controller('USB')
JointToID = {1: "Gripper", 2: "Wrist2", 3: "Wrist1", 4: "Elbow", 5: "Shoulder", 6: "Base"}
order = [6, 5, 4, 3, 2, 1]
position = [6.0, 0.0, 0.0, 0.0, 0.0, 0.0]

servos_to_move = [Servo(id, pos) for id, pos in zip(order, position)]

try:
    # Time setPosition command
    start_set = time.perf_counter()  # Highest precision timer
    arm.setPosition(servos_to_move, duration=500, wait=True)
    end_set = time.perf_counter()
    
    set_time = end_set - start_set
    print(f"setPosition took: {set_time:.4f} seconds")
    print(f"Expected: 1.000 seconds (duration=1000ms)")
    print(f"Difference: {abs(set_time - 1.0):.4f} seconds")
    
    # Time servoOff command  
    start_off = time.perf_counter()
    arm.servoOff(order)
    end_off = time.perf_counter()
    
    off_time = end_off - start_off
    print(f"servoOff took: {off_time:.4f} seconds")
    
    # Total time
    total = set_time + off_time
    print(f"\nTotal time: {total:.4f} seconds")
    

except KeyboardInterrupt:
    print("Stopped.")