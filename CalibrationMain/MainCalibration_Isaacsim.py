import time
from xarm import Controller
import numpy as np

#from Simple_Teleoperation import TeleoperationConnection
from ManualCalibration_Isaacsim import SimpleXArmCalibrator


def main():
    # Initialize calibrator
    calibrator = SimpleXArmCalibrator()
    #TeleOperator = TeleoperationConnection()
    
    # Try to connect to Isaac Sim
    if not calibrator.connect_to_simulator():
        print("Failed to connect to Isaac Sim. Exiting...")
        return
    
    # Setup XArm
    print("Connecting to xArm...")
    arm = Controller('USB')

    print("Waiting for UR10e initial pose: ")
    time.sleep(10) #Give isaacsim time to start

    ur10e_initial_pose_deg  = None
    if calibrator.sim_current_pos:
        ur10e_initial_pose_deg = calibrator.sim_current_pos
    else:
        #Fallback to hardcode pose if no connection:
        ur10e_initial_pose_deg = np.degrees([0.0, 0.0, 0.0, -1.57, 1.57, 0.0])
        print(f"Using fallback pose: {[round(a, 1) for a in ur10e_initial_pose_deg]}")
    
    # Store initial xArm angles BEFORE initialization
    print("\nRecording initial xArm pose BEFORE initialization...")
    xarm_before_init = []
    for i in range(1, 7):
        pos = arm.getPosition(i, False)
        angle_deg = (pos - 500) * 0.25
        xarm_before_init.append(angle_deg)
    print(f"xArm BEFORE initialization: {[round(a, 1) for a in xarm_before_init]}°")
    
    # Initialize xArm to match UR10e
    calibrator.initialize_xarm_to_ur10e(arm, ur10e_initial_pose_deg)

    print("Getting XArm current positions...")
    xarm_start_positions = []
    for i in range(1, 7):
        pos = arm.getPosition(i, False)
        angle_deg = (pos - 500) * 0.25
        xarm_start_positions.append(angle_deg)
    
    print(f"xArm starting positions: {[round(a, 1) for a in xarm_start_positions]}°")
    time.sleep(1)
    
    prev_angles = xarm_start_positions[:]
    
    print("\n========================================")
    print("Starting FULL TELEOPERATION...")
    print(f"ALL 6 joints will be sent to simulator")
    print(f"xArm joints: 1-6 mapped to UR10e joints 1-6")
    print(f"Move the xArm to see it mirrored in Isaac Sim!")
    print("Press Ctrl+C to stop")
    print("========================================\n")
    
    try:
        while True:
            # Read XArm angles
            curr_angles = []
            for i in range(1, 7):
                try:
                    pos = arm.getPosition(i, False)
                    angle_deg = (pos - 500) * 0.25
                    curr_angles.append(angle_deg)
                except Exception:
                    curr_angles.append(prev_angles[i-1])
            
            # Check if any position changed (for display)
            changed = any(abs(p - c) > 0.3 for p, c in zip(prev_angles, curr_angles))
            
            # Send ALL joints to Isaac Sim
            if changed:
                # Send raw angles
                success = calibrator.send_xarm_angles(curr_angles)
                
                if success:
                    # Display all joints being sent
                    formatted_angles = [round(a, 1) for a in curr_angles]
                    print(f"Sent all joints: {formatted_angles}")
            
            # Update previous angles
            prev_angles = curr_angles[:]

            if changed:
                order = [6, 5, 4, 3, 2, 1]
                arm.servoOff(order)
                #print("  (Servos released)")
            
            #time.sleep(0.033)  # ~30Hz for real-time control
            
    except KeyboardInterrupt:
        print("\n\nStopping teleoperation...")
    
    except Exception as e:
        print(f"\nError: {e}")
    
    finally:
        # Cleanup
        print("Disconnecting...")
        if calibrator.sim_sender:
            calibrator.sim_sender.close()
        if calibrator.calibration_receiver:
            calibrator.calibration_receiver.close()
        print("Disconnected from Isaac Sim.")

if __name__ == "__main__":
    main()