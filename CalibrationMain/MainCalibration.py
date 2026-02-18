import time
from xarm import Controller
import numpy as np

#from Simple_Teleoperation import TeleoperationConnection
from ManualCalibiration import SimpleXArmCalibrator


def main():
    # Initialize calibrator
    calibrator = SimpleXArmCalibrator()
    #TeleOperator = TeleoperationConnection()
    
    # Try to connect to UR10e GUI (port 12345)
    if not calibrator.connect_to_simulator():
        print("Failed to connect to UR10e GUI. Exiting...")
        return
    
    # Setup XArm
    print("Connecting to xArm...")
    arm = Controller('USB')

    print("Waiting for UR10e initial pose: ")
    time.sleep(10)  # Give UR10e GUI time to start and send initial pose

    ur10e_initial_pose_deg = None
    if calibrator.sim_current_pos:
        ur10e_initial_pose_deg = calibrator.sim_current_pos
    else:
        # Fallback to hardcoded pose if no connection:
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
    # This will:
    # 1. Move xArm to the initial position
    # 2. Release servos so you can manually move it
    calibrator.initialize_xarm_to_ur10e(arm, ur10e_initial_pose_deg)

    print("\nGetting XArm current positions after initialization...")
    xarm_start_positions = []
    for i in range(1, 7):
        pos = arm.getPosition(i, False)
        angle_deg = (pos - 500) * 0.25
        xarm_start_positions.append(angle_deg)
    
    print(f"xArm starting positions: {[round(a, 1) for a in xarm_start_positions]}°")
    time.sleep(1)
    
    prev_angles = xarm_start_positions[:]
    
    print("\n" + "="*60)
    print("STARTING TELEOPERATION")
    print("="*60)
    print("Status: Servos are OFF - you can freely move the xArm")
    print("The UR10e will mirror your movements in real-time")
    print("Press Ctrl+C to stop")
    print("="*60 + "\n")
    
    consecutive_errors = 0
    MAX_CONSECUTIVE_ERRORS = 10
    
    try:
        loop_count = 0
        while True:
            loop_count += 1
            
            # Read XArm angles (servos are OFF, so we're reading encoder positions)
            curr_angles = []
            read_success = True
            
            for i in range(1, 7):
                try:
                    pos = arm.getPosition(i, False)
                    
                    # CRITICAL: Check if getPosition returned a valid number
                    if pos is False or pos is None or not isinstance(pos, (int, float)):
                        print(f"Warning: getPosition({i}) returned invalid value: {pos} (type: {type(pos)})")
                        # Use previous value
                        curr_angles.append(prev_angles[i-1])
                        read_success = False
                        continue
                    
                    # Convert position to angle
                    angle_deg = (pos - 500) * 0.25
                    curr_angles.append(angle_deg)
                    
                except Exception as e:
                    # If reading fails, use previous value
                    print(f"Warning: Exception reading joint {i}: {e}")
                    curr_angles.append(prev_angles[i-1])
                    read_success = False
            
            # Safety check: make sure we have 6 angles
            if len(curr_angles) != 6:
                print(f"ERROR: Expected 6 angles, got {len(curr_angles)}. Using previous values.")
                curr_angles = prev_angles[:]
                consecutive_errors += 1
                if consecutive_errors > MAX_CONSECUTIVE_ERRORS:
                    print(f"ERROR: Too many consecutive read errors ({consecutive_errors}). Stopping.")
                    break
                time.sleep(0.1)
                continue
            
            # Reset error counter on successful read
            if read_success:
                consecutive_errors = 0
            else:
                consecutive_errors += 1
                if consecutive_errors > MAX_CONSECUTIVE_ERRORS:
                    print(f"ERROR: Too many consecutive read errors ({consecutive_errors}). Stopping.")
                    break
            
            # Check if any position changed significantly
            try:
                changed = any(abs(p - c) > 0.3 for p, c in zip(prev_angles, curr_angles))
            except Exception as e:
                print(f"ERROR: Failed to compare angles: {e}")
                print(f"  prev_angles: {prev_angles}")
                print(f"  curr_angles: {curr_angles}")
                break
            
            # Send to UR10e when positions change
            if changed:
                try:
                    # Send converted angles to UR10e
                    success = calibrator.send_xarm_angles(curr_angles)
                    
                    if not success:
                        print("Warning: Failed to send angles to UR10e")
                        
                except Exception as e:
                    print(f"ERROR: Exception in send_xarm_angles: {e}")
                    import traceback
                    traceback.print_exc()
                    # Continue anyway, don't break the loop
            
            # Update previous angles
            try:
                prev_angles = curr_angles[:]
            except Exception as e:
                print(f"ERROR: Failed to copy curr_angles: {e}")
                print(f"  curr_angles type: {type(curr_angles)}")
                print(f"  curr_angles value: {curr_angles}")
                break
            
            # Small delay to prevent overwhelming the system (~30Hz)
            time.sleep(0.033)
            
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("STOPPING TELEOPERATION (User interrupted)")
        print("="*60)
    
    except Exception as e:
        print(f"\nERROR in main teleoperation loop: {e}")
        print(f"Error type: {type(e)}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        print("\nCleaning up...")
        
        # Note: Servos are already OFF, no need to turn them off again
        print("(Servos were already released during initialization)")
        
        # Close socket connections
        print("Disconnecting from UR10e GUI...")
        if calibrator.sim_sender:
            try:
                calibrator.sim_sender.close()
            except:
                pass
        if calibrator.calibration_receiver:
            try:
                calibrator.calibration_receiver.close()
            except:
                pass
        print("Disconnected.")
        print("\nTeleoperation stopped successfully.")

if __name__ == "__main__":
    main()