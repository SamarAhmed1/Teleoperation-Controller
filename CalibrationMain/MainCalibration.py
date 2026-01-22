import time
import msvcrt
from xarm import Controller

from Simple_Teleoperation import SimpleXArmCalibrator
from MultiPosition_Calibiration import MultiPositionCalibrator


def main():
    # Initialize calibrator
    calibrator = SimpleXArmCalibrator()
    
    # Try to connect to Isaac Sim
    if not calibrator.connect_to_simulator():
        print("Running without Isaac Sim connection...")
    
    # Setup XArm
    print("Connecting to xArm...")
    arm = Controller('USB')

    multi_calibrator = MultiPositionCalibrator(calibrator, arm)

    print("Getting XArm current positions")
    xarm_start_positions = []
    for i in range(1,7):
        pos = arm.getPosition(i, False)
        angle_deg = (pos - 500) * 0.25
        xarm_start_positions.append(angle_deg)
        print(f"xArm starting positions: {[round(a, 1) for a in xarm_start_positions]}°")

    time.sleep(1)
    prev_angles = xarm_start_positions[:]
    
    print("Starting teleoperation...")
    print("Move the xArm to see it in Isaac Sim!")
    print("Press Ctrl+C to stop")
    
    calibration_offsets = None
    try:
        while True:

            if msvcrt.kbhit():  # Check if key pressed
                key = msvcrt.getch()
                
                if key == b'1':  # Press '1' to record position
                    if multi_calibrator.add_position():
                        print(f"Recorded position {len(multi_calibrator.positions)}")
                
                elif key == b'2':  # Press '2' to calculate calibration
                    if len(multi_calibrator.positions) >= 2:
                        offsets = multi_calibrator.calculate_average_offsets()
                        if offsets:
                            calibration_offsets = offsets
                            print(f"MULTI-POSITION CALIBRATED! Offsets: {[round(o,1) for o in offsets]}")
                    else:
                        print(f"Need at least 2 positions, have {len(multi_calibrator.positions)}")
            # Read XArm angles
            curr_angles = []
            for i in range(1, 7):
                try:
                    pos = arm.getPosition(i, False)
                    angle_deg = (pos - 500) * 0.25
                    curr_angles.append(angle_deg)
                except Exception:
                    curr_angles.append(prev_angles[i-1])
            
            # Check if position changed
            changed = any(abs(p - c) > 0.3 for p, c in zip(prev_angles, curr_angles))
            
            # Send to Isaac Sim
            if changed:
                if calibration_offsets:
                    # Apply fixed calibration offsets
                    calibrated_angles = calibrator.apply_calibration(curr_angles, calibration_offsets)
                else:
                    calibrated_angles = curr_angles
                
                success = calibrator.send_xarm_angles(calibrated_angles)
                if success:
                    print(f"Sent: {[round(a, 1) for a in curr_angles]}")

            time.sleep(0.033)  # ~30Hz
            
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        # Cleanup
        if calibrator.sim_sender:
            calibrator.sim_sender.close()
        if calibrator.calibration_receiver:
            calibrator.calibration_receiver.close()
        print("Disconnected.")

if __name__ == "__main__":
    main()