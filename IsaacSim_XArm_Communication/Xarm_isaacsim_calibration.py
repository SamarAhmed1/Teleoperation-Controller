from xarm import Controller
import time
from CalibrationSystem import RobotCalibration
from Simulation_Connection import SimulatorCalibrator

calibrator = SimulatorCalibrator()
calibrator.connect_socket()

# Wait a moment for connection to stabilize
time.sleep(0.5)

# Initialize with default calibration
calibrator.calibration = RobotCalibration(source_robot='XArm', target_robot='UR10e')

# Try to get current simulator position
print("Requesting current simulator position...")
if calibrator.update_calibration_from_simulator():
    print("Calibration updated from simulator!")
    # Update the calibration object with simulator position
    calibrator.calibration.update_target_position(calibrator.simulator_starting_pos)
else:
    print("Using default calibration")

# Setup XArm
arm = Controller('USB')
prev_angles = [0.0] * 6

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
        
        # Check if changed
        changed = any(abs(p - c) > 0.3 for p, c in zip(prev_angles, curr_angles))
        
        if changed:
            # Send calibrated angles
            calibrator.send_calibrated_angles(curr_angles)
        prev_angles = curr_angles[:]
        time.sleep(0.03)
        
except KeyboardInterrupt:
    if calibrator.sock:
        calibrator.sock.close()
    print("Stopped.")