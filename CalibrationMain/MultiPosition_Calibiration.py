# xarm_teleop_main.py
from xarm import Controller

class MultiPositionCalibrator:
    def __init__(self, calibrator, arm):
        self.calibrator = calibrator
        self.arm = arm
        self.positions = []
        self.current_step = 0

    def add_position(self):
        #Record current XArm position and wait for simulator position
        xarm_pos = self.read_xarm_joint()
        print(f"Position {len(self.positions)+1}: xArm at {[round(p,1) for p in xarm_pos]}")
        print("Set Isaac Sim to matching pose, then press Enter")
        input()

        if self.calibrator.sim_current_pos:
            sim_pos = self.calibrator.sim_current_pos
            self.positions.append((xarm_pos, sim_pos))
            print(f"Recorded position {len(self.positions)}")
            return True
        return False
    
    def calculate_average_offsets(self):
        #Calculating average offsets from all recorded positions
        if len(self.positions) < 2:
            return None
        
        all_offsets = []
        for xarm_pos, sim_pos in self.positions:
            offsets = self.calibrator.calculate_calibration_offset(xarm_pos, sim_pos)
            if offsets:
                all_offsets.append(offsets)

        average_offsets = [sum(o[i] for o in all_offsets)/len(all_offsets) for i in range(6)]
        return average_offsets
    
    def read_xarm_joint(self):
            angles = []
            for i in range(1, 7):
                pos = self.arm.getPosition(i, False)
                angle_deg = (pos - 500) * 0.25
                angles.append(angle_deg)
            return angles
