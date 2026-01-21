# xarm_teleop_main.py
import socket
import json
import time
import sys
import select
import msvcrt
from xarm import Controller

class SimpleXArmCalibrator:
    def __init__(self):
        # Connection to Isaac Sim
        self.sim_sender = None
        self.calibration_receiver = None
        # Calibration data
        self.sim_current_pos = None
        self.last_sim_update = 0
        
    def connect_to_simulator(self, send_port=12345, receive_port=12346):
        """Connect to Isaac Sim for sending and receiving data"""
        try:
            # Connect to send joint angles TO Isaac Sim
            self.sim_sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sim_sender.connect(('localhost', send_port))
            print(f"Connected to Isaac Sim (send) on port {send_port}")
            
            # Connect to receive calibration data FROM Isaac Sim
            self.calibration_receiver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.calibration_receiver.connect(('localhost', receive_port))
            print(f"Connected to Isaac Sim (receive) on port {receive_port}")
            
            # Start thread to receive calibration data
            import threading
            self.receive_thread = threading.Thread(target=self._receive_calibration_data, daemon=True)
            self.receive_thread.start()
            
            return True
            
        except ConnectionRefusedError:
            print(f"Could not connect to Isaac Sim. Make sure it's running.")
            return False
    
    def _receive_calibration_data(self):
        """Receive calibration data from Isaac Sim"""
        while True:
            try:
                data = self.calibration_receiver.recv(1024)
                if data:
                    msg = json.loads(data.decode('utf-8'))
                    
                    if msg.get('type') == 'isaac_position':
                        joints_rad = msg.get('joints_rad', [])
                        joints_deg = msg.get('joints_deg', [])
                        
                        # Update current simulator position
                        self.sim_current_pos = joints_deg
                        self.last_sim_update = time.time()

                        if time.time() - self.last_sim_update > 1.0:
                            print(f"Updated sim position: {[round(j, 1) for j in joints_deg]}")
                        
            except (json.JSONDecodeError, ConnectionError) as e:
                print(f"Error receiving calibration data: {e}")
                time.sleep(0.1)
            except Exception as e:
                time.sleep(0.1)
    
    def send_xarm_angles(self, angles_deg):
        # if self.sim_sender:
        #     try:
        #         # Test joint 1 only
        #         test_angles = [0, 0, 0, 0, 0, 0]
        #         test_angles[0] = angles_deg[0]  # Only send base
                
        #         message = {
        #             'joints': test_angles,
        #             'timestamp': time.time()
        #         }
                
        #         self.sim_sender.sendall(json.dumps(message).encode('utf-8'))
        #         print(f"DEBUG: Sent only base: {angles_deg[0]}")
        #         return True
        #     except Exception as e:
        #         print(f"Error sending angles: {e}")
        #         return False
        # return False
        """Send xArm angles to Isaac Sim"""
        if self.sim_sender:
            try:
                # Test: send raw reversed without any sign changes
                reversed_angles = angles_deg[::-1]
                
                message = {
                    'joints': reversed_angles,
                    'timestamp': time.time()
                }
                
                self.sim_sender.sendall(json.dumps(message).encode('utf-8'))
                return True
            except Exception as e:
                print(f"Error sending angles: {e}")
                return False
        return False
    
    def calculate_calibration_offset(self, xarm_angles, sim_angles):
        """Calculate the offset between xArm and simulator positions"""
        if len(xarm_angles) != 6 or len(sim_angles) != 6:
            return None
            
        # Simple offset calculation
        offsets = [sim - xarm for xarm, sim in zip(xarm_angles, sim_angles)]
        return offsets
    
    def apply_calibration(self, xarm_angles, offsets):
        """Apply calibration offsets to xArm angles"""
        if offsets and len(offsets) == 6:
            return [xarm + offset for xarm, offset in zip(xarm_angles, offsets)]
        return xarm_angles

# Main execution
def main():
    # Initialize calibrator
    calibrator = SimpleXArmCalibrator()
    
    # Try to connect to Isaac Sim
    if not calibrator.connect_to_simulator():
        print("Running without Isaac Sim connection...")
    
    # Setup XArm
    print("Connecting to xArm...")
    arm = Controller('USB')

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
                if key == b' ' and calibrator.sim_current_pos:
                    offsets = calibrator.calculate_calibration_offset(xarm_start_positions, calibrator.sim_current_pos)
                    if offsets:
                        calibration_offsets = offsets
                        print(f"CALIBRATED! Offsets: {[round(o, 1) for o in offsets]}")
           
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