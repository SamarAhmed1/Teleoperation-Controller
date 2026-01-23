# xarm_teleop_main.py
import socket
import json
import time
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
            
            # Try to connect to receive calibration data FROM Isaac Sim
            try:
                self.calibration_receiver = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.calibration_receiver.connect(('localhost', receive_port))
                print(f"Connected to Isaac Sim (receive) on port {receive_port}")
                
                # Start thread to receive calibration data
                import threading
                self.receive_thread = threading.Thread(target=self._receive_calibration_data, daemon=True)
                self.receive_thread.start()
                
            except ConnectionRefusedError:
                print(f"Note: Isaac Sim not listening on port {receive_port} (teleoperation only)")
                self.calibration_receiver = None
            
            return True
            
        except ConnectionRefusedError:
            print(f"Could not connect to Isaac Sim on port {send_port}. Make sure it's running.")
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
        """Send ALL joint angles to Isaac Sim"""
        if self.sim_sender:
            try:
                # Send ALL joints as-is
                # Important: Keep array reversal to match simulator coordinate system
                reversed_angles = angles_deg[::-1]  # Reverse all 6 joints
                reversed_angles[1] *= -1
                reversed_angles[3] *= -1
                
                message = {
                    'joints': reversed_angles,  # Send all 6 joints (reversed)
                    'timestamp': time.time(),
                    'comment': 'All joints teleoperation'
                }
                
                self.sim_sender.sendall(json.dumps(message).encode('utf-8'))
                return True
            except Exception as e:
                print(f"Error sending angles: {e}")
                return False
        return False


def main():
    # Initialize calibrator
    calibrator = SimpleXArmCalibrator()
    
    # Try to connect to Isaac Sim
    if not calibrator.connect_to_simulator():
        print("Failed to connect to Isaac Sim. Exiting...")
        return
    
    # Setup XArm
    print("Connecting to xArm...")
    arm = Controller('USB')
    
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
            
            time.sleep(0.033)  # ~30Hz for real-time control
            
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