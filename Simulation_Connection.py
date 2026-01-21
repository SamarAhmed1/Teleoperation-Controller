import socket
import math
import time
import json

class SimulatorCalibrator:
    def __init__(self, host='localhost', port=12345):
        self.host = host
        self.port = port
        self.sock = None
        self.calibration = None
        self.simulator_starting_pos = None
        
    def connect_socket(self):
        """Connect to simulator with bidirectional capability"""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)  # Set timeout for receiving data
        self.sock.connect((self.host, self.port))
        print(f"Connected to simulator at {self.host}:{self.port}")
        
    def request_simulator_position(self):
        """Request current joint positions from simulator"""
        try:
            # Send request for simulator position
            request = {"command": "get_position", "data": ""}
            self.sock.sendall(json.dumps(request).encode('utf-8'))
            
            # Wait for response
            response = self.sock.recv(4096)
            if response:
                data = json.loads(response.decode('utf-8'))
                if data.get("type") == "position_update":
                        positions_deg = data.get("positions_deg", [])
                        return positions_deg
        except (socket.timeout, json.JSONDecodeError, ConnectionError) as e:
            print(f"Failed to get simulator position: {e}")
        return None
    
    def update_calibration_from_simulator(self):
        """Update calibration using current simulator position"""
        sim_pos_deg = self.request_simulator_position()
        if sim_pos_deg and len(sim_pos_deg) == 6:
            self.simulator_starting_pos = sim_pos_deg
            print(f"Got simulator starting position: {sim_pos_deg}")
            return True
        return False
    
    def send_calibrated_angles(self, xarm_angles_deg):
        """Send calibrated angles to simulator"""
        if self.calibration and self.simulator_starting_pos:
            try:
                # Use the renamed method
                calibrated_angles_deg = self.calibration.calibrate_to_position(
                    xarm_angles_deg, 
                    self.simulator_starting_pos,
                    reverse_order=True
                )
                # Convert to radians
                rad_angles = self.calibration.degrees_to_radians(calibrated_angles_deg)
                
                # CHANGE TO SIMPLE CSV INSTEAD OF JSON
                data = ','.join(f"{a:.6f}" for a in rad_angles)
                self.sock.sendall(data.encode('utf-8'))
                print(f"Sent calibrated angles (CSV): {rad_angles}")
                return True
                
            except Exception as e:
                print(f"Error sending calibrated angles: {e}")
                return False
        
        # Fallback: send raw angles if calibration not available
        try:
            # Just reverse and convert to radians
            rad_angles = [math.radians(a) for a in xarm_angles_deg[::-1]]
            data = ','.join(f"{a:.6f}" for a in rad_angles)
            self.sock.sendall(data.encode('utf-8'))
            print(f"Sent raw angles (fallback): {rad_angles}")
            return True
        except Exception as e:
            print(f"Error in fallback: {e}")
            return False

