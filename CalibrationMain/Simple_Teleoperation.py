# xarm_teleop_main.py
import socket
import json
import time

class TeleoperationConnection:
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
                        print(f"DEBUG RAW RECEIVED: joints_deg = {joints_deg}")

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