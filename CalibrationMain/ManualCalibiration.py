# xarm_teleop_main.py
import socket
import json
import time
import numpy as np
from Simple_Teleoperation import TeleoperationConnection

Servo_offsets = {
    1: 2.2,
    2: 2,
    3: 2,
    4: 0.5,
    5: -3,
    6: 1
}

XARM_JOINT_LIMITS = {
    1: (-86.0, 31.5),   # Gripper
    2: (-125.0, 161.5), #wrist 2
    3: (-115.0, 130.8),   # Wrist1
    4: (-122.0, 136.5),   # Elbow
    5: (-95.2, 90),   # Shoulder
    6: (-125.0, 165.0),   # Base
}

class SimpleXArmCalibrator:
    def __init__(self):
        # Store initial xArm pose for relative movement
        self.xarm_initial_angles = None
        self.sim_sender = None
        self.calibration_receiver = None
        # Calibration data
        self.sim_current_pos = None
        self.last_sim_update = 0

    @staticmethod
    def clamp_to_xarm_limits(servo_id, angle_deg):
        """Clamp angle to xArm physical limits"""
        min_angle, max_angle = XARM_JOINT_LIMITS[servo_id]
        return max(min_angle, min(max_angle, angle_deg))

    def connect_to_simulator(self, send_port=12345, receive_port=12346):
        """Connect to Isaac Sim for sending and receiving data"""
        try:
            # Connect to send joint angles TO Isaac Sim
            self.sim_sender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sim_sender.connect(('localhost', send_port))
            print(f"Connected to send on port {send_port}")
            
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
            print(f"Could not connect to GUI on port {send_port}. Make sure it's running.")
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
    def send_xarm_angles(self, angles_deg):
        """Send xArm joint angles to Isaac Sim with ALL offsets compensation"""
        if self.sim_sender:
            try:
                # angles_deg = readings from xArm [ID1-ID6]
                # These readings INCLUDE servo offsets
                
                print(f"\n[XArm Raw] IDs 1-6: {[round(a, 1) for a in angles_deg]}")
    
                # STEP 1: Remove servo calibration offsets (small, 0-5°)
                servo_offset_degrees = {
                    1: Servo_offsets.get(1, 0) * 0.25,  # Gripper
                    2: Servo_offsets.get(2, 0) * 0.25,  # Wrist2
                    3: Servo_offsets.get(3, 0) * 0.25,  # Wrist1
                    4: Servo_offsets.get(4, 0) * 0.25,  # Elbow
                    5: Servo_offsets.get(5, 0) * 0.25,  # Shoulder
                    6: Servo_offsets.get(6, 0) * 0.25,  # Base
                }# offset_degrees = offset_raw * 0.25
                
                # Remove servo offsets to get "true visual" position
                visual_angles = []
                for i in range(6):
                    servo_id = i + 1
                    raw_angle = angles_deg[i]
                    visual_angle = raw_angle - servo_offset_degrees[servo_id]
                    visual_angles.append(visual_angle)
                    
                    print(f"  ID{servo_id}: {raw_angle:6.1f}° - servo {servo_offset_degrees[servo_id]:.2f}° = {visual_angle:6.1f}°")
                
                print(f"[Visual Angles after servo offset]: {[round(a, 1) for a in visual_angles]}")
    
                # STEP 2: Apply kinematic offsets (large, up to 90°)

                # When UR10e is at [0,0,0,0,0,0], xArm needs to be at these angles
                # So: UR10e_angle = xArm_visual_angle - kinematic_offset
                kinematic_offsets = {
                    1: 27.1,   # Gripper
                    2: -107.1,  # Wrist2  
                    3: 91.3,   # Wrist1
                    4: -3.0,   # Elbow
                    5: 86.0,   # Shoulder
                    6: 3.2,    # Base
                }
                
                # Apply kinematic offsets
                ur10e_reference_angles = []
                for i in range(6):
                    servo_id = i + 1
                    visual_angle = visual_angles[i]
                    kinematic_offset = kinematic_offsets[servo_id]
                    
                    # Check if the required xArm visual angle is within limits
                    min_limit, max_limit = XARM_JOINT_LIMITS[servo_id]
                    
                    if visual_angle < min_limit or visual_angle > max_limit:
                        print(f"WARNING: ID{servo_id} at {visual_angle:.1f}° is outside xArm limits [{min_limit}, {max_limit}]")
                        # Clamp the visual angle for calculation
                        visual_angle = self.clamp_to_xarm_limits(servo_id, visual_angle)
                        print(f"         Using clamped value: {visual_angle:.1f}°")
                    
                    # if servo_id == 3:
                    #     ur10e_angle = -(visual_angle - kinematic_offset)
                    # elif servo_id == 5:
                    #     ur10e_angle = -(visual_angle - kinematic_offset)
                    #else:
                    ur10e_angle = visual_angle - kinematic_offset
                    ur10e_reference_angles.append(ur10e_angle)
                    
                    print(f"  ID{servo_id}: {visual_angle:6.1f}° - kinematic {kinematic_offset:6.1f}° = {ur10e_angle:6.1f}°")
                
                print(f"[UR10e Reference Angles]: {[round(a, 1) for a in ur10e_reference_angles]}")
                
                # STEP 3: Map xArm order to UR10e order
                # xArm: [gripper(ID1), wrist2(ID2), wrist1(ID3), elbow(ID4), shoulder(ID5), base(ID6)]
                # UR10e: [base(J1), shoulder(J2), elbow(J3), wrist1(J4), wrist2(J5), wrist3(J6)]
                
                ur10e_ordered_angles = [
                    ur10e_reference_angles[5],  # xArm base → UR10e base (J1)
                    ur10e_reference_angles[4],  # xArm shoulder → UR10e shoulder (J2)
                    ur10e_reference_angles[3],  # xArm elbow → UR10e elbow (J3)
                    ur10e_reference_angles[2],  # xArm wrist1 → UR10e wrist1 (J4)
                    ur10e_reference_angles[1],  # xArm wrist2 → UR10e wrist2 (J5)
                    ur10e_reference_angles[0],  # xArm gripper → UR10e wrist3 (J6)
                ]
                
                print(f"[UR10e Ordered]: {[round(a, 1) for a in ur10e_ordered_angles]}")
    

                # STEP 4: Send to Isaac Sim
                message = {
                    'joints': ur10e_ordered_angles,
                    'timestamp': time.time(),
                    'note': 'With servo + kinematic offsets + joint limit clamping'
                }
                
                self.sim_sender.sendall(json.dumps(message).encode('utf-8'))
                
                print(f"\nFINAL SENT TO ISAAC SIM:")
                print(f"  xArm Raw: {[round(a, 1) for a in angles_deg]}")
                print(f"  UR10e Final: {[round(a, 1) for a in ur10e_ordered_angles]}")
                
                return True
                
            except Exception as e:
                print(f"Error sending angles: {e}")
                import traceback
                traceback.print_exc()
                return False
        return False

    def initialize_xarm_to_ur10e(self, arm, ur10e_pose_deg):
        print(f"\n=== INITIALIZE WITH ALL OFFSETS ===")
        print(f"UR10e target pose: {[round(a, 1) for a in ur10e_pose_deg]}")
        
        xarm_id_order = [6, 5, 4, 3, 2, 1]  # Base to gripper
        
        servo_list = []
        print("\nCalculating positions with ALL offsets and joint limits:")
        
        for i in range(6):
            xarm_id = xarm_id_order[i]
            ur10e_target_angle = ur10e_pose_deg[i]
            # REVERSE LOGIC: UR10e angle → xArm visual angle
            kinematic_offsets = {
                1: 27.1,   # Gripper
                2: -107.1,  # Wrist2  
                3: 91.3,   # Wrist1
                4: -3.0,   # Elbow
                5: 86.0,   # Shoulder
                6: 3.2,    # Base
            }
            
            # Step 1: Add kinematic offset
            visual_angle = ur10e_target_angle + kinematic_offsets[xarm_id]
            
            # Step 2: CLAMP to xArm joint limits
            visual_angle = self.clamp_to_xarm_limits(xarm_id, visual_angle)
            
            # Step 3: Check if we had to clamp
            original_angle = ur10e_target_angle + kinematic_offsets[xarm_id]
            if abs(original_angle - visual_angle) > 0.1:
                print(f"  WARNING: ID{xarm_id} would need {original_angle:.1f}° but clamped to {visual_angle:.1f}°")
                # Recalculate UR10e achievable angle
                achievable_ur10e_angle = visual_angle - kinematic_offsets[xarm_id]
                print(f"           UR10e achievable angle: {achievable_ur10e_angle:.1f}° (instead of {ur10e_target_angle:.1f}°)")
            
            # Step 4: Add servo offset (in degrees)
            servo_offset_degrees = Servo_offsets.get(xarm_id, 0) * 0.25
            target_with_servo_offset = visual_angle + servo_offset_degrees
            
            # Clamp again after servo offset
            target_with_servo_offset = self.clamp_to_xarm_limits(xarm_id, target_with_servo_offset)
            
            print(f"\n  ID{xarm_id}: UR10e target: {ur10e_target_angle:6.1f}°")
            print(f"    + kinematic {kinematic_offsets[xarm_id]:6.1f}° = {visual_angle:6.1f}°")
            print(f"    + servo {servo_offset_degrees:.2f}° = {target_with_servo_offset:6.1f}°")
            
            # Step 5: Convert to raw position
            base_position = int((target_with_servo_offset + 125.0) * 4)
            
            # Handle float offsets
            if isinstance(Servo_offsets.get(xarm_id, 0), float):
                position_float = float(base_position) + Servo_offsets.get(xarm_id, 0)
                position = int(round(position_float))
                print(f"    Raw: ({target_with_servo_offset} + 125) * 4 = {base_position}")
                print(f"    + raw offset {Servo_offsets.get(xarm_id, 0):.2f} = {position_float:.1f}")
                print(f"    Rounded to: {position}")
            else:
                position = base_position + Servo_offsets.get(xarm_id, 0)
                print(f"    Raw: ({target_with_servo_offset} + 125) * 4 = {base_position}")
                print(f"    + raw offset {Servo_offsets.get(xarm_id, 0)} = {position}")
            
            # Clamp to valid range
            position = max(0, min(1000, position))
            if not isinstance(position, int):
                position = int(round(position))
            
            servo_list.append([xarm_id, position])
        
        print(f"\nFinal positions with offsets (integers): {servo_list}")
        
        # Verify all positions are integers before moving
        print("\nVerifying all positions are integers:")
        for i, (servo_id, position) in enumerate(servo_list):
            if not isinstance(position, int):
                print(f"  WARNING: ID{servo_id} position {position} is type {type(position)}")
                # Force conversion
                servo_list[i][1] = int(round(float(position)))
                print(f"    Converted to: {servo_list[i][1]}")
        
        # Check current positions before moving
        print(f"\nBEFORE movement - current positions:")
        for xarm_id in [1, 2, 3, 4, 5, 6]:
            try:
                pos = arm.getPosition(xarm_id, False)
                angle_deg = (pos - 500) * 0.25
                print(f"  ID{xarm_id}: raw={pos}, angle={angle_deg:.1f}°")
            except Exception as e:
                print(f"  ID{xarm_id}: Error reading - {e}")
        
        # Move with calibrated positions
        print(f"\nMoving xArm with calibrated offsets...")
        try:
            arm.setPosition(servo_list, duration=2000, wait=True)
            time.sleep(0.5)
        except Exception as e:
            print(f"ERROR in setPosition: {e}")
            print("Trying alternative approach with float angles...")
            
            # Alternative: Convert to float angles
            servo_list_angles = []
            for servo_id, raw_position in servo_list:
                # Convert raw position to angle
                angle = (raw_position - 500) * 0.25
                servo_list_angles.append([servo_id, float(angle)])
            
            print(f"Trying with angles instead: {servo_list_angles}")
            arm.setPosition(servo_list_angles, duration=2000, wait=True)
            time.sleep(0.5)
        
        # Verify after movement
        print(f"\nAFTER movement - current positions:")
        for xarm_id in [1, 2, 3, 4, 5, 6]:
            try:
                pos = arm.getPosition(xarm_id, False)
                angle_deg = (pos - 500) * 0.25
                
                # Find target for this ID
                if xarm_id in [6, 5, 4, 3, 2, 1]:
                    idx = xarm_id_order.index(xarm_id)
                    target_angle = ur10e_pose_deg[idx]
                else:
                    target_angle = 0
                
                error = angle_deg - target_angle
                print(f"  ID{xarm_id}: {angle_deg:6.1f}° (target: {target_angle:5.1f}°, error: {error:+.1f}°)")
            except Exception as e:
                print(f"  ID{xarm_id}: Error reading - {e}")

        # Release servos
        print("\nReleasing servos (servoOff)...")
        try:
            order = [6, 5, 4, 3, 2, 1]
            arm.servoOff(order)
            time.sleep(0.5)
            print("XArm servos now released")
        except Exception as e:
            print(f"Error releasing servos: {e}")

        return True