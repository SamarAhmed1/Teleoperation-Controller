import math

class RobotCalibration:
    """
    Calibration functions for different robot arms.
    """
    
    ROBOT_SPECS = {
        'UR10e': {
            'joint_count': 6,
            'starting_position_deg': [0.0, -90.0, -90.0, -90.0, 90.0, 0.0],
            'joint_limits_deg': [
                (-360, 360),    # Base
                (-360, 360),    # Shoulder
                (-360, 360),    # Elbow
                (-360, 360),    # Wrist 1
                (-360, 360),    # Wrist 2
                (-360, 360)     # Wrist 3
            ],
        },
        'XArm': {
            'joint_count': 6,
            'starting_position_deg': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            'joint_limits_deg': [
                (-360, 360),    # Joint 1
                (-118, 120),    # Joint 2
                (-225, 11),     # Joint 3
                (-360, 360),    # Joint 4
                (-97, 180),     # Joint 5
                (-360, 360)     # Joint 6
            ],
        }
    }
    
    def __init__(self, source_robot='XArm', target_robot='UR10e', simulator_starting_pos=None):
        self.source_robot = source_robot
        self.target_robot = target_robot
        
        if source_robot not in self.ROBOT_SPECS:
            raise ValueError(f"Unknown source robot: {source_robot}")
        if target_robot not in self.ROBOT_SPECS:
            raise ValueError(f"Unknown target robot: {target_robot}")
            
        self.source_spec = self.ROBOT_SPECS[source_robot]
        self.target_spec = self.ROBOT_SPECS[target_robot]
        
        # Use simulator position if provided
        if simulator_starting_pos is not None:
            self.target_starting_pos_deg = simulator_starting_pos
            print(f"Using custom simulator position: {simulator_starting_pos}")
        else:
            self.target_starting_pos_deg = self.target_spec['starting_position_deg']
        
        # Calculate calibration offsets
        self.calibration_offsets = self._calculate_calibration_offsets()
    
    def _calculate_calibration_offsets(self):
        print("Calculating Offset")
        """Calculate offsets between source and target starting positions"""
        source_start = self.source_spec['starting_position_deg']
        target_start = self.target_starting_pos_deg
        
        if len(source_start) != len(target_start):
            raise ValueError("Source and target must have same number of joints")
        
        offsets = [t - s for s, t in zip(source_start, target_start)]
        print(offsets)
        return offsets
    
    def apply_calibration(self, angles_deg, reverse_order=False):
        """Apply calibration to convert angles from source to target robot"""
        if len(angles_deg) != self.source_spec['joint_count']:
            raise ValueError(f"Expected {self.source_spec['joint_count']} joints, got {len(angles_deg)}")
        
        # Reverse order if needed
        if reverse_order:
            angles_deg = angles_deg[::-1]
        
        # Apply calibration offsets
        calibrated_angles = [angle + offset for angle, offset in zip(angles_deg, self.calibration_offsets)]
        
        # Apply joint limits
        calibrated_angles = self._apply_joint_limits(calibrated_angles)
        
        return calibrated_angles
    
    # RENAMED METHOD: apply_calibration_with_custom_target -> calibrate_to_position
    def calibrate_to_position(self, angles_deg, target_position_deg, reverse_order=False):
        """
        Calibrate angles to a specific target position
        This replaces apply_calibration_with_custom_target
        """
        if len(angles_deg) != self.source_spec['joint_count']:
            raise ValueError(f"Expected {self.source_spec['joint_count']} joints, got {len(angles_deg)}")
        
        # Calculate temporary offsets for this target position
        source_start = self.source_spec['starting_position_deg']
        temp_offsets = [t - s for s, t in zip(source_start, target_position_deg)]
        
        # Reverse order if needed
        if reverse_order:
            angles_deg = angles_deg[::-1]
        
        # Apply temporary calibration
        calibrated_angles = [angle + offset for angle, offset in zip(angles_deg, temp_offsets)]
        
        # Apply joint limits
        calibrated_angles = self._apply_joint_limits(calibrated_angles)
        
        return calibrated_angles
    
    def _apply_joint_limits(self, angles_deg):
        """Ensure angles are within target robot's joint limits"""
        limits = self.target_spec['joint_limits_deg']
        clamped_angles = []
        
        for angle, (min_limit, max_limit) in zip(angles_deg, limits):
            # Normalize angle
            normalized_angle = ((angle + 180) % 360) - 180
            
            if min_limit <= max_limit:
                if normalized_angle < min_limit:
                    clamped = min_limit
                elif normalized_angle > max_limit:
                    clamped = max_limit
                else:
                    clamped = normalized_angle
            else:
                clamped = normalized_angle
            
            clamped_angles.append(clamped)
        
        return clamped_angles
    
    def degrees_to_radians(self, angles_deg):
        """Convert angles from degrees to radians"""
        return [math.radians(angle) for angle in angles_deg]
    
    def radians_to_degrees(self, angles_rad):
        """Convert angles from radians to degrees"""
        return [math.degrees(angle) for angle in angles_rad]
    
    def update_target_position(self, new_target_pos_deg):
        """Update target position and recalculate offsets"""
        self.target_starting_pos_deg = new_target_pos_deg
        self.calibration_offsets = self._calculate_calibration_offsets()
        print(f"Updated calibration with new target position: {new_target_pos_deg}")
    
    def get_calibration_info(self):
        """Get information about the current calibration setup"""
        return {
            'source_robot': self.source_robot,
            'target_robot': self.target_robot,
            'calibration_offsets_deg': self.calibration_offsets,
            'target_starting_position_deg': self.target_starting_pos_deg
        }