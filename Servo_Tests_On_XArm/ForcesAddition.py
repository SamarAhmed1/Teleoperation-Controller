class MagnetizationConfig:
    def __init__(self, debug_joint=None):
        self.JointToID = {1: "Gripper", 2: "Wrist2", 3: "Wrist1", 4: "Elbow", 5: "Shoulder", 6: "Base"}
        self.enabled_joints = [6, 5, 4, 3, 2, 1]
        self.magnetization_start = 20.0
        self.max_resistance_factor = 3.0
        self.base_duration = 500
        self.push_threshold = 2.0
        self.safety_buffer = 3.0
        self.debug_joint = debug_joint  # Set to 6 for Base joint debugging

        
        self.XARM_JOINT_LIMITS = {
            1: (-86.0, 31.5),   # Gripper
            2: (-125.0, 161.5), #wrist 2
            3: (-115.0, 130.8), # Wrist1
            4: (-122.0, 136.5), # Elbow
            5: (-95.2, 90),     # Shoulder
            6: (-125.0, 165.0), # Base
        }
        zones = [
            (0, 40, 0.0),    # 0-20° from center: 0% resistance
            (40.01, 80, 0.2),   # 20-40°: 20% resistance  
            (80.01, 100, 0.5),   # 40-60°: 50% resistance
            (100.01, 120, 0.8),   # 60-80°: 80% resistance
            (120.01, float('inf'), 1.0)  # 80°+: 100% resistance
        ]

        # Timing control
        self.last_command_time = {jid: 0 for jid in self.enabled_joints}
        self.min_command_interval = 0.2  # 200ms between commands
        self.current_zones = {jid: -1 for jid in self.enabled_joints}  #
        self.joint_centers = {}

        for joint_id, (min_limit, max_limit) in self.XARM_JOINT_LIMITS.items():
            self.joint_centers[joint_id] = (min_limit+max_limit)/2
    
        self.soft_limits = {}
        for joint_id, (min_limit, max_limit) in self.XARM_JOINT_LIMITS.items():
            self.soft_limits[joint_id] = (
                min_limit + self.safety_buffer,
                max_limit - self.safety_buffer
            )
        
        self.soft_centers = {}
        for joint_id, (min_limit, max_limit) in self.soft_limits.items():
            self.soft_centers[joint_id] = (min_limit + max_limit)/2

        
        self.joint_states = {jid: "NORMAL" for jid in self.enabled_joints}
        self.last_commanded = {jid: None for jid in self.enabled_joints}
        self.push_counts = {jid: 0 for jid in self.enabled_joints}

    def get_zone_for_distance(self, distance_from_center):
        """Find which zone contains this distance from center."""
        for zone_min, zone_max, strength, pushback in self.resistance_zones:
            if zone_min <= distance_from_center < zone_max:
                return zone_min, zone_max, strength, pushback
        # Fallback: return last zone
        return self.resistance_zones[-1]

    def calculate_magnetization_strength_exponential(self, current_angle, min_limit, max_limit, joint_id=None):
        center = self.soft_centers.get(joint_id, (min_limit + max_limit)/2)
        
        if current_angle <= center:
            max_possible_distance = center - min_limit
            if max_possible_distance > 0:
                distance_from_center = center - current_angle
                normalized = distance_from_center / max_possible_distance
                strength = normalized ** 2
            else:
                strength = 0.0
        else:
            max_possible_distance = max_limit - center
            if max_possible_distance > 0:
                distance_from_center = current_angle - center
                normalized = distance_from_center / max_possible_distance
                strength = normalized ** 2
            else:
                strength = 0.0
        
        return max(0.0, min(1.0, strength))
    
    def calculate_magnetization_strength_linear(self, current_angle, min_limit, max_limit, joint_id=None):
        """Linear strength: 0 at center, 1 at limits (your current function)"""
        center = self.soft_centers.get(joint_id, (min_limit + max_limit)/2)
        
        if current_angle <= center:
            max_possible_distance = center - min_limit
            if max_possible_distance > 0:
                distance_from_center = center - current_angle
                strength = distance_from_center / max_possible_distance
            else:
                strength = 0.0
        else:
            max_possible_distance = max_limit - center
            if max_possible_distance > 0:
                distance_from_center = current_angle - center
                strength = distance_from_center / max_possible_distance
            else:
                strength = 0.0
        
        return max(0.0, min(1.0, strength))

    def calculate_magnetization_strength_constant(self, current_angle, min_limit, max_limit, joint_id=None):
        """Constant strength within a zone"""
        # Always return a fixed strength value
        CONSTANT_STRENGTH = 0.2  # 20% constant strength
        return CONSTANT_STRENGTH
    
    def calculate_magnetization_strength(self, current_angle, min_limit, max_limit, joint_id=None):
        """
        ZONE-BASED strength:
        - Constant near center (-20° to 20°)
        - Linear in mid-range (-60° to -20° and 20° to 60°)
        - Exponential near limits (beyond ±60°)
        """
        center = self.soft_centers.get(joint_id, (min_limit + max_limit)/2)
        
        # Calculate distance from center (absolute value)
        distance_from_center = abs(current_angle - center)
        
        # Define zone boundaries (in degrees from center)
        CONSTANT_ZONE = 20.0    # -20° to 20° from center
        LINEAR_ZONE = 60.0      # -60° to -20° and 20° to 60°
        # Beyond 60° = exponential zone
        
        if distance_from_center <= CONSTANT_ZONE:
            # ZONE 1: Constant strength near center
            strength = self.calculate_magnetization_strength_constant(
                current_angle, min_limit, max_limit, joint_id
            )
            zone_name = "CONSTANT"
            
        elif distance_from_center <= LINEAR_ZONE:
            # ZONE 2: Linear strength in mid-range
            strength = self.calculate_magnetization_strength_linear(
                current_angle, min_limit, max_limit, joint_id
            )
            zone_name = "LINEAR"
            
        else:
            # ZONE 3: Exponential strength near limits
            strength = self.calculate_magnetization_strength_exponential(
                current_angle, min_limit, max_limit, joint_id
            )
            zone_name = "EXPONENTIAL"
        
        # DEBUG PRINT
        if self.debug_joint == joint_id:
            print(f"\n[DEBUG calculate_magnetization_strength] Joint {joint_id} ({self.JointToID[joint_id]}):")
            print(f"  current_angle: {current_angle:.1f}°, center: {center:.1f}°")
            print(f"  distance_from_center: {distance_from_center:.1f}°")
            print(f"  ZONE: {zone_name} ({CONSTANT_ZONE}°|{LINEAR_ZONE}° zones)")
            print(f"  strength: {strength:.3f}")
            print(f"  Zones:")
            print(f"    - Constant: ±{CONSTANT_ZONE}° from center")
            print(f"    - Linear: ±{LINEAR_ZONE}° from center")
            print(f"    - Exponential: beyond ±{LINEAR_ZONE}°")
        
        return max(0.0, min(1.0, strength))

    def calculate_resistance_position_simple(self, current_angle, strength, min_limit, max_limit, joint_id=None):
        """
        SIMPLIFIED: Push back proportional to strength.
        More strength = more pushback away from max.
        """
        center = self.soft_centers.get(joint_id, (min_limit + max_limit)/2)

        # Base pushback amount at full strength
        #base_pushback = 14.0 #INCREASED from 6.0 to 15.0

        # Calculate distance from center
        distance_from_center = abs(current_angle - center)
        
        # Different base pushback per zone
        CONSTANT_ZONE = 20.0
        LINEAR_ZONE = 60.0
        
        if distance_from_center <= CONSTANT_ZONE:
            # Zone 1: Gentle pushback
            base_pushback = 2.0
            zone_name = "CONSTANT"
        elif distance_from_center <= LINEAR_ZONE:
            # Zone 2: Moderate pushback
            base_pushback = 6.0
            zone_name = "LINEAR"
        else:
            # Zone 3: Strong pushback
            base_pushback = 10.0
            zone_name = "EXPONENTIAL"
            #REDUCE pushback near limits (tapering)
        phys_min, phys_max = self.XARM_JOINT_LIMITS[joint_id]
        limit_buffer = 10.0  # Degrees
        if current_angle > center:
            # Right side - check proximity to max
            distance_to_max = phys_max - current_angle
            limit_factor = min(1.0, max(0.0, distance_to_max / limit_buffer))
        else:
            # Left side - check proximity to min
            distance_to_min = current_angle - phys_min
            limit_factor = min(1.0, max(0.0, distance_to_min / limit_buffer))
        
        # Calculate pushback (more when strength is high)
        pushback_amount = base_pushback * strength * limit_factor

        if current_angle > center:
            resistance_position = current_angle - pushback_amount #Push left, position is right of center
            direction = "Left Toward center"

        else:
            resistance_position = current_angle + pushback_amount #Push right, position is Left of center
            direction = "Right Toward Center"

        resistance_position = max(min_limit, min(max_limit, resistance_position))

        # DEBUG PRINT
        if self.debug_joint == joint_id:
            print(f"\n[DEBUG calculate_resistance_position_simple] Joint {joint_id} ({self.JointToID[joint_id]}):")
            print(f"  current_angle: {current_angle:.1f}°, strength: {strength:.3f}")
            print(f"  center: {center:.1f}°")
            print(f"  base_pushback: {base_pushback}°, pushback_amount: {pushback_amount:.2f}°")
            print(f"  direction: {direction}")
            print(f"  resistance_position: {resistance_position:.1f}°")
            print(f"  Final: position={resistance_position:.1f}°, strength={strength:.3f}")
        
        return resistance_position, strength
    
    def apply_magnetization_to_joint(self, joint_id, current_angle, prev_angle):
        """
        Main function to apply magnetization effect to a single joint.
        Returns: (should_command, target_angle, duration, strength, state)
        """
        # DEBUG PRINT
        if self.debug_joint == joint_id:
            print(f"\n{'='*60}")
            print(f"[DEBUG apply_magnetization_to_joint] START for Joint {joint_id} ({self.JointToID[joint_id]})")
            print(f"  current_angle: {current_angle:.1f}°, prev_angle: {prev_angle:.1f}°")
        
        # Skip if joint not enabled
        if joint_id not in self.enabled_joints:
            if self.debug_joint == joint_id:
                print(f"  → Joint not enabled, returning DISABLED")
            return False, current_angle, self.base_duration, 0.0, "DISABLED"
        
        # Get limits for this joint
        min_limit, max_limit = self.soft_limits[joint_id]
        
        if self.debug_joint == joint_id:
            print(f"  soft_limits: min={min_limit:.1f}, max={max_limit:.1f}")
        
        # Calculate magnetization strength
        strength = self.calculate_magnetization_strength(current_angle, min_limit, max_limit, joint_id)
        
        # Check if stuck at a physical limit
        is_stuck, stuck_limit = self.is_stuck_at_limit(joint_id, current_angle, strength, min_limit, max_limit)
        
        # Calculate movement direction
        movement = current_angle - prev_angle
        center = self.soft_centers[joint_id]
        
        # If stuck at limit AND user is trying to escape (moving toward center)
        if is_stuck:
            if self.debug_joint == joint_id:
                print(f"  → STUCK at {stuck_limit} limit! strength={strength:.3f}")
            
            # Check if user is trying to escape (moving toward center)
            if (stuck_limit == "MAX" and movement < -1.0) or (stuck_limit == "MIN" and movement > 1.0):     # Stuck at min, moving right toward center # Stuck at max, moving left toward center
                
                if self.debug_joint == joint_id:
                    print(f"  → USER TRYING TO ESCAPE from {stuck_limit} limit! movement={movement:.1f}")
                
                # Return special flag to use servoOff()
                return False, current_angle, 0, 0.0, "ESCAPING"
        
        # Update state based on strength
        if strength < 0.3:
            state = "NORMAL"
        elif strength > 0.3 and strength < 0.7:
            state = "MID_RANGE"
        else:
            state = "NEAR_LIMIT"
        
        self.joint_states[joint_id] = state
        
        if self.debug_joint == joint_id:
            print(f"  Calculated strength: {strength:.3f} → state: {state}")
        
        # If no magnetization needed, just return current position
        if strength < 0.01:
            self.last_commanded[joint_id] = current_angle
            if self.debug_joint == joint_id:
                print(f"  → strength < 0.01, NO MAGNETIZATION, returning current position")
            return False, current_angle, self.base_duration, strength, state
        
        # Calculate resistance position and duration
        resistance_position, resistance_strength = self.calculate_resistance_position_simple(
            current_angle, strength, min_limit, max_limit, joint_id
        )
        
        duration = self.calculate_movement_duration(self.base_duration, strength)
        
        # Check for user push
        user_pushing = self.detect_user_push(
            joint_id, current_angle, resistance_position, prev_angle
        )
        
        if user_pushing:
            state = "USER_PUSHING"
            if self.debug_joint == joint_id:
                print(f"  → USER PUSH DETECTED! State changed to USER_PUSHING")
        
        self.last_commanded[joint_id] = resistance_position
        
        # Determine if we should command
        should_command = strength >= 0.01 and resistance_strength > 0
        
        if self.debug_joint == joint_id:
            print(f"  Final decision:")
            print(f"    should_command: {should_command} (strength={strength:.3f}>=0.01 and resistance_strength={resistance_strength:.3f}>0)")
            print(f"    target_angle: {resistance_position:.1f}°")
            print(f"    duration: {duration}ms")
            print(f"    strength: {strength:.3f}")
            print(f"    state: {state}")
            print(f"{'='*60}")
        
        return True, resistance_position, duration, strength, state

    def detect_user_push(self, joint_id, current_angle, commanded_angle, prev_angle):
        if self.last_commanded[joint_id] is None:
            self.last_commanded[joint_id] = commanded_angle
            return False
        
        position_error = abs(current_angle - commanded_angle)
        movement = current_angle - prev_angle
        
        # Quick return if not enough movement
        if abs(movement) <= 0.5:
            return False
        
        # Quick return if not enough position error
        if position_error <= self.push_threshold:
            return False
        
        center = self.soft_centers[joint_id]
        
        # Check if moving AWAY from center (user pushing against our resistance)
        # We're always pushing TOWARD center, so user push = moving AWAY from center
        if movement > 0 and current_angle > center:
            # Moving right while on right side = away from center = user push
            self.push_counts[joint_id] += 1
            self.joint_states[joint_id] = "USER_PUSHING"
            return True
        elif movement < 0 and current_angle < center:
            # Moving left while on left side = away from center = user push
            self.push_counts[joint_id] += 1
            self.joint_states[joint_id] = "USER_PUSHING"
            return True
        
        return False

    def calculate_movement_duration(self, base_duration, strength):
        if strength < 0.01:
            return base_duration

        #Theory: Higher strength = shorter duration
        min_duration = 100
        max_duration = 500

        duration = max_duration - (strength * (max_duration - min_duration))
        return int(duration)

    def display_joint_info(self, joint_id, current_angle, strength, state, push_count):
        """Same as before..."""
        joint_name = self.JointToID.get(joint_id, f"Joint_{joint_id}")
        
        colors = {
            "NORMAL": "\033[32m",
            "MID_RANGE": "\033[33m",
            "NEAR_LIMIT": "\033[91m",
            "USER_PUSHING": "\033[31m",
            "DISABLED": "\033[90m"
        }
        
        color = colors.get(state, "\033[0m")
        reset = "\033[0m"
        
        if strength > 0:
            mag_indicator = "█" * int(strength * 10) + "░" * (10 - int(strength * 10))
            mag_text = f" Mag[{mag_indicator}]"
        else:
            mag_text = " " * 15
        
        state_abbr = {
            "NORMAL": "NORM",
            "MID_RANGE": "APPR",
            "NEAR_LIMIT": "NEAR",
            "USER_PUSHING": "PUSH",
            "DISABLED": "OFF"
        }.get(state, state[:4])
        
        print(f"{color}{joint_name:<10}: {current_angle:6.1f}° [{state_abbr}]{mag_text} ")

    def is_stuck_at_limit(self, joint_id, current_angle, strength, min_limit, max_limit):
        """
        Check if we're stuck at a physical limit.
        Returns: (is_stuck, which_limit)
        """
        PHYSICAL_BUFFER = 2.0  # Degrees from physical limit
        
        phys_min, phys_max = self.XARM_JOINT_LIMITS[joint_id]
        
        # Check if near physical max
        if abs(current_angle - phys_max) < PHYSICAL_BUFFER and strength > 0.7:
            return True, "MAX"
        
        # Check if near physical min  
        if abs(current_angle - phys_min) < PHYSICAL_BUFFER and strength > 0.7:
            return True, "MIN"
        
        return False, None