
class MagnetizationConfig:
    def __init__(self):
        self.JointToID = {1: "Gripper", 2: "Wrist2", 3: "Wrist1", 4: "Elbow", 5: "Shoulder", 6: "Base"}
        self.enabled_joints = [6,5,4,3,2,1]
        self.magnetization_start = 20.0
        self.max_resistance_factor = 3.0  #(1 = no effect, 5 = very strong)
        self.base_duration = 1000
        self.push_threshold = 2.0 #(degrees change)
        self.safety_buffer = 3.0 

        self.XARM_JOINT_LIMITS = {
            1: (-86.0, 31.5),   # Gripper
            2: (-125.0, 161.5), #wrist 2
            3: (-115.0, 130.8), # Wrist1
            4: (-122.0, 136.5), # Elbow
            5: (-95.2, 90),     # Shoulder
            6: (-125.0, 165.0), # Base
        }
        
        # Calculate soft limits (inside physical limits by safety buffer)
        self.soft_limits = {}
        for joint_id, (min_limit, max_limit) in self.XARM_JOINT_LIMITS.items():
            self.soft_limits[joint_id] = (
                min_limit + self.safety_buffer,
                max_limit - self.safety_buffer
            )
        
        # State tracking for each joint
        self.joint_states = {jid: "NORMAL" for jid in self.enabled_joints}
        
        # Previous commanded positions for push detection
        self.last_commanded = {jid: None for jid in self.enabled_joints}
        
        # Push event counters
        self.push_counts = {jid: 0 for jid in self.enabled_joints}

    def calculate_magnetization_strength(self, current_angle, min_limit, max_limit):
        """
        Strength calculation:
        0 = outside magnetization zone (no effect)
        0 to 1 = inside zone, increasing as you approach limit
        1 = at soft limit (maximum resistance)
        """
        dist_to_min = current_angle - min_limit
        dist_to_max = max_limit - current_angle
        
        # Find which limit is closer
        closest_distance = min(dist_to_min, dist_to_max)
        
        # If outside magnetization zone, no effect
        if closest_distance > self.magnetization_start:
            return 0.0
        
        # Calculate normalized strength (0 to 1)
        # Using inverse square: strength increases faster near limit
        normalized = 1.0 - (closest_distance / self.magnetization_start)
        strength = normalized ** 2  # Square for stronger effect near limit
        return min(1.0, max(0.0, strength))
    
    def calculate_resistance_position_asymmetric(self, current_angle, strength, min_limit, max_limit):
        """
        Asymmetric Resistance: Increases from min to max.
        Min Limit: 0% Resistance
        Center: 50% Resistance
        Max Limit: 100% Resistance
        """
        full_range = max_limit - min_limit
        distance_from_min = current_angle - min_limit
        
        if full_range == 0:
            return current_angle, 0.0
            
        normalized_position = distance_from_min / full_range
        resistance_strength = strength * normalized_position
        base_pushback = 5.0  # Max pushback at full resistance
        pushback_amount = base_pushback * resistance_strength
        resistance_position = current_angle - pushback_amount
        resistance_position = max(min_limit, resistance_position)
        
        return resistance_position, resistance_strength
        
    def calculate_resistance_position_simple(self, current_angle, strength, min_limit, max_limit):
        """
        Simple: Resistance only when moving toward MAX limit from center.
        No resistance when moving toward MIN limit.
        """
        center = (min_limit + max_limit) / 2
        
        # Only apply resistance when moving toward MAX (right of center)
        if current_angle <= center:
            return current_angle, 0.0  # No resistance toward min
        
        # Calculate how far from center toward max (0 to 1)
        distance_from_center = current_angle - center
        total_range_to_max = max_limit - center
        
        if total_range_to_max == 0:
            normalized = 0
        else:
            normalized = distance_from_center / total_range_to_max
        
        # Exponential resistance toward max
        resistance_strength = strength * (normalized ** 2)
        
        # Push back toward center
        pushback_amount = 6.0 * resistance_strength
        resistance_position = current_angle - pushback_amount
        
        # Clamp
        resistance_position = max(min_limit, resistance_position)
        
        return resistance_position, resistance_strength
    
    def detect_user_push(self, joint_id, current_angle, commanded_angle, prev_angle):
        """
        Detection logic:
        1. If magnetization is active (we're commanding resistance)
        2. And current angle differs significantly from commanded angle
        3. And the movement is toward the limit (not away from it)
        4. Then user is pushing against the resistance
        """
        if self.last_commanded[joint_id] is None:
            self.last_commanded[joint_id] = commanded_angle
            return False
        
        position_error = abs(current_angle - commanded_angle)
        movement = current_angle - prev_angle
        min_limit, max_limit = self.soft_limits[joint_id]

        moving_toward_limit = False
        if movement > 0 and current_angle > (min_limit + max_limit)/2:
            moving_toward_limit = True
        elif movement < 0 and current_angle < (min_limit + max_limit)/2:
            moving_toward_limit = True

        # Fixed: abs(movement > 0.5) -> abs(movement) > 0.5
        if (position_error > self.push_threshold and 
            moving_toward_limit and 
            abs(movement) > 0.5):
            
            self.push_counts[joint_id] += 1
            self.joint_states[joint_id] = "USER_PUSHING"
            return True
        
        return False

    def calculate_movement_duration(self, base_duration, strength):
        """Calculate movement duration based on magnetization strength."""
        if strength < 0.01:
            return base_duration
        
        scale_factor = 1.0 + (self.max_resistance_factor - 1.0) * (strength ** 1.5)
        return int(base_duration * scale_factor)

    def apply_magnetization_to_joint(self, joint_id, current_angle, prev_angle):
        """
        Main function to apply magnetization effect to a single joint.
        Returns: (should_command, target_angle, duration, strength, state)
        """
        # Skip if joint not enabled
        if joint_id not in self.enabled_joints:
            return False, current_angle, self.base_duration, 0.0, "DISABLED"
        
        # Get limits for this joint
        min_limit, max_limit = self.soft_limits[joint_id]
        
        # Calculate magnetization strength
        strength = self.calculate_magnetization_strength(current_angle, min_limit, max_limit)
        
        # Update state based on strength
        if strength < 0.01:
            state = "NORMAL"
        elif strength < 0.5:
            state = "APPROACHING_LIMIT"
        else:
            state = "NEAR_LIMIT"
        
        self.joint_states[joint_id] = state
        
        # If no magnetization needed, just return current position
        if strength < 0.01:
            self.last_commanded[joint_id] = current_angle
            return False, current_angle, self.base_duration, strength, state
        
        # Calculate resistance position and duration
        resistance_position, resistance_strength = self.calculate_resistance_position_simple(
            current_angle, strength, min_limit, max_limit
        )
        
        duration = self.calculate_movement_duration(self.base_duration, strength)
        
        # Check for user push
        user_pushing = self.detect_user_push(
            joint_id, current_angle, resistance_position, prev_angle
        )
        
        if user_pushing:
            state = "USER_PUSHING"
        
        self.last_commanded[joint_id] = resistance_position
        
        return True, resistance_position, duration, strength, state

    def display_joint_info(self, joint_id, current_angle, strength, state, push_count):
        """
        Display formatted joint information with magnetization status.
        """
        joint_name = self.JointToID.get(joint_id, f"Joint_{joint_id}")
        
        # Color coding for states (ANSI escape codes)
        colors = {
            "NORMAL": "\033[32m",      # Green
            "APPROACHING_LIMIT": "\033[33m",  # Yellow
            "NEAR_LIMIT": "\033[91m",  # Light Red
            "USER_PUSHING": "\033[31m",  # Red
            "DISABLED": "\033[90m"     # Gray
        }
        
        color = colors.get(state, "\033[0m")
        reset = "\033[0m"
        
        # Magnetization indicator
        if strength > 0:
            mag_indicator = "█" * int(strength * 10) + "░" * (10 - int(strength * 10))
            mag_text = f" Mag[{mag_indicator}]"
        else:
            mag_text = " " * 15
        
        # State abbreviation
        state_abbr = {
            "NORMAL": "NORM",
            "APPROACHING_LIMIT": "APPR",
            "NEAR_LIMIT": "NEAR",
            "USER_PUSHING": "PUSH",
            "DISABLED": "OFF"
        }.get(state, state[:4])
        
        print(f"{color}{joint_name:<10}: {current_angle:6.1f}° [{state_abbr}]{mag_text} ")
