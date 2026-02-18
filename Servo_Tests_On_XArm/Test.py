import time

class MagnetizationConfig:
    def __init__(self, debug_joint=None, debug_strength_interval=5):
        """
        debug_strength_interval: Print strength every X degrees (default: 5)
        """
        self.JointToID = {1: "Gripper", 2: "Wrist2", 3: "Wrist1", 4: "Elbow", 5: "Shoulder", 6: "Base"}
        self.enabled_joints = [6, 5, 4, 3, 2, 1]
        self.debug_joint = debug_joint
        self.debug_strength_interval = debug_strength_interval
        self.last_strength_debug = {jid: -999 for jid in self.enabled_joints}  # Track last debug print
        
        # CRITICAL: Tuned parameters for smooth, escapable resistance
        self.base_duration = 300  # Faster response (was 500)
        self.min_command_interval = 0.05  # Much faster updates (was 0.2)
        self.safety_buffer = 3.0
        
        # Resistance tuning - THESE ARE KEY
        self.max_pushback_degrees = 3.0  # Maximum pushback distance (reduced from 4.0 for gentler resistance)
        self.min_strength_threshold = 0.05  # Start resistance earlier
        self.escape_strength_reduction = 0.4  # Reduce strength by 60% when escaping
        
        # CRITICAL: Overall strength multiplier to reduce resistance globally
        self.global_strength_multiplier = 0.55  # Reduce all strength by 45% (adjustable: 0.4-0.7)
        
        # Physical limits
        self.XARM_JOINT_LIMITS = {
            1: (-86.0, 31.5),   # Gripper
            2: (-125.0, 161.5), # Wrist 2
            3: (-115.0, 130.8), # Wrist1
            4: (-122.0, 136.5), # Elbow
            5: (-95.2, 90),     # Shoulder
            6: (-125.0, 165.0), # Base
        }
        
        # Calculate soft limits and centers
        self.soft_limits = {}
        self.soft_centers = {}
        self.joint_centers = {}
        
        for joint_id, (min_limit, max_limit) in self.XARM_JOINT_LIMITS.items():
            self.soft_limits[joint_id] = (
                min_limit + self.safety_buffer,
                max_limit - self.safety_buffer
            )
            soft_min, soft_max = self.soft_limits[joint_id]
            self.soft_centers[joint_id] = (soft_min + soft_max) / 2
            self.joint_centers[joint_id] = (min_limit + max_limit) / 2
        
        # State tracking
        self.joint_states = {jid: "NORMAL" for jid in self.enabled_joints}
        self.last_commanded = {jid: None for jid in self.enabled_joints}
        self.last_command_time = {jid: 0 for jid in self.enabled_joints}
        self.push_counts = {jid: 0 for jid in self.enabled_joints}
        self.escape_mode = {jid: False for jid in self.enabled_joints}
        self.escape_start_time = {jid: 0 for jid in self.enabled_joints}
        
        # Movement detection for jitter prevention
        self.last_angle = {jid: None for jid in self.enabled_joints}
        self.movement_velocity = {jid: 0.0 for jid in self.enabled_joints}
        self.stationary_time = {jid: 0 for jid in self.enabled_joints}
        self.settling_threshold = 0.3  # degrees/update for "stopped" detection (reduced from 0.5)
        self.settling_time = 0.6  # seconds to wait before commanding after stop (increased from 0.3)
        
        # Strength debug tracking
        self.strength_history = {jid: [] for jid in self.enabled_joints}
        self.max_history_length = 100

    def debug_strength_by_degree(self, joint_id, current_angle, strength):
        """
        Print strength at regular degree intervals
        """
        if self.debug_joint != joint_id:
            return
            
        # Round to nearest debug interval
        rounded_angle = round(current_angle / self.debug_strength_interval) * self.debug_strength_interval
        
        # Check if we should print (if angle has changed by at least the interval)
        if abs(rounded_angle - self.last_strength_debug[joint_id]) >= self.debug_strength_interval:
            min_limit, max_limit = self.soft_limits[joint_id]
            center = self.soft_centers[joint_id]
            distance_from_center = current_angle - center
            
            # Visual indicator
            bar_length = 40
            normalized_pos = (current_angle - min_limit) / (max_limit - min_limit)
            bar_pos = int(normalized_pos * bar_length)
            
            bar = ""
            for i in range(bar_length):
                if i == bar_pos:
                    bar += "▊"
                elif i == int((center - min_limit) / (max_limit - min_limit) * bar_length):
                    bar += "│"
                else:
                    bar += "░"
            
            print(f"\n[STRENGTH @ {current_angle:5.1f}°]")
            print(f"  Joint {joint_id} ({self.JointToID[joint_id]}): Strength = {strength:.3f}")
            print(f"  Distance from center: {distance_from_center:5.1f}°")
            print(f"  [{bar}]")
            print(f"  Limits: [{min_limit:5.1f}°, {max_limit:5.1f}°], Center: {center:5.1f}°")
            
            # Store for history
            self.strength_history[joint_id].append((time.time(), current_angle, strength))
            if len(self.strength_history[joint_id]) > self.max_history_length:
                self.strength_history[joint_id].pop(0)
            
            self.last_strength_debug[joint_id] = rounded_angle

    def print_strength_profile(self, joint_id):
        """
        Print the complete strength profile for a joint
        """
        if joint_id not in self.soft_limits:
            print(f"Joint {joint_id} not found!")
            return
            
        min_limit, max_limit = self.soft_limits[joint_id]
        center = self.soft_centers[joint_id]
        
        print(f"\n{'='*60}")
        print(f"STRENGTH PROFILE - Joint {joint_id} ({self.JointToID[joint_id]})")
        print(f"Soft limits: [{min_limit:.1f}°, {max_limit:.1f}°]")
        print(f"Center: {center:.1f}°")
        print(f"Range: {max_limit - min_limit:.1f}°")
        print(f"Global strength multiplier: {self.global_strength_multiplier:.2f}")
        print(f"{'='*60}")
        
        # Calculate strength at key points
        test_points = [
            min_limit,
            min_limit + (center - min_limit) * 0.25,
            min_limit + (center - min_limit) * 0.5,
            center,
            center + (max_limit - center) * 0.5,
            center + (max_limit - center) * 0.75,
            max_limit
        ]
        
        for angle in test_points:
            strength = self.calculate_magnetization_strength_cubic(angle, min_limit, max_limit, joint_id)
            pushback = self.max_pushback_degrees * strength
            state = "NORMAL" if strength < 0.2 else "MID" if strength < 0.6 else "HIGH"
            
            print(f"  {angle:6.1f}°: {strength:.3f} strength | {pushback:4.1f}° pushback | {state}")
        
        print(f"{'='*60}")

    def is_joint_moving(self, joint_id, current_angle):
        """
        Detect if joint is actively moving or has settled.
        Returns: (is_moving, velocity)
        """
        current_time = time.time()
        
        # Initialize if first reading
        if self.last_angle.get(joint_id) is None:
            self.last_angle[joint_id] = current_angle
            self.stationary_time[joint_id] = current_time
            return False, 0.0
        
        # Calculate velocity (degrees per update)
        velocity = abs(current_angle - self.last_angle[joint_id])
        self.movement_velocity[joint_id] = velocity
        self.last_angle[joint_id] = current_angle
        
        # Check if moving significantly
        if velocity > self.settling_threshold:
            # Joint is moving
            self.stationary_time[joint_id] = current_time
            return True, velocity
        else:
            # Joint appears stationary
            time_stationary = current_time - self.stationary_time[joint_id]
            
            # Only consider truly stopped after settling time
            if time_stationary < self.settling_time:
                # Still in settling period - treat as moving
                return True, velocity
            else:
                # Fully settled
                return False, velocity

    def calculate_magnetization_strength_cubic(self, current_angle, min_limit, max_limit, joint_id=None):
        """
        CUBIC resistance curve: smooth and progressive
        - Gentle near center
        - Moderate in mid-range (WITH DAMPING)
        - Strong but not overwhelming near limits
        - Always escapable
        - WITH GLOBAL STRENGTH REDUCTION
        """
        center = self.soft_centers.get(joint_id, (min_limit + max_limit) / 2)
        
        # Calculate normalized distance from center (0 to 1)
        if current_angle <= center:
            max_distance = center - min_limit
            if max_distance > 0:
                distance_from_center = center - current_angle
                normalized = distance_from_center / max_distance
            else:
                normalized = 0.0
        else:
            max_distance = max_limit - center
            if max_distance > 0:
                distance_from_center = current_angle - center
                normalized = distance_from_center / max_distance
            else:
                normalized = 0.0
        
        # Cubic curve: x³ gives smooth acceleration
        strength = normalized ** 3
        
        # DAMPING for mid-range (reduce strength between 20-60%)
        original_strength = strength  # Store for debug
        if 0.2 <= strength <= 0.6:
            # Apply damping factor to reduce mid-range resistance
            damping_factor = 0.6  # Reduce to 60% of original strength
            strength = strength * damping_factor
        
        # CRITICAL: Apply global strength multiplier to reduce all resistance
        strength = strength * self.global_strength_multiplier
        
        # If in escape mode, reduce strength significantly
        if self.escape_mode.get(joint_id, False):
            strength *= self.escape_strength_reduction
        
        # Call the degree-based debugger
        if joint_id is not None:
            self.debug_strength_by_degree(joint_id, current_angle, strength)
        
        # Enhanced debug output for the debug joint
        if self.debug_joint == joint_id:
            print(f"\n[DEBUG Cubic Strength] Joint {joint_id}:")
            print(f"  Angle: {current_angle:.1f}°, Center: {center:.1f}°")
            print(f"  Limits: [{min_limit:.1f}°, {max_limit:.1f}°]")
            print(f"  Distance from center: {abs(current_angle - center):.1f}°")
            print(f"  Normalized: {normalized:.3f}")
            print(f"  Raw strength (cubic): {normalized**3:.3f}")
            
            if 0.2 <= original_strength <= 0.6:
                print(f"  After mid-range damping (60%): {original_strength * 0.6:.3f}")
            
            print(f"  After global multiplier ({self.global_strength_multiplier}): {strength:.3f}")
            
            # Visual strength indicator
            stars = "★" * int(strength * 20) + "☆" * (20 - int(strength * 20))
            print(f"  Strength meter: [{stars}] {strength:.1%}")
            
            if self.escape_mode.get(joint_id, False):
                print(f"  ESCAPE MODE ACTIVE - strength reduced by {(1-self.escape_strength_reduction)*100:.0f}%")
            
            # Calculate pushback for this strength
            pushback = self.max_pushback_degrees * strength
            print(f"  Current pushback: {pushback:.2f}° (max: {self.max_pushback_degrees}°)")
        
        return max(0.0, min(1.0, strength))

    def calculate_resistance_position_adaptive(self, current_angle, strength, min_limit, max_limit, joint_id=None):
        """
        Adaptive pushback that scales with strength
        - Uses cubic strength curve
        - RESISTS movement away from center (not pushes toward it)
        - Only applies resistance when moving outward
        - Scales smoothly with distance
        """
        center = self.soft_centers.get(joint_id, (min_limit + max_limit) / 2)
        
        # Calculate pushback amount (scales with strength)
        pushback_amount = self.max_pushback_degrees * strength
        
        # CRITICAL FIX: Resistance should OPPOSE movement, not assist return to center
        # When far from center, we only apply resistance to prevent moving FURTHER away
        # We DON'T resist movement back toward center
        
        if current_angle > center:
            # Right of center: resist movement to the right (push left toward center)
            resistance_position = current_angle - pushback_amount
            direction = "← resisting outward movement"
        else:
            # Left of center: resist movement to the left (push right toward center)
            resistance_position = current_angle + pushback_amount
            direction = "→ resisting outward movement"
        
        # Clamp to limits
        resistance_position = max(min_limit, min(max_limit, resistance_position))
        
        if self.debug_joint == joint_id:
            print(f"\n[DEBUG Resistance Position]:")
            print(f"  Current: {current_angle:.1f}°, Center: {center:.1f}°")
            print(f"  Strength: {strength:.3f}")
            print(f"  Pushback: {pushback_amount:.2f}° {direction}")
            print(f"  Target: {resistance_position:.1f}°")
            
            # Visual movement indicator
            if pushback_amount > 0:
                arrow_count = min(int(pushback_amount), 20)
                arrow = ">" if direction == "→ toward center" else "<"
                print(f"  Movement: {' ' * 10}{arrow * arrow_count}")
        
        return resistance_position, strength

    def detect_escape_attempt(self, joint_id, current_angle, prev_angle, strength):
        """
        Detect if user is trying to escape from high resistance
        Returns: True if escaping, False otherwise
        """
        if strength < 0.5:  # Only relevant at moderate-high resistance
            if self.escape_mode.get(joint_id, False):
                # Exit escape mode when back to low resistance
                self.escape_mode[joint_id] = False
            return False
        
        center = self.soft_centers[joint_id]
        movement = current_angle - prev_angle
        
        # Check if movement is significant
        if abs(movement) < 1.0:
            return False
        
        # Determine if moving toward center (escape attempt)
        moving_toward_center = False
        if current_angle > center and movement < -0.5:  # Right of center, moving left
            moving_toward_center = True
        elif current_angle < center and movement > 0.5:  # Left of center, moving right
            moving_toward_center = True
        
        if moving_toward_center and strength > 0.5:
            # User is pushing back against resistance!
            if not self.escape_mode.get(joint_id, False):
                self.escape_mode[joint_id] = True
                self.escape_start_time[joint_id] = time.time()
                if self.debug_joint == joint_id:
                    print(f"\n{'!'*60}")
                    print(f"[ESCAPE MODE ACTIVATED] User pushing toward center!")
                    print(f"  Strength reduced by {(1-self.escape_strength_reduction)*100:.0f}%")
                    print(f"{'!'*60}")
            return True
        
        # Auto-exit escape mode after 2 seconds
        if self.escape_mode.get(joint_id, False):
            if time.time() - self.escape_start_time[joint_id] > 2.0:
                self.escape_mode[joint_id] = False
                if self.debug_joint == joint_id:
                    print(f"\n[ESCAPE MODE DEACTIVATED] Timeout")
        
        return False

    def apply_magnetization_to_joint(self, joint_id, current_angle, prev_angle):
        """
        Main function to apply magnetization effect to a single joint.
        Returns: (should_command, target_angle, duration, strength, state)
        """
        if self.debug_joint == joint_id:
            print(f"\n{'='*60}")
            print(f"[START] Joint {joint_id} ({self.JointToID[joint_id]})")
            print(f"  Current: {current_angle:.1f}°, Previous: {prev_angle:.1f}°, Δ: {current_angle-prev_angle:.1f}°")
        
        # Skip if joint not enabled
        if joint_id not in self.enabled_joints:
            return False, current_angle, self.base_duration, 0.0, "DISABLED"
        
        # Check if joint is actively moving (prevents jitter from command queue buildup)
        is_moving, velocity = self.is_joint_moving(joint_id, current_angle)
        
        if self.debug_joint == joint_id:
            print(f"  Movement: velocity={velocity:.2f}°/update, moving={is_moving}")
        
        # CRITICAL: Don't send commands while actively moving fast
        # This prevents command queue buildup that causes jitter when you stop
        if is_moving and velocity > 1.0:  # Fast movement threshold (reduced from 2.0)
            if self.debug_joint == joint_id:
                print(f"  → FAST MOVEMENT DETECTED - Skipping command to prevent jitter")
            # Return current position but don't command
            return False, current_angle, self.base_duration, 0.0, "MOVING"
        
        # Get limits
        min_limit, max_limit = self.soft_limits[joint_id]
        center = self.soft_centers[joint_id]
        
        # CRITICAL: Detect movement direction relative to center
        movement_delta = current_angle - prev_angle
        
        # Determine if moving away from or toward center
        moving_away_from_center = False
        if current_angle > center and movement_delta > 0:  # Right of center, moving right (away)
            moving_away_from_center = True
        elif current_angle < center and movement_delta < 0:  # Left of center, moving left (away)
            moving_away_from_center = True
        
        # If moving TOWARD center, reduce resistance significantly or skip
        if not moving_away_from_center and abs(movement_delta) > 0.3:
            # Moving toward center - allow free movement
            if self.debug_joint == joint_id:
                print(f"  → MOVING TOWARD CENTER - Allowing free movement (no resistance)")
            return False, current_angle, self.base_duration, 0.0, "RETURNING"
        
        # Check for escape attempt FIRST
        is_escaping = self.detect_escape_attempt(joint_id, current_angle, prev_angle, 
                                                  self.calculate_magnetization_strength_cubic(
                                                      current_angle, min_limit, max_limit, joint_id))
        
        # Calculate magnetization strength (cubic curve, reduced if escaping)
        strength = self.calculate_magnetization_strength_cubic(
            current_angle, min_limit, max_limit, joint_id
        )
        
        # Update state based on strength
        if is_moving and velocity > 2.0:
            state = "MOVING"
        elif strength < 0.2:
            state = "NORMAL"
        elif strength < 0.6:
            state = "MID_RANGE"
        else:
            state = "NEAR_LIMIT"
        
        if is_escaping:
            state = "ESCAPING"
        
        self.joint_states[joint_id] = state
        
        # If strength too low, no magnetization needed
        if strength < self.min_strength_threshold:
            if self.debug_joint == joint_id:
                print(f"  Strength too low ({strength:.3f} < {self.min_strength_threshold:.3f}) - No magnetization")
            self.last_commanded[joint_id] = current_angle
            return False, current_angle, self.base_duration, strength, state
        
        # Calculate resistance position
        resistance_position, _ = self.calculate_resistance_position_adaptive(
            current_angle, strength, min_limit, max_limit, joint_id
        )
        
        # Check if resistance position is close to current (prevents micro-adjustments)
        position_difference = abs(resistance_position - current_angle)
        if position_difference < 2.0 and not is_moving:  # Increased from 1.0
            # Too close to bother commanding, and joint is stationary
            if self.debug_joint == joint_id:
                print(f"  → Position difference too small ({position_difference:.2f}°) - Skipping")
            return False, current_angle, self.base_duration, strength, state
        
        # Dynamic duration: stronger resistance = faster response
        duration = int(self.base_duration * (1.0 - 0.5 * strength))
        duration = max(100, min(400, duration))
        
        self.last_commanded[joint_id] = resistance_position
        
        should_command = True
        
        if self.debug_joint == joint_id:
            print(f"  Final: command={should_command}, target={resistance_position:.1f}°, "
                  f"duration={duration}ms, strength={strength:.3f}, state={state}")
            
            # Summary visualization
            print(f"\n  [SUMMARY]")
            print(f"    Angle: {current_angle:6.1f}° → {resistance_position:6.1f}° ({position_difference:5.1f}° move)")
            print(f"    Strength: {strength:.3f} ({'█' * int(strength * 20):20})")
            print(f"    State: {state}")
            print(f"    Direction: {'AWAY from center' if moving_away_from_center else 'TOWARD center'}")
            
            print(f"{'='*60}")
        
        return should_command, resistance_position, duration, strength, state

    def calculate_movement_duration(self, base_duration, strength):
        """Dynamic duration based on strength"""
        min_duration = 100
        max_duration = 400
        duration = max_duration - (strength * (max_duration - min_duration))
        return int(duration)

    def display_joint_info(self, joint_id, current_angle, strength, state, push_count):
        """Display joint information with color coding"""
        joint_name = self.JointToID.get(joint_id, f"Joint_{joint_id}")
        
        colors = {
            "NORMAL": "\033[32m",      # Green
            "MID_RANGE": "\033[33m",   # Yellow
            "NEAR_LIMIT": "\033[91m",  # Light red
            "ESCAPING": "\033[36m",    # Cyan
            "MOVING": "\033[35m",      # Magenta
            "RETURNING": "\033[92m",   # Bright green
            "DISABLED": "\033[90m"     # Gray
        }
        
        color = colors.get(state, "\033[0m")
        reset = "\033[0m"
        
        if strength > 0:
            mag_indicator = "█" * int(strength * 10) + "░" * (10 - int(strength * 10))
            mag_text = f" [{mag_indicator}] {strength:.2f}"
        else:
            mag_text = " " * 20
        
        state_abbr = {
            "NORMAL": "NORM",
            "MID_RANGE": "MID ",
            "NEAR_LIMIT": "NEAR",
            "ESCAPING": "ESC!",
            "MOVING": "MOV→",
            "RETURNING": "RET←",
            "DISABLED": "OFF "
        }.get(state, state[:4])
        
        escape_indicator = "Escape" if state == "ESCAPING" else ""
        moving_indicator = "Moving" if state == "MOVING" else ""
        returning_indicator = "Returning" if state == "RETURNING" else ""
        
        print(f"{color}{joint_name:<10}: {current_angle:6.1f}° [{state_abbr}]{mag_text}{escape_indicator}{moving_indicator}{returning_indicator}{reset}")

# Example usage with debugger:
if __name__ == "__main__":
    # Create config with debug on Joint 4 (Elbow)
    config = MagnetizationConfig(debug_joint=4, debug_strength_interval=5)
    
    # Print strength profile
    config.print_strength_profile(4)
    
    # Simulate movement through the joint range
    print("\n\nSimulating movement from -119° to -50°:")
    for angle in range(-119, -49, 1):
        # Add some randomness to simulate user movement
        simulated_prev = angle - 1.5 + (0.5 if angle % 3 == 0 else 0)
        
        result = config.apply_magnetization_to_joint(4, angle, simulated_prev)
        should_command, target, duration, strength, state = result
        
        time.sleep(0.02)  # Small delay for readability