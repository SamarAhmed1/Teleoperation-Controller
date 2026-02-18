from Test import MagnetizationConfig
from xarm import Controller, Servo  
import time

def main():
    # Initialize magnetization configuration
    # Set debug_joint=6 for Base joint, or None for no debug output
    config = MagnetizationConfig(debug_joint=6, debug_strength_interval=5)  # Change to 6 to debug Base joint

    # Print strength profile for debug joint at startup
    if config.debug_joint is not None:
        config.print_strength_profile(config.debug_joint)
    
    arm = Controller('USB')
    prev_angles = [0.0] * 6
    order = [6, 5, 4, 3, 2, 1]
    last_command_time = {joint_id: 0 for joint_id in order}
    last_display_time = time.time()
    
    print("=" * 70)
    print("XArm Magnetization System - CUBIC RESISTANCE v2.0")
    print("Features:")
    print("  • Smooth cubic resistance curve (gentle → moderate → strong)")
    print("  • Fast response (50ms update rate)")
    print("  • Auto-escape mode (push toward center to reduce resistance)")
    print("  • Progressive pushback (max 8° from current position)")
    print("=" * 70)
    print("\nPress Ctrl+C to stop\n")
    
    # Track strength history for all joints
    strength_log = {jid: [] for jid in order}
    max_log_length = 50
    
    try:
        while True:
            current_time = time.time()
            curr_angles = []
            changed = False
            
            # Read all joint positions
            for joint_id in order:
                try:
                    angle = arm.getPosition(joint_id, degrees=True)
                    curr_angles.append(angle)
                except:
                    # If read fails, use previous angle
                    curr_angles.append(prev_angles[order.index(joint_id)])
            
            # Check for significant changes (lower threshold for faster response)
            for prev, curr in zip(prev_angles, curr_angles):
                if abs(prev - curr) > 0.2:  # Reduced from 0.3 for faster detection
                    changed = True
                    break
            
            # Display update every 0.5 seconds or on change
            should_display = changed or (current_time - last_display_time > 0.5)
            
            if should_display:
                print(f"\n{'Joint':<10} {'Angle':<7} {'State':<6} {'Magnetization':<20}")
                print("-" * 60)
                last_display_time = current_time
            
            # Process each joint
            for i, joint_id in enumerate(order):
                current_angle = curr_angles[i]
                prev_angle = prev_angles[i]
                
                # Apply magnetization to this joint
                should_command, target_angle, duration, strength, state = \
                    config.apply_magnetization_to_joint(joint_id, current_angle, prev_angle)
                
                # Log strength for analysis
                strength_log[joint_id].append((current_time, current_angle, strength))
                if len(strength_log[joint_id]) > max_log_length:
                    strength_log[joint_id].pop(0)
                
                # Display joint information
                if should_display:
                    config.display_joint_info(
                        joint_id, current_angle, strength, state, 
                        config.push_counts[joint_id]
                    )
                
                # Send resistance command if needed
                if should_command and strength > config.min_strength_threshold:
                    # Check timing (respect minimum interval)
                    if current_time - last_command_time[joint_id] >= config.min_command_interval:
                        try:
                            # Send command with wait=False for non-blocking
                            arm.setPosition(joint_id, target_angle, duration=500, wait=False)
                            last_command_time[joint_id] = current_time
                            
                            # Enhanced debug for the debug joint
                            if config.debug_joint == joint_id:
                                print(f"\n[COMMAND SENT] Joint {joint_id}: {current_angle:.1f}° → {target_angle:.1f}° "
                                      f"({target_angle-current_angle:+.1f}°) in {duration}ms")
                        except Exception as e:
                            # Fallback: try without duration parameter
                            try:
                                arm.setPosition(joint_id, target_angle, wait=False)
                                last_command_time[joint_id] = current_time
                            except:
                                pass  # Silently skip if command fails
            
            # Update previous angles
            prev_angles = curr_angles[:]
            
            # Fast loop: 50ms (20Hz update rate)
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        print("Magnetization System Stopped")
        print("\nSession Summary:")
        print("-" * 70)
        
        total_pushes = sum(config.push_counts.values())
        if total_pushes > 0:
            print("Push Events Detected:")
            for joint_id in config.enabled_joints:
                if config.push_counts[joint_id] > 0:
                    print(f"  {config.JointToID[joint_id]}: {config.push_counts[joint_id]} events")
        else:
            print("No push events detected during session")
        
        print("\nFinal Joint States:")
        for joint_id in config.enabled_joints:
            state = config.joint_states[joint_id]
            print(f"  {config.JointToID[joint_id]}: {state}")
        
        # Print strength analysis for debug joint
        if config.debug_joint is not None and strength_log[config.debug_joint]:
            print(f"\nStrength Analysis for {config.JointToID[config.debug_joint]}:")
            print("-" * 70)
            
            # Find min, max, average strength
            strengths = [s for _, _, s in strength_log[config.debug_joint]]
            if strengths:
                angles = [a for _, a, _ in strength_log[config.debug_joint]]
                min_strength_idx = strengths.index(min(strengths))
                max_strength_idx = strengths.index(max(strengths))
                
                print(f"  Minimum strength: {min(strengths):.3f} at {angles[min_strength_idx]:.1f}°")
                print(f"  Maximum strength: {max(strengths):.3f} at {angles[max_strength_idx]:.1f}°")
                print(f"  Average strength: {sum(strengths)/len(strengths):.3f}")
                
                # Count strength ranges
                low_count = sum(1 for s in strengths if s < 0.2)
                mid_count = sum(1 for s in strengths if 0.2 <= s < 0.6)
                high_count = sum(1 for s in strengths if s >= 0.6)
                
                print(f"\n  Strength Distribution:")
                print(f"    Low (<0.2):   {low_count:3d} samples ({low_count/len(strengths)*100:.1f}%)")
                print(f"    Mid (0.2-0.6): {mid_count:3d} samples ({mid_count/len(strengths)*100:.1f}%)")
                print(f"    High (≥0.6):  {high_count:3d} samples ({high_count/len(strengths)*100:.1f}%)")
        
        print("=" * 70)
        print("\nSafely turning off servos...")
        arm.servoOff()
        print("Done. Goodbye!")
        
    except Exception as e:
        print(f"\n{'='*70}")
        print(f"ERROR: {e}")
        import traceback
        traceback.print_exc()
        print("=" * 70)
        print("Turning off all servos for safety...")
        try:
            arm.servoOff()
            print("Servos turned off successfully")
        except:
            print("Warning: Could not turn off servos")
        print("=" * 70)


if __name__ == "__main__":
    main()