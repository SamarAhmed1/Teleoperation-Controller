
from ForcesAddition import MagnetizationConfig
from xarm import Controller, Servo  
import time

def main():
    # Initialize magnetization configuration with debug for Base joint (6)
    config = MagnetizationConfig(debug_joint=6)  # Set to 6 for Base joint

    arm = Controller('USB')
    prev_angles = [0.0] * 6
    order = [6, 5, 4, 3, 2, 1]
    last_command_time = {joint_id: 0 for joint_id in order}
    
    print("=" * 70)
    print("XArm Magnetization System - DEBUG MODE")
    print("Debugging Base joint (6)")
    print("=" * 70)
    
    try:
        while True:
            curr_angles = []
            changed = False
            
            # Read all joint positions
            for joint_id in order:
                angle = arm.getPosition(joint_id, degrees=True)
                curr_angles.append(angle)
            
            # Check for significant changes
            for prev, curr in zip(prev_angles, curr_angles):
                if abs(prev - curr) > 0.3:
                    changed = True
                    break
            
            # Apply magnetization and display if changed
            if changed:
                print(f"\n{'Joint':<10} {'Angle':<7} {'State':<6} {'Magnetization':<15}")
                print("-" * 60)
                
                for i, joint_id in enumerate(order):
                    current_angle = curr_angles[i]
                    prev_angle = prev_angles[i]
                    
                    # Apply magnetization to this joint #Target_angle is resistance_position, should_command = true
                    should_command, target_angle, duration, strength, state = \
                        config.apply_magnetization_to_joint(joint_id, current_angle, prev_angle)
                    
                    # Display joint information (only for non-debug joints or all)
                    config.display_joint_info(
                        joint_id, current_angle, strength, state, 
                        config.push_counts[joint_id]
                    )
                    
                    if state == "ESCAPING":
                        print(f"  [ESCAPE] Using servoOff() to help escape from limit")
                        try:
                            arm.servoOff(joint_id)  # Turn off this servo
                            # Wait a moment for user to move it
                            time.sleep(0.1)
                            # Re-enable with lower resistance
                            # You could set a flag to reduce resistance temporarily
                        except:
                            print(f"  [ESCAPE ERROR] Failed to use servoOff")
                        continue  # Skip the normal command
                    # Send resistance command if needed
                    current_time = time.time()
                    if should_command and strength > 0:
                        if current_time - last_command_time[joint_id] > 0.1:  # 100ms between commands
                            try:
                                arm.setPosition(joint_id, target_angle, duration=duration, wait=False)
                                last_command_time[joint_id] = current_time
                            except:
                                arm.setPosition(joint_id, target_angle)
            
            # Update previous angles
            prev_angles = curr_angles[:]
            
            # Small delay
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        print("Magnetization System Stopped")
        print("Push Event Summary:")
        for joint_id in config.enabled_joints:
            if config.push_counts[joint_id] > 0:
                print(f"  {config.JointToID[joint_id]}: {config.push_counts[joint_id]} push events")
        print("=" * 70)
        arm.servoOff()
        
    except Exception as e:
        print(f"\nError: {e}")
        print("Turning off all servos for safety...")
        arm.servoOff()


if __name__ == "__main__":
    main()
