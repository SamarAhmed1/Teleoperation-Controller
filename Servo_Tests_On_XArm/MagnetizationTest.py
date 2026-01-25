
from ForcesAddition import MagnetizationConfig
from xarm import Controller, Servo  
import hid
import time

def main():
    # Initialize magnetization configuration
    config = MagnetizationConfig()

    arm = Controller('USB')
    prev_angles = [0.0] * 6
    order = [6, 5, 4, 3, 2, 1]
    # You can disable specific joints if needed:
    # config.enabled_joints = [6, 5, 4]  # Only base, shoulder, elbow
    
    print("=" * 70)
    print("XArm Magnetization System - Active Resistance Near Limits")
    print("=" * 70)
    print(f"Magnetization Zone: {config.magnetization_start}° from limits")
    print(f"Safety Buffer: {config.safety_buffer}° inside physical limits")
    print(f"Enabled Joints: {[config.JointToID[j] for j in config.enabled_joints]}")
    print("=" * 70)
    print("Legend: NORM=Normal, APPR=Approaching, NEAR=Near Limit, PUSH=User Pushing")
    print("-" * 70)
    
    try:
        while True:
            curr_angles = []
            changed = False
            
            # Read all joint positions
            for joint_id in order:
                angle = arm.getPosition(joint_id, degrees=True)
                curr_angles.append(angle)
            
            # Check for significant changes (for display purposes)
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
                    
                    # Apply magnetization to this joint
                    should_command, target_angle, duration, strength, state = \
                        config.apply_magnetization_to_joint(joint_id, current_angle, prev_angle)
                    
                    # Display joint information
                    config.display_joint_info(
                        joint_id, current_angle, strength, state, 
                        config.push_counts[joint_id]
                    )
                    
                    # Send resistance command if needed
                    if should_command and strength > 0:
                        # Convert angle to servo position units
                        # Note: This depends on your Servo class implementation
                        try:
                            # Try to use the setPosition method
                            arm.setPosition(joint_id, target_angle, duration=1000, wait=False)
                        except:
                            # Fallback if setPosition doesn't accept duration parameter
                            arm.setPosition(joint_id, target_angle)
            
            # Update previous angles
            prev_angles = curr_angles[:]
            
            # Small delay to prevent overwhelming the system
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        print("\n" + "=" * 70)
        print("Magnetization System Stopped")
        print("Push Event Summary:")
        for joint_id in config.enabled_joints:
            if config.push_counts[joint_id] > 0:
                print(f"  {config.JointToID[joint_id]}: {config.push_counts[joint_id]} push events")
        print("=" * 70)
        
    except Exception as e:
        print(f"\nError: {e}")
        print("Turning off all servos for safety...")
        arm.servoOff()


if __name__ == "__main__":
    main()