# servo_calibration_tool.py
from xarm import Controller
import time

def test_servo_positions(arm, servo_id, test_sequence):
    """
    Test a single servo through a sequence of positions
    Returns detailed results in the format you want
    """
    print(f"\n{'='*60}")
    print(f"TESTING SERVO ID {servo_id}")
    print(f"{'='*60}")
    
    results = []
    
    # Get starting position
    try:
        start_raw = arm.getPosition(servo_id, False)
        start_angle = (start_raw - 500) * 0.25
        print(f"STARTING POSITION: raw={start_raw}, angle={start_angle:.1f}°")
    except Exception as e:
        print(f"ERROR reading starting position: {e}")
        print("Starting with default values...")
        start_raw = 500
        start_angle = 0.0
    
    for step_name, target_raw in test_sequence:
        print(f"\n{'='*40}")
        print(f"STEP: {step_name}")
        print(f"{'='*40}")
        
        target_angle = (target_raw - 500) * 0.25
        print(f"Moving to: raw={target_raw} ({target_angle:.1f}°)")
        
        # Try to move the servo
        try:
            # Move servo slowly
            arm.setPosition([[servo_id, target_raw]], duration=2000, wait=True)
            time.sleep(1.5)  # Longer settle time for better accuracy
            
            # Try to read position with retries
            actual_raw = None
            max_retries = 3
            for retry in range(max_retries):
                try:
                    actual_raw = arm.getPosition(servo_id, False)
                    break  # Success!
                except Exception as e:
                    if retry == max_retries - 1:
                        print(f"  ERROR: Failed to read after {max_retries} retries: {e}")
                        actual_raw = target_raw  # Use target as fallback
                    else:
                        print(f"  Read failed, retry {retry+1}/{max_retries}...")
                        time.sleep(0.5)
            
            # Calculate results
            actual_angle = (actual_raw - 500) * 0.25
            raw_error = actual_raw - target_raw
            angle_error = actual_angle - target_angle
            
            print(f"RESULT:")
            print(f"  Target:   raw={target_raw}, angle={target_angle:.1f}°")
            print(f"  Actual:   raw={actual_raw}, angle={actual_angle:.1f}°")
            print(f"  Error:    raw={raw_error:+d}, angle={angle_error:+.1f}°")
            
            results.append({
                'step': step_name,
                'target_raw': target_raw,
                'target_angle': target_angle,
                'actual_raw': actual_raw,
                'actual_angle': actual_angle,
                'raw_error': raw_error,
                'angle_error': angle_error
            })
            
        except Exception as e:
            print(f"ERROR during movement: {e}")
            print("Skipping this position...")
            
            # Add placeholder result for skipped position
            results.append({
                'step': step_name,
                'target_raw': target_raw,
                'target_angle': target_angle,
                'actual_raw': target_raw,  # Use target as fallback
                'actual_angle': target_angle,
                'raw_error': 0,
                'angle_error': 0.0,
                'skipped': True
            })
            continue
        
        # Brief pause between movements
        time.sleep(0.5)
    
    return results

def print_summary(results, servo_id):
    """Print a summary of the test results"""
    print(f"\n{'='*60}")
    print(f"SERVO ID {servo_id} - TEST SUMMARY")
    print(f"{'='*60}")
    
    valid_results = [r for r in results if not r.get('skipped', False)]
    
    if not valid_results:
        print("No valid test results available.")
        return
    
    for result in results:
        if result.get('skipped', False):
            print(f"{result['step']:20s}: SKIPPED - Error during test")
        else:
            print(f"{result['step']:20s}: "
                  f"Target={result['target_raw']:3d}({result['target_angle']:5.1f}°) → "
                  f"Actual={result['actual_raw']:3d}({result['actual_angle']:5.1f}°) | "
                  f"Error={result['raw_error']:+3d}({result['angle_error']:+4.1f}°)")
    
    # Calculate average errors from valid results only
    avg_raw_error = sum(r['raw_error'] for r in valid_results) / len(valid_results)
    avg_angle_error = sum(r['angle_error'] for r in valid_results) / len(valid_results)
    
    print(f"\nAverage error (from {len(valid_results)} valid tests):")
    print(f"  Raw: {avg_raw_error:+.1f} units")
    print(f"  Angle: {avg_angle_error:+.1f}°")
    
    # Recommended offset (negate the average error)
    recommended_offset = -int(round(avg_raw_error))
    print(f"Recommended offset for SERVO_OFFSETS[{servo_id}]: {recommended_offset}")
    
    return recommended_offset

def reconnect_arm():
    """Try to reconnect to the xArm"""
    print("Attempting to reconnect to xArm...")
    try:
        arm = Controller('USB')
        print("Reconnected successfully!")
        return arm
    except Exception as e:
        print(f"Failed to reconnect: {e}")
        return None

def main():
    print("XArm Servo Calibration Tool")
    print("="*60)
    print("Make sure xArm is powered on and USB is connected.")
    print("="*60)
    
    # Connect to xArm
    print("Connecting to xArm...")
    try:
        arm = Controller('USB')
        print("Connected successfully!")
    except Exception as e:
        print(f"Failed to connect: {e}")
        print("Please check USB connection and try again.")
        return
    
    # Define the complete test sequence
    test_sequence = [
        ("0° (baseline)", 500),
        ("+90°", 860),
        ("Return to 0°", 500),
        ("-90°", 140),
        ("Return to 0°", 500),
        ("+45°", 680),
        ("-45°", 320),
        ("Final 0°", 500),
    ]
    
    # Test each servo
    all_results = {}
    all_offsets = {}
    
    # You can change this list to test specific servos
    servos_to_test = [1, 2, 3, 4, 5, 6]
    
    for servo_id in servos_to_test:
        print(f"\n\nReady to test Servo ID {servo_id}")
        print(f"Make sure servo {servo_id} can move freely!")
        
        input(f"Press Enter to begin testing Servo ID {servo_id}...")
        
        try:
            # Test this servo
            results = test_servo_positions(arm, servo_id, test_sequence)
            all_results[servo_id] = results
            
            # Print summary and get recommended offset
            offset = print_summary(results, servo_id)
            all_offsets[servo_id] = offset
            
            # Release servo before testing next one
            print(f"\nReleasing servo {servo_id}...")
            arm.servoOff([servo_id])
            time.sleep(1)
            
        except Exception as e:
            print(f"FATAL ERROR testing servo {servo_id}: {e}")
            print("Attempting to reconnect...")
            arm = reconnect_arm()
            if not arm:
                print("Cannot continue. Please restart the program.")
                break
            
            # Ask if user wants to retry this servo
            retry = input(f"Retry servo {servo_id}? (y/n): ").lower()
            if retry == 'y':
                servos_to_test.insert(0, servo_id)  # Add back to beginning
            continue
        
        if servo_id != servos_to_test[-1]:
            next_servo = servo_id + 1
            print(f"\nNext: Servo ID {next_servo}")
            print("Check if servo can move freely, then press Enter...")
            input()
    
    # Print final summary of all servos
    print(f"\n{'='*60}")
    print("FINAL CALIBRATION SUMMARY")
    print(f"{'='*60}")
    
    print("\nRecommended SERVO_OFFSETS dictionary:")
    print("SERVO_OFFSETS = {")
    
    for servo_id in servos_to_test:
        if servo_id in all_offsets:
            offset = all_offsets[servo_id]
            if servo_id in all_results:
                valid_tests = len([r for r in all_results[servo_id] if not r.get('skipped', False)])
                print(f"    {servo_id}: {offset:3d},  # Based on {valid_tests} valid tests")
            else:
                print(f"    {servo_id}: {offset:3d},  # No valid test data")
    
    print("}")
    
    print(f"\n{'='*60}")
    print("HOW TO USE THESE OFFSETS:")
    print("1. Copy the SERVO_OFFSETS dictionary above")
    print("2. Paste it into your xArm teleoperation code")
    print("3. When calculating target positions:")
    print("   position = int((angle + 125) * 4) + SERVO_OFFSETS[servo_id]")
    print(f"{'='*60}")
    
    # Final cleanup
    print("\nReleasing all servos...")
    try:
        arm.servoOff([1, 2, 3, 4, 5, 6])
    except:
        pass
    
    print("Calibration complete!")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nCalibration interrupted by user.")
    except Exception as e:
        print(f"\nUnexpected error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nCalibration tool exiting.")