#!/usr/bin/env python3

def test_angles():
    """Test function for simulating OpenCV angle detection"""
    print("\nTEST MODE - Simulating Angle Detection")
    print("Current format: current_angle,target_angle")
    print("Example: 90,30 means move from 90 to 30 degrees")
    print("Type 'done' when finished testing\n")
    
    while True:
        command = input("Enter angles (current,target) or 'done': ")
        if command.lower() == 'done':
            break
            
        try:
            # Parse input
            current_angle, target_angle = map(int, command.split(','))
            if not (0 <= current_angle <= 180 and 0 <= target_angle <= 180):
                print("Angles must be between 0 and 180 degrees")
                continue
            
            # Write to angles file
            with open('robot_angles.txt', 'w') as f:
                f.write(f"{current_angle},{target_angle}")
            # Create trigger file
            with open('new_positions.trigger', 'w') as f:
                f.write('1')
                
            print(f"Test angles set: {current_angle}° → {target_angle}°")
            print("Waiting for robot to complete movement...")
            input("Press Enter when ready for next test...")
                
        except ValueError:
            print("Invalid format. Use: current_angle,target_angle")
            continue

if __name__ == "__main__":
    test_angles()