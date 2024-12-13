#!/usr/bin/env python3
import paramiko
import time
import os

#C:/Python312/python.exe C:\Users\charl\Desktop\OXYCSComps\testAngleCalc\ev3-ssh-control.py

# Get the absolute path to the script's directory
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
ANGLES_FILE = os.path.join(SCRIPT_DIR, 'robot_angles.txt')
TRIGGER_FILE = os.path.join(SCRIPT_DIR, 'new_positions.trigger')

def get_motor_input(prompt, default=None):
    """Helper function to get user input with default value"""
    if default is not None:
        value = input(f"{prompt} [{default}]: ").strip()
        return value if value else default
    return input(f"{prompt}: ").strip()

def read_target_angle():
    """Prompt user for target angle between 0 and 180"""
    while True:
        try:
            angle = int(input("Enter target angle (0 to 180 degrees): "))
            if 0 <= angle <= 180:
                return angle
            else:
                print("Angle must be between 0 and 180 degrees")
        except ValueError:
            print("Please enter a valid integer")

def update_position(ssh, position):
    """Update the current position in a file"""
    ssh.exec_command(f'echo {position} > /home/robot/robot_arm/current_position.txt')

def get_current_position(ssh):
    """Get the current position from file"""
    stdin, stdout, stderr = ssh.exec_command('cat /home/robot/robot_arm/current_position.txt')
    pos = stdout.read().decode().strip()
    return int(pos) if pos else 0

def initialization(ssh):
    """Run robot initialization"""
    print("Starting robot initialization...")
    # Clear any existing position files
    ssh.exec_command('rm -f /home/robot/robot_arm/current_position.txt')
    ssh.exec_command('rm -f /home/robot/robot_arm/new_positions.trigger')
    
    # Run initialization
    stdin, stdout, stderr = ssh.exec_command('brickrun /home/robot/robot_arm/init.py')
    while True:
        line = stdout.readline()
        if not line:
            break
        print(line.strip())
    
    # Set position to 0 degrees
    update_position(ssh, 0)
    print("Robot initialized at 20 degrees forward from touch sensor")
    return True

def check_arm_safe(ssh):
    """Check if robot arm is in safe position before movement"""
    stdin, stdout, stderr = ssh.exec_command('python3 -c "from pybricks.ev3devices import ColorSensor, Port; \
        print(ColorSensor(Port.S3).reflection())"')
    try:
        reflection = int(stdout.read().decode().strip())
        return reflection >= 32
    except:
        return False

def adjust_angle(angle):
    """Add 5 degrees to angles over 90 degrees"""
    return angle + 7 if angle > 45 else angle

def perform(ssh):
    """Coordinate pick and place operation between two angles"""
    try:
        # Check arm position FIRST
        stdin, stdout, stderr = ssh.exec_command('brickrun -r -- pybricks-micropython -c "from pybricks.ev3devices import ColorSensor; from pybricks.parameters import Port; print(ColorSensor(Port.S3).reflection())"')
        reflection = stdout.read().decode().strip()
        if reflection and int(reflection) < 32:
            print("Arm not in safe position. Please run initialization first.")
            return
        
        print("Enter two angles between 0 and 180 degrees:")
        current_angle = read_target_angle()
        target_angle = read_target_angle()
        
        # Adjust angles if they're over 90 degrees
        adjusted_current = adjust_angle(current_angle)
        adjusted_target = adjust_angle(target_angle)
        
        robot_position = get_current_position(ssh)
        
        # Execute pick operation
        print(f"1. Moving to pickup position (angle: {adjusted_current})")
        stdin, stdout, stderr = ssh.exec_command(f'brickrun /home/robot/robot_arm/pick.py {adjusted_current} {robot_position}')
        while True:
            line = stdout.readline()
            if not line:
                break
            print(line.strip())
        update_position(ssh, adjusted_current)
        time.sleep(4)
        
        # Execute release operation
        print(f"2. Moving to release position (angle: {adjusted_target})")
        stdin, stdout, stderr = ssh.exec_command(f'brickrun /home/robot/robot_arm/release.py {adjusted_target} {adjusted_current}')
        while True:
            line = stdout.readline()
            if not line:
                break
            print(line.strip())
        update_position(ssh, adjusted_target)
        time.sleep(4)
        
        # Return to init position
        initialization(ssh)

        print("\nOperation sequence completed!")

    except Exception as e:
        print(f"Error during perform operation: {e}")


def update_position(ssh, position):
    """Update the current position in a file"""
    ssh.exec_command(f'echo {position} > /home/robot/robot_arm/current_position.txt')

def get_current_position(ssh):
    """Get the current position from file"""
    stdin, stdout, stderr = ssh.exec_command('cat /home/robot/robot_arm/current_position.txt')
    pos = stdout.read().decode().strip()
    return int(pos) if pos else 0

def read_angles_from_camera():
    """Read angles from the file written by camera script"""
    try:
        with open(ANGLES_FILE, 'r') as f:
            current_angle, target_angle = map(int, f.read().split(','))
        return current_angle, target_angle
    except:
        return None, None

def check_for_new_positions():
    """Check if new positions are available"""
    try:
        if os.path.exists(TRIGGER_FILE):
            os.remove(TRIGGER_FILE)
            current_angle, target_angle = read_angles_from_camera()
            if current_angle is not None and target_angle is not None:
                print(f"\nNew positions detected:")
                print(f"Current position: {current_angle}°")
                print(f"Target position: {target_angle}°")
                return current_angle, target_angle
    except:
        pass
    return None, None

def monitor_and_execute(ssh):
    """Monitor for new positions and execute robot movements"""
    print("Monitoring for position changes... (Ctrl+C to stop)")
    robot_position = get_current_position(ssh)
    if robot_position != 0:
        print("Warning: Robot position may be incorrect.")
        print("Please run 'init' command before monitoring.")
        return
    
    print(f"Starting monitor mode. Robot position: {robot_position}")
    
    try:
        while True:
            current_angle, target_angle = check_for_new_positions()
            if current_angle is not None and target_angle is not None:
                # Adjust angles if they're over 90 degrees
                adjusted_current = adjust_angle(current_angle)
                adjusted_target = adjust_angle(target_angle)
                
                robot_position = get_current_position(ssh)
                
                # Execute pick operation
                print(f"1. Moving to pickup position (angle: {adjusted_current})")
                stdin, stdout, stderr = ssh.exec_command(f'brickrun /home/robot/robot_arm/pick.py {adjusted_current} {robot_position}')
                while True:
                    line = stdout.readline()
                    if not line:
                        break
                    print(line.strip())
                update_position(ssh, adjusted_current)
                time.sleep(4)
                
                # Execute release operation
                print(f"2. Moving to release position (angle: {adjusted_target})")
                stdin, stdout, stderr = ssh.exec_command(f'brickrun /home/robot/robot_arm/release.py {adjusted_target} {adjusted_current}')
                while True:
                    line = stdout.readline()
                    if not line:
                        break
                    print(line.strip())
                update_position(ssh, adjusted_target)
                time.sleep(4)
                
                # Return to init position
                initialization(ssh)
                
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nMonitoring stopped")

def handle_command(ssh, command):
    """Handle special commands"""
    try:
        parts = command.split()
        if not parts:
            return False

        if parts[0] == "init":
            initialization(ssh)
            return True
            
        elif parts[0] == "perform":
            perform(ssh)
            return True
            
        elif parts[0] == "monitor":
            monitor_and_execute(ssh)
            return True
            
        elif parts[0] == "reset":
            print("Resetting position to 0...")
            update_position(ssh, 0)
            return True
            
        elif parts[0] == "help":
            print("\nCommands:")
            print("  init         - Initialize robot (must run first)")
            print("  perform      - Execute pick and place operation")
            print("  monitor      - Start monitoring camera for positions")
            print("  reset        - Reset position tracking to 0")
            print("  help         - Show this help message")
            return True

        return False

    except Exception as e:
        print(f"Error executing command: {e}")
        return True

def interactive_ssh_terminal(hostname, username, password):
    ssh = paramiko.SSHClient()
    ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        print(f"Connecting to {hostname}...")
        ssh.connect(hostname=hostname, username=username, password=password)
        print("Connected! Type 'exit' to quit, 'help' for commands.")
        print("-" * 50)
        
        while True:
            command = input("ev3> ").strip()
            
            if command.lower() == 'exit':
                print("Disconnecting...")
                # Clean up position files before exiting
                ssh.exec_command('rm -f /home/robot/robot_arm/current_position.txt')
                ssh.exec_command('rm -f /home/robot/robot_arm/new_positions.trigger')
                break
            
            if handle_command(ssh, command):
                continue
            
            # Execute regular command
            stdin, stdout, stderr = ssh.exec_command(command)
            
            output = stdout.read().decode()
            error = stderr.read().decode()
            
            if output:
                print(output.rstrip())
            if error:
                print("Error:", error.rstrip())
    
    except paramiko.AuthenticationException:
        print("Authentication failed. Please check your username and password.")
    except paramiko.SSHException as ssh_exception:
        print(f"SSH exception occurred: {ssh_exception}")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        ssh.close()
        print("Disconnected from EV3")

if __name__ == "__main__":
    ev3_hostname = "ev3dev.local"  # or IP address
    ev3_username = "robot"         # default username
    ev3_password = "maker"         # your password
    
    interactive_ssh_terminal(ev3_hostname, ev3_username, ev3_password)