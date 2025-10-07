'''
Some utilities for UIUC Hand that help with converting joint angles between each convention.
'''
from re import L
import numpy as np


'''
Embodiments:

uiuc_hand: Real UIUC hand (180 for the motor is actual zero)
LEAPsim:  UIUC hand in sim (has allegro-like zero positions)
one_range: [-1, 1] for all joints to facilitate RL
allegro:  Allegro hand in real or sim
'''

#Safety clips all joints so nothing unsafe can happen. Highly recommend using this before commanding
def angle_safety_clip(joints):
    sim_min, sim_max = LEAPsim_limits()
    real_min = LEAPsim_to_uiuc_hand(sim_min)
    real_max = LEAPsim_to_uiuc_hand(sim_max)
    return np.clip(joints, real_min, real_max)

###Sometimes it's useful to constrain the thumb more heavily(you have to implement here), but regular usually works good.
def LEAPsim_limits(type = "regular"):
    global _custom_limits
    # Use custom limits if available (from calibration)
    if _custom_limits is not None:
        return _custom_limits
    
    # Otherwise use default limits
    if type == "regular":
        sim_min = np.array([-1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -1.047, -0.314, -0.506, -0.366, -0.349, -0.47, -1.20, -1.34])
        sim_max = np.array([1.047,    2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  1.047,   2.23,  1.885,  2.042,  2.094,  2.443, 1.90,  1.88])
    return sim_min, sim_max

#this goes from [-1, 1] to [lower, upper]
def scale(x, lower, upper):
    return (0.5 * (x + 1.0) * (upper - lower) + lower)
#this goes from [lower, upper] to [-1, 1]
def unscale(x, lower, upper):
    return (2.0 * x - upper - lower)/(upper - lower)

#-----------------------------------------------------------------------------------
#Isaac has custom ranges from -1 to 1 so we convert that to UIUC hand real world
def sim_ones_to_uiuc_hand(joints, hack_thumb = False):
    sim_min, sim_max = LEAPsim_limits(type = hack_thumb)
    joints = scale(joints, sim_min, sim_max)
    joints = LEAPsim_to_uiuc_hand(joints)
    return joints
#UIUC hand real world to Isaac has custom ranges from -1 to 1
def uiuc_hand_to_sim_ones(joints, hack_thumb = False):  
    joints = uiuc_hand_to_LEAPsim(joints)
    sim_min, sim_max = LEAPsim_limits(type = hack_thumb)
    joints = unscale(joints, sim_min, sim_max)
    return joints

#-----------------------------------------------------------------------------------
###Sim UIUC hand to real uiuc hand  Sim is allegro-like but all 16 joints are usable.
def LEAPsim_to_uiuc_hand(joints):
    joints = np.array(joints)
    ret_joints = joints + 3.14159
    return ret_joints
###Real UIUC hand to sim uiuc hand  Sim is allegro-like but all 16 joints are usable.
def uiuc_hand_to_LEAPsim(joints):
    joints = np.array(joints)
    ret_joints = joints - 3.14159
    return ret_joints

#-----------------------------------------------------------------------------------
#Converts allegrohand radians to LEAP (radians)
#Only converts the joints that match, all 4 of the thumb and the outer 3 for each of the other fingers
#All the clockwise/counterclockwise signs are the same between the two hands.  Just the offset (mostly 180 degrees off)
def allegro_to_uiuc_hand(joints, teleop = False, zeros = True):
    joints = np.array(joints)
    ret_joints = joints + 3.14159
    if zeros:
        ret_joints[0] = ret_joints[4] = ret_joints[8] = 3.14
    if teleop:
        ret_joints[12] = joints[12] + 0.2 
        ret_joints[14] = joints[14] - 0.2   
    return ret_joints
# Converts LEAP to allegrohand (radians)
def uiuc_hand_to_allegro(joints, teleop = False, zeros = True):
    joints = np.array(joints)
    ret_joints = joints - 3.14159
    if zeros:
        ret_joints[0] = ret_joints[4] = ret_joints[8] = 0
    if teleop:
        ret_joints[12] = joints[12] - 0.2
        ret_joints[14] = joints[14] + 0.2    
    return ret_joints

#-----------------------------------------------------------------------------------
# Motor limit testing and updating functions

# Global variable to store custom limits
_custom_limits = None

def find_motor_limits(motors_to_test=[1, 2, 3], port='/dev/ttyUSB0', baudrate=57600):
    """
    Test actual motor limits by sending different angle values and checking for errors.
    Returns updated min/max arrays for the specified motors.
    
    Args:
        motors_to_test: List of motor IDs to test (e.g., [1, 2, 3])
        port: Serial port (e.g., '/dev/ttyUSB0')
        baudrate: Baud rate (e.g., 57600)
    
    Returns:
        (sim_min, sim_max): Updated min/max arrays for all 16 motors
    """
    try:
        from .dynamixel_client import DynamixelClient
        import time
        
        print(f"Testing motor limits for motors: {motors_to_test}")
        
        # Initialize Dynamixel client
        dxl_client = DynamixelClient(motors_to_test, port, baudrate)
        dxl_client.connect()
        print("✓ Connected to UIUC Hand")
        
        # Get current limits as starting point
        sim_min, sim_max = LEAPsim_limits()
        
        # Test ranges for each motor (in radians)
        test_ranges = {
            1: np.linspace(-2.0, 2.0, 21),  # Motor 1: Index MCP Side
            2: np.linspace(-1.0, 3.0, 21),  # Motor 2: Index MCP Forward  
            3: np.linspace(-1.0, 3.0, 21),  # Motor 3: Index PIP
            4: np.linspace(-2.0, 2.0, 21),  # Motor 4: Middle MCP Side
            5: np.linspace(-1.0, 3.0, 21),  # Motor 5: Middle MCP Forward
            6: np.linspace(-1.0, 3.0, 21),  # Motor 6: Middle PIP
            7: np.linspace(-1.0, 3.0, 21),  # Motor 7: Middle DIP
            8: np.linspace(-2.0, 2.0, 21),  # Motor 8: Ring MCP Side
            9: np.linspace(-1.0, 3.0, 21),  # Motor 9: Ring MCP Forward
            10: np.linspace(-1.0, 3.0, 21), # Motor 10: Ring PIP
            11: np.linspace(-1.0, 3.0, 21), # Motor 11: Ring DIP
            12: np.linspace(-2.0, 2.0, 21), # Motor 12: Thumb MCP Side
            13: np.linspace(-1.0, 3.0, 21), # Motor 13: Thumb MCP Forward
            14: np.linspace(-1.0, 3.0, 21), # Motor 14: Thumb PIP
            15: np.linspace(-1.0, 3.0, 21), # Motor 15: Thumb DIP
            0: np.linspace(-2.0, 2.0, 21)   # Motor 0: Index MCP Side (alternative)
        }
        
        # Store successful ranges for each motor
        successful_ranges = {}
        
        print("\nTesting individual motor limits...")
        print("Motor | Angle (rad) | Angle (deg) | Status")
        print("------|-------------|-------------|--------")
        
        for motor_id in motors_to_test:
            if motor_id not in test_ranges:
                print(f"Motor {motor_id}: No test range defined, skipping")
                continue
                
            successful_ranges[motor_id] = []
            print(f"\nTesting Motor {motor_id}:")
            
            for angle in test_ranges[motor_id]:
                try:
                    # Try to set the position
                    dxl_client.write_desired_pos([motor_id], [angle])
                    time.sleep(0.05)  # Small delay
                    
                    # Try to read back the position to verify it worked
                    current_pos = dxl_client.read_pos([motor_id])
                    
                    if current_pos is not None and len(current_pos) > 0:
                        successful_ranges[motor_id].append(angle)
                        status = "✓ OK"
                    else:
                        status = "✗ No response"
                        
                except Exception as e:
                    status = f"✗ Error"
                
                print(f"  {motor_id:5d} | {angle:11.3f} | {np.degrees(angle):11.1f} | {status}")
        
        # Update the limits arrays with found values
        for motor_id in motors_to_test:
            if motor_id in successful_ranges and successful_ranges[motor_id]:
                min_angle = min(successful_ranges[motor_id])
                max_angle = max(successful_ranges[motor_id])
                
                # Convert from UIUC hand coordinates to sim coordinates
                sim_min_angle = min_angle - 3.14159
                sim_max_angle = max_angle - 3.14159
                
                # Update the arrays (convert to 0-indexed for array access)
                array_idx = motor_id if motor_id < 16 else motor_id - 16
                if array_idx < len(sim_min):
                    sim_min[array_idx] = sim_min_angle
                    sim_max[array_idx] = sim_max_angle
                    
                print(f"Motor {motor_id}: Updated limits to {sim_min_angle:.3f} to {sim_max_angle:.3f} rad (sim coords)")
        
        dxl_client.disconnect()
        print("\n✓ Motor limit testing complete!")
        
        return sim_min, sim_max
        
    except Exception as e:
        print(f"Error testing motor limits: {e}")
        print("Returning original limits")
        return LEAPsim_limits()

def calibrate_motor_limits(motors_to_calibrate=[1, 2, 3], port='/dev/ttyUSB0', baudrate=57600):
    """
    Manual calibration of motor limits by letting user rotate motors manually.
    This directly updates the global limits and modifies LEAPsim_limits() behavior.
    """
    global _custom_limits
    import time
    import threading
    from .dynamixel_client import DynamixelClient
    
    try:
        print("Starting manual motor calibration...")
        print(f"Calibrating motors: {motors_to_calibrate}")
        
        # Connect to UIUC Hand
        print("Connecting to UIUC Hand...")
        dxl_client = DynamixelClient(motors_to_calibrate, port, baudrate)
        dxl_client.connect()
        print("✓ Connected successfully!")
        
        # Disable torque so user can rotate motors manually
        print("Disabling torque on all motors...")
        dxl_client.set_torque_enabled(motors_to_calibrate, False)
        print("✓ Torque disabled - you can now rotate the motors manually!")
        
        # Initialize tracking variables
        min_values = {motor_id: float('inf') for motor_id in motors_to_calibrate}
        max_values = {motor_id: float('-inf') for motor_id in motors_to_calibrate}
        current_values = {motor_id: 0.0 for motor_id in motors_to_calibrate}
        running = True
        
        def monitor_loop():
            nonlocal running, min_values, max_values, current_values
            print("\n" + "="*60)
            print("CALIBRATION MODE - Rotate the motors manually!")
            print("="*60)
            print("Instructions:")
            print("1. Manually rotate each motor through its full range")
            print("2. Move slowly to ensure we capture all positions")
            print("3. Press Enter when done with all motors")
            print("\nCurrent readings (updating in real-time):")
            print("Motor | Current (rad) | Current (deg) | Min (deg) | Max (deg)")
            print("------|---------------|---------------|-----------|----------")
            
            while running:
                try:
                    # Read current positions
                    positions = dxl_client.read_pos()
                    if positions is not None and len(positions) == len(motors_to_calibrate):
                        for i, motor_id in enumerate(motors_to_calibrate):
                            current_values[motor_id] = positions[i]
                            
                            # Update min/max
                            if positions[i] != 0.0:
                                min_values[motor_id] = min(min_values[motor_id], positions[i])
                                max_values[motor_id] = max(max_values[motor_id], positions[i])
                    
                    # Display current status
                    print("\r", end="")
                    for motor_id in motors_to_calibrate:
                        current_deg = current_values[motor_id] * 180 / 3.14159
                        min_deg = min_values[motor_id] * 180 / 3.14159 if min_values[motor_id] != float('inf') else 0
                        max_deg = max_values[motor_id] * 180 / 3.14159 if max_values[motor_id] != float('-inf') else 0
                        print(f"  {motor_id:5d} | {current_values[motor_id]:13.3f} | {current_deg:13.1f} | {min_deg:9.1f} | {max_deg:9.1f}")
                    
                    time.sleep(0.1)  # Update 10 times per second
                    
                except Exception as e:
                    print(f"Error in monitoring: {e}")
                    break
        
        # Start monitoring in a separate thread
        monitor_thread = threading.Thread(target=monitor_loop)
        monitor_thread.daemon = True
        monitor_thread.start()
        
        # Wait for user to press Enter
        input("\nPress Enter when you're done rotating all motors...")
        
        # Stop monitoring
        running = False
        time.sleep(0.2)  # Give thread time to stop
        
        # Re-enable torque
        print("Re-enabling torque...")
        dxl_client.set_torque_enabled(motors_to_calibrate, True)
        print("✓ Torque re-enabled")
        
        # Convert to sim coordinates and update global limits
        sim_min, sim_max = LEAPsim_limits()
        
        print("\n" + "="*60)
        print("CALIBRATION RESULTS:")
        print("="*60)
        
        for motor_id in motors_to_calibrate:
            if min_values[motor_id] != float('inf') and max_values[motor_id] != float('-inf'):
                # Convert from UIUC hand coordinates to sim coordinates
                sim_min_val = min_values[motor_id] - 3.14159
                sim_max_val = max_values[motor_id] - 3.14159
                
                # Update the arrays (convert to 0-indexed for array access)
                array_idx = motor_id if motor_id < 16 else motor_id - 16
                if array_idx < len(sim_min):
                    sim_min[array_idx] = sim_min_val
                    sim_max[array_idx] = sim_max_val
                
                min_deg = min_values[motor_id] * 180 / 3.14159
                max_deg = max_values[motor_id] * 180 / 3.14159
                sim_min_deg = sim_min_val * 180 / 3.14159
                sim_max_deg = sim_max_val * 180 / 3.14159
                
                print(f"Motor {motor_id}:")
                print(f"  UIUC Hand coords: {min_values[motor_id]:.3f} to {max_values[motor_id]:.3f} rad ({min_deg:.1f}° to {max_deg:.1f}°)")
                print(f"  Sim coords:       {sim_min_val:.3f} to {sim_max_val:.3f} rad ({sim_min_deg:.1f}° to {sim_max_deg:.1f}°)")
                print()
        
        # Update global custom limits
        _custom_limits = (sim_min, sim_max)
        
        print("✓ Motor limits calibrated and updated successfully!")
        print("The LEAPsim_limits() function will now use these calibrated values.")
        
        dxl_client.disconnect()
        return sim_min, sim_max
        
    except Exception as e:
        print(f"Calibration failed: {e}")
        return LEAPsim_limits()

def update_motor_limits(motors_to_test=[1, 2, 3], port='/dev/ttyUSB0', baudrate=57600):
    """
    Update the global motor limits by testing actual hardware.
    This function modifies the LEAPsim_limits() function behavior.
    """
    global _custom_limits
    try:
        sim_min, sim_max = find_motor_limits(motors_to_test, port, baudrate)
        _custom_limits = (sim_min, sim_max)
        print("✓ Motor limits updated successfully!")
        return sim_min, sim_max
    except Exception as e:
        print(f"Failed to update motor limits: {e}")
        return LEAPsim_limits()

def LEAPsim_limits_with_custom(type="regular"):
    """
    Modified version of LEAPsim_limits that uses custom limits if available.
    """
    global _custom_limits
    if _custom_limits is not None:
        return _custom_limits
    else:
        return LEAPsim_limits(type)

#-----------------------------------------------------------------------------------
